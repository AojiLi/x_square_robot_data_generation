"""CPU-only sampled Revo2 pinch proposals; these are NOT physical grasp results.

Uses the URDF collision-hull surfaces from HandGeometry. Positions and normals
are samples, so penetration/clearance numbers are estimates, not mesh-distance
certificates. A forward-physics trial must establish every claimed contact.
"""
from pathlib import Path
import json
import math
import numpy as np
from scipy.spatial.transform import Rotation
from room01_sim.kinematics import HAND_SUFFIXES
from room01_sim.umi_layout import HandGeometry

ROOT = Path(__file__).resolve().parents[1]
RADIUS = .00625
HALF_LENGTH = .0245


def cylinder_sdf(points):
    p = np.asarray(points)
    q = np.c_[abs(p[:, 0]) - HALF_LENGTH, np.linalg.norm(p[:, 1:], axis=1) - RADIUS]
    return np.linalg.norm(np.maximum(q, 0), axis=1) + np.minimum(q.max(axis=1), 0)


def cloud_at(geometry, motor_rad, normals=False):
    # HandGeometry uses source-device thumb flex/opposition ordering, while
    # this module exposes the URDF motor order defined by HAND_SUFFIXES.
    source_deg = np.rad2deg(np.asarray(motor_rad)[[1, 0, 2, 3, 4, 5]])
    return geometry.at('right', source_deg, np.eye(4), normals=normals)


def evaluate(cloud, hand_in_battery):
    r, t = hand_in_battery[:3, :3], hand_in_battery[:3, 3]
    points = {n: p @ r.T + t for n, p in cloud.items()}
    all_points = np.vstack(list(points.values()))
    contacts = {}
    for family in ['thumb', 'index', 'middle']:
        p = np.vstack([v for n, v in points.items() if family in n])
        sd = cylinder_sdf(p)
        index = int(sd.argmin())
        contacts[family] = {'sampled_signed_gap_m': float(sd[index]), 'nearest_sample_battery_m': p[index].tolist()}
    return {'sampled_foam_penetration_m': float(max(0, -RADIUS - all_points[:, 2].min())),
            'sampled_foam_clearance_m': float(all_points[:, 2].min() + RADIUS),
            'sampled_battery_penetration_m': float(max(0, -cylinder_sdf(all_points).min())),
            'contacts': contacts}


def propose(seed_count=140, output=None):
    g = HandGeometry()
    config = json.loads((ROOT / 'assets/room01/battery_task/task_config.json').read_text())
    battery = config['task_objects']['BatteryRight']
    world_battery = np.eye(4)
    world_battery[:3, :3] = Rotation.from_quat(battery['orientation_xyzw']).as_matrix()
    world_battery[:3, 3] = battery['position']
    rng = np.random.default_rng(20260916)
    bounds = np.array([[g.kin.joints['right_' + n + '_joint'][k] for k in ['lower', 'upper']] for n in HAND_SUFFIXES])
    pool = []
    for iteration in range(seed_count):
        motor = np.array([rng.uniform(.6,1.57), rng.uniform(.1,1.03), rng.uniform(.5,1.41),1.41,1.41,1.41])
        cloud = cloud_at(g,motor)
        normals = cloud_at(g,motor,normals=True)
        a = np.vstack([cloud[n] for n in ['right_thumb_distal_link','right_thumb_touch_link']])
        an = np.vstack([normals[n] for n in ['right_thumb_distal_link','right_thumb_touch_link']])
        b = np.vstack([cloud[n] for n in ['right_index_distal_link','right_index_touch_link']])
        bn = np.vstack([normals[n] for n in ['right_index_distal_link','right_index_touch_link']])
        d = b[None,:,:]-a[:,None,:]
        gap = np.linalg.norm(d,axis=-1)
        n = d/np.maximum(gap[...,None],1e-12)
        facing_a = np.sum(an[:,None,:]*n,axis=-1)
        facing_b = -np.sum(bn[None,:,:]*n,axis=-1)
        opposition = an@bn.T
        costs = abs(gap-.0125)
        costs[(facing_a<.35)|(facing_b<.35)|(opposition>-.25)|(gap<.008)|(gap>.019)] = np.inf
        shortlist = np.argpartition(costs.ravel(), min(5,costs.size-1))[:5]
        all_points = np.vstack(list(cloud.values()))
        for flat in shortlist:
            if not np.isfinite(costs.ravel()[flat]): continue
            i,j = np.unravel_index(flat,costs.shape)
            center = (a[i]+b[j])/2
            transverse = n[i,j]
            up0 = np.array([0.,0.,-1.])
            up0 -= transverse*np.dot(up0,transverse)
            up0 /= max(np.linalg.norm(up0),1e-12)
            up1 = np.cross(transverse,up0)
            angles=np.linspace(-math.pi,math.pi,32,endpoint=False)
            ups=np.cos(angles[:,None])*up0+np.sin(angles[:,None])*up1
            minz=((all_points-center)@ups.T).min(axis=0)
            for ui in np.argsort(-minz)[:2]:
                up = ups[ui]
                axis = np.cross(transverse,up)
                rotation = np.vstack([axis,transverse,up])
                transform = np.eye(4);transform[:3,:3]=rotation;transform[:3,3]=-rotation@center
                metrics=evaluate(cloud,transform)
                targetgap=metrics['contacts']['thumb']['sampled_signed_gap_m']
                fingergap=metrics['contacts']['index']['sampled_signed_gap_m']
                score=(abs(targetgap)+abs(fingergap)+12*metrics['sampled_foam_penetration_m']+6*metrics['sampled_battery_penetration_m'])
                pool.append({'score':score,'motor':motor.copy(),'transform':transform.copy(), 'metrics':metrics,
                             'pair_gap_m':float(gap[i,j]),'normal_dot':float(opposition[i,j]),
                             'facing_dot':[float(facing_a[i,j]),float(facing_b[i,j])]})
        if iteration%20==0:
            print('CPU_SEEDS',iteration,'candidate_count',len(pool),'best',min([p['score'] for p in pool],default=None),flush=True)
    pool.sort(key=lambda p:p['score'])
    candidates=[]
    for p in pool:
        if len(candidates)>=6:break
        if any(np.linalg.norm(p['motor']-np.array(v['closed_motor_rad']))<.10 for v in candidates): continue
        closed=p['motor']; opened=closed.copy();opened[1]=max(0,opened[1]-.22);opened[2]=max(0,opened[2]-.35)
        trajectory_metrics=[]
        for alpha in np.linspace(0,1,9):
            v=opened*(1-alpha)+closed*alpha
            trajectory_metrics.append(evaluate(cloud_at(g,v),p['transform']))
        candidate_id=len(candidates)
        inverse=np.linalg.inv(p['transform'])
        world_hand=world_battery@p['transform']
        candidates.append({'candidate_id':candidate_id,'status':'unvalidated_geometric_proposal',
            'score_m':float(p['score']),'side':'right','object':'BatteryRight',
            'motor_names':['right_'+n+'_joint' for n in HAND_SUFFIXES],
            'open_motor_rad':opened.tolist(),'closed_motor_rad':closed.tolist(),
            'battery_from_hand_base':p['transform'].tolist(),
            'hand_base_from_battery':inverse.tolist(),
            'world_from_hand_base':world_hand.tolist(),
            'world_hand_position_m':world_hand[:3,3].tolist(),
            'world_hand_quaternion_xyzw':Rotation.from_matrix(world_hand[:3,:3]).as_quat().tolist(),
            'pair_gap_m':p['pair_gap_m'],'opposing_surface_normal_dot':p['normal_dot'],
            'surface_facing_dots':p['facing_dot'],'closed_geometry':p['metrics'],
            'open_geometry':trajectory_metrics[0],
            'closure_max_sampled_foam_penetration_m':max(m['sampled_foam_penetration_m'] for m in trajectory_metrics),
            'closure_max_sampled_battery_penetration_m':max(m['sampled_battery_penetration_m'] for m in trajectory_metrics),
            'motor_limits_satisfied':bool(np.all(closed>=bounds[:,0]) and np.all(closed<=bounds[:,1])),
            'approach_world_direction':[0,0,-1], 'pregrasp_lift_m':.10})
    report={'scope':'CPU sampled collision-hull geometry only; no contact, lift, or insertion success claimed',
            'battery_diameter_m':2*RADIUS,'battery_length_m':2*HALF_LENGTH,'foam_top_z_m':.792,
            'seed_count':seed_count,'raw_candidate_count':len(pool),'candidates':candidates,
            'caveats':['Convex collision-hull sample distances can miss triangles between samples.',
                       'Infinite horizontal foam-top plane is conservative outside the actual foam footprint.',
                       'Hand self-collision, arm IK/collision, contact stability and actual collision cooking require separate verification.',
                       'Closed commands near material overlap are proposals for compliant contact response, never evidence of a grasp.',
                       'No object attachment, size change or kinematic battery motion is used.']}
    if output is None:output=ROOT/'reports/room01_battery_feasibility/grasp_candidates.json'
    output=Path(output);output.parent.mkdir(parents=True,exist_ok=True);output.write_text(json.dumps(report,indent=2)+'\n')
    print('OUTPUT',output,flush=True)
    print(json.dumps(candidates[:2],indent=2),flush=True)
    return report


if __name__=='__main__':
    import argparse
    parser=argparse.ArgumentParser();parser.add_argument('--seeds',type=int,default=140);args=parser.parse_args()
    propose(args.seeds)


def refine_static_commands(source=None, output=None):
    """Refine near-contact motor commands at fixed candidate transforms.

    Existing seed IDs are retained in the separate output. The starting
    near-contact seed is the preshaped open pose; opening all the way would
    sweep fingertips through the support plane in several orientations.
    """
    from scipy.optimize import least_squares
    source=Path(source or ROOT/'reports/room01_battery_feasibility/grasp_candidates.json')
    output=Path(output or ROOT/'reports/room01_battery_feasibility/grasp_candidates_refined.json')
    report=json.loads(source.read_text());g=HandGeometry()
    refined=[]
    for c in report['candidates'][:4]:
        t=np.array(c['battery_from_hand_base']);q=np.array(c['closed_motor_rad'])
        lower=np.maximum(q[:3]-.075,0);upper=np.minimum(q[:3]+.075,[1.57,1.03,1.41])
        def result(x):
            qs=q.copy();qs[:3]=x
            return evaluate(cloud_at(g,qs),t)
        def residual(x):
            m=result(x)
            return np.array([m['contacts']['thumb']['sampled_signed_gap_m']+.00012,
                             m['contacts']['index']['sampled_signed_gap_m']+.00012,
                             12*max(m['sampled_foam_penetration_m'],0),
                             10*max(m['sampled_battery_penetration_m']-.00025,0),
                             .00005*np.linalg.norm(x-q[:3])])
        fit=least_squares(residual,q[:3],bounds=(lower,upper),max_nfev=90,ftol=1e-10,xtol=1e-10,gtol=1e-10,diff_step=1e-4)
        close=q.copy();close[:3]=fit.x
        # Search narrow preshapes; keep both pads at least 0.7mm clear.
        options=[]
        for da in [0,.006,.012,.024]:
            for db in [0,-.006,-.012,-.020,-.030]:
                for dc in [0,-.006,-.012,-.020,-.030]:
                    op=close.copy();op[:3]+=np.array([da,db,dc]);op[:3]=np.clip(op[:3],0,[1.57,1.03,1.41])
                    m=evaluate(cloud_at(g,op),t)
                    gaps=[m['contacts'][f]['sampled_signed_gap_m'] for f in ['thumb','index']]
                    cost=sum(max(.0007-v,0) for v in gaps)+15*m['sampled_foam_penetration_m']+10*m['sampled_battery_penetration_m']+.0001*np.linalg.norm(op-close)
                    options.append((cost,op,m))
        _,opened,openedmetrics=min(options,key=lambda p:p[0])
        trajectory=[evaluate(cloud_at(g,(1-a)*opened+a*close),t) for a in np.linspace(0,1,17)]
        cm=trajectory[-1]
        c.update(object_name='BatteryRight',hand_from_battery=c['hand_base_from_battery'],
                 open_fingers_rad=opened.tolist(),closed_fingers_rad=close.tolist(),
                 open_motor_rad=opened.tolist(),closed_motor_rad=close.tolist(),
                 open_geometry=openedmetrics,closed_geometry=cm,
                 closure_max_sampled_foam_penetration_m=max(m['sampled_foam_penetration_m'] for m in trajectory),
                 closure_max_sampled_battery_penetration_m=max(m['sampled_battery_penetration_m'] for m in trajectory))
        c['static_geometry_screen_pass']=bool(c['closure_max_sampled_foam_penetration_m']<.00001 and c['closure_max_sampled_battery_penetration_m']<.0003 and all(abs(cm['contacts'][f]['sampled_signed_gap_m'])<.0004 for f in ['thumb','index']))
        c['geometry_limit']='Sampled collision hulls only; up to .25mm commanded overlap permitted for contact preload, not a measured physical penetration.'
        refined.append(c)
        print('REFINED',c['candidate_id'],'pass',c['static_geometry_screen_pass'],'q',close[:3].tolist(),'foam',c['closure_max_sampled_foam_penetration_m'],'gaps',[cm['contacts'][f]['sampled_signed_gap_m'] for f in ['thumb','index']],flush=True)
    report['candidates']=refined;report['refinement']='Preshaped opening and contact command refinement; source candidate IDs and hand poses unchanged.'
    output.write_text(json.dumps(report,indent=2)+'\n');print('REFINED_OUTPUT',output,flush=True)
    return report


def dense_hand_geometry():
    """Use every collision-convex-hull vertex plus triangle centres/edge centres."""
    from room01_sim.umi_layout import stl_surface
    from room01_sim.kinematics import URDF, origin_transform
    g=HandGeometry()
    for name,link in g.kin.links.items():
        if not name.startswith('right_') or name not in g.meshes:continue
        points=[]
        for collision in link.findall('collision'):
            mesh=collision.find('geometry/mesh')
            if mesh is None:continue
            samples=stl_surface(URDF.parent/mesh.get('filename'),maximum=100000)
            scale=np.array([float(v) for v in mesh.get('scale','1 1 1').split()])
            t=origin_transform(collision.find('origin'))
            points.append((samples*scale)@t[:3,:3].T+t[:3,3])
        g.meshes[name]=np.vstack(points)
    return g


def verify_dense(report_path=None):
    path=Path(report_path or ROOT/'reports/room01_battery_feasibility/grasp_candidates_refined.json')
    report=json.loads(path.read_text());g=dense_hand_geometry()
    print('DENSE_POINTS',sum(len(v) for k,v in g.meshes.items() if k.startswith('right_')),flush=True)
    for c in report['candidates']:
        t=np.array(c['battery_from_hand_base']);op=np.array(c['open_fingers_rad']);cl=np.array(c['closed_fingers_rad'])
        ms=[evaluate(cloud_at(g,(1-a)*op+a*cl),t) for a in np.linspace(0,1,9)]
        c['dense_confirmation']={'closed':ms[-1],'open':ms[0],
            'closure_max_foam_penetration_m':max(m['sampled_foam_penetration_m'] for m in ms),
            'closure_max_battery_penetration_m':max(m['sampled_battery_penetration_m'] for m in ms),
            'sampling':'All hull vertices, triangle centroids and edge midpoints; still not exact cylinder-mesh distances'}
        c['dense_screen_pass']=bool(c['dense_confirmation']['closure_max_foam_penetration_m']<1e-5 and c['dense_confirmation']['closure_max_battery_penetration_m']<.0004)
        print('DENSE',c['candidate_id'],'pass',c['dense_screen_pass'],'foam',c['dense_confirmation']['closure_max_foam_penetration_m'],'battery',c['dense_confirmation']['closure_max_battery_penetration_m'],flush=True)
    path.write_text(json.dumps(report,indent=2)+'\n')
    return report


def actual_support_metrics(cloud, hand_in_battery, object_name='BatteryRight'):
    """Finite foam-box and fixed red-box checks plus the full tabletop plane.

    Using the enclosing box for bevelled foam is conservative near its edges.
    The red block outer box is conservative at its holes (far from this grasp).
    """
    cfg=json.loads((ROOT/'assets/room01/battery_task/task_config.json').read_text())
    spec=json.loads((ROOT/'assets/room01/battery_task/scene_spec.json').read_text())
    battery=cfg['task_objects'][object_name];rb=Rotation.from_quat(battery['orientation_xyzw']).as_matrix()
    rf=np.array(spec['frame']['rotation_world_from_local']);origin=np.array(spec['frame']['world_origin_m'])
    p=np.vstack(list(cloud.values()));p=p@hand_in_battery[:3,:3].T+hand_in_battery[:3,3]
    w=p@rb.T+np.array(battery['position']);local=(w-origin)@rf
    def boxsd(points,center,dims):
        d=abs(points-np.array(center))-np.array(dims)/2
        return np.linalg.norm(np.maximum(d,0),axis=1)+np.minimum(d.max(axis=1),0)
    obstacles={}
    for i,xy in enumerate(spec['props']['yellow_positions_xy_m']):
        dims=spec['props']['yellow_dims_m'];center=[*xy,spec['table']['height_m']+dims[2]/2]
        d=boxsd(local,center,dims);idx=int(d.argmin())
        obstacles['YellowFoam_'+str(i+1)]={'minimum_sampled_signed_gap_m':float(d[idx]),'nearest_point_table_local_m':local[idx].tolist(),'max_sampled_penetration_m':float(max(0,-d[idx]))}
    dims=spec['props']['red_dims_m'];center=[*spec['props']['red_position_xy_m'],spec['table']['height_m']+dims[2]/2]
    d=boxsd(local,center,dims);obstacles['RedFoam_enclosing_box']={'minimum_sampled_signed_gap_m':float(d.min()),'max_sampled_penetration_m':float(max(0,-d.min()))}
    obstacles['Tabletop_plane']={'minimum_sampled_signed_gap_m':float(w[:,2].min()-spec['table']['height_m']), 'max_sampled_penetration_m':float(max(0,spec['table']['height_m']-w[:,2].min()))}
    return {'obstacles':obstacles,'max_sampled_obstacle_penetration_m':max(v['max_sampled_penetration_m'] for v in obstacles.values())}
