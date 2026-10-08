#!/usr/bin/env python3
"""Read-only SSH download of bounded episode packages for batch replay."""
import argparse
import hashlib
import io
import json
from pathlib import Path
import shlex
import subprocess
import tarfile
import tempfile
import shutil

REMOTE = r'''
import sys,json,hashlib,tarfile,io
from pathlib import Path
dataset=Path(sys.argv[1]); index=int(sys.argv[2])
meta=json.loads((dataset/'meta/umi_conversion.json').read_text())
record=next(x for x in meta['episodes'] if x['episode_index']==index)
eid=record['episode_id']
if Path(eid).name!=eid or '..' in eid: raise ValueError('invalid source episode name')
source=Path('/mnt/data/dzq/umi_v2/data/task_v2_x2')/eid
files={f'episode_{index:06d}.parquet':dataset/f'data/chunk-{index//1000:03d}/episode_{index:06d}.parquet'}
for name in ['info.json','umi_conversion.json','tasks.jsonl']:
 files['meta/'+name]=dataset/'meta'/name
for name in ['hand_pose_test_true_absolute.csv','hand_pose.csv','e6_rgb_controller_poses.csv']:
 files['source/camera/'+name]=source/'camera'/name
files['source/sync/e6_rgb_timing.csv']=source/'sync/e6_rgb_timing.csv'
files['alignment/alignment_output_grid.csv']=dataset/f'meta/alignment/episode_{index:06d}/alignment_output_grid.csv'
files['videos/head_rgb.mp4']=dataset/f'videos/chunk-{index//1000:03d}/observation.images.head_rgb/episode_{index:06d}.mp4'
manifest={'schema_version':'umi_replay_source_manifest_v1','episode_index':index,'dataset_root':str(dataset),'source_episode_root':str(source),'files':[]}
with tarfile.open(fileobj=sys.stdout.buffer,mode='w|') as tar:
 for name,path in files.items():
  content=path.read_bytes(); item=tarfile.TarInfo(name);item.size=len(content);tar.addfile(item,io.BytesIO(content))
  manifest['files'].append({'local_path':name,'remote_path':str(path),'size_bytes':len(content),'sha256':hashlib.sha256(content).hexdigest(),'remote_sha256_verified':True})
 content=json.dumps(manifest,indent=2).encode();item=tarfile.TarInfo('source_manifest.json');item.size=len(content);tar.addfile(item,io.BytesIO(content))
'''


def main(args):
    if args.episodes == "all":
        command = shlex.join(["python3", "-c", "import json,sys;from pathlib import Path;print(json.loads((Path(sys.argv[1])/'meta/info.json').read_text())['total_episodes'])", args.dataset])
        total = int(subprocess.check_output(["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=10", args.host, command], text=True))
        episodes = range(total)
    else:
        episodes = list(dict.fromkeys(int(i) for i in args.episodes.split(",")))
    args.output.mkdir(parents=True, exist_ok=True)
    for index in episodes:
        target = args.output / f"episode_{index:06d}"
        if target.exists():
            manifest = json.loads((target / "source_manifest.json").read_text())
            for entry in manifest["files"]:
                if hashlib.sha256((target / entry["local_path"]).read_bytes()).hexdigest() != entry["sha256"]:
                    raise ValueError(f"Existing source package failed checksum: {target}")
            print("CACHED", target, flush=True); continue
        command = shlex.join(["python3", "-c", REMOTE, args.dataset, str(index)])
        result = subprocess.run(["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=10", args.host, command],
                                check=True, stdout=subprocess.PIPE, timeout=180)
        with tempfile.TemporaryDirectory(prefix="umi-fetch-", dir=args.output) as temporary:
            tmp = Path(temporary)
            with tarfile.open(fileobj=io.BytesIO(result.stdout)) as tar:
                for member in tar:
                    name = Path(member.name)
                    if name.is_absolute() or ".." in name.parts or not member.isfile():
                        raise ValueError("Unexpected archive member")
                    path = tmp/name; path.parent.mkdir(parents=True, exist_ok=True)
                    path.write_bytes(tar.extractfile(member).read())
            manifest = json.loads((tmp / "source_manifest.json").read_text())
            manifest["ssh_alias"] = args.host
            for entry in manifest["files"]:
                if hashlib.sha256((tmp / entry["local_path"]).read_bytes()).hexdigest() != entry["sha256"]:
                    raise ValueError("Downloaded source checksum mismatch")
            (tmp / "source_manifest.json").write_text(json.dumps(manifest, indent=2))
            shutil.move(str(tmp), target)
        print("FETCHED", target, flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="training-kai")
    parser.add_argument("--dataset", default="/mnt/data/dzq/umi_v2/datasets/task_v2_x2_high_lerobot_10hz_h50_right_eye_v2")
    parser.add_argument("--episodes", default="0,20,84")
    parser.add_argument("--output", type=Path, default=Path(__file__).resolve().parents[1]/"data/umi_replay")
    main(parser.parse_args())
