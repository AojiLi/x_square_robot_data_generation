# Episode 0 inspection facts

Source dataset and source episode are recorded in `source_manifest.json`. All fourteen downloaded source files match SHA-256 computed over their server counterparts. No server file was modified.

## Data semantics

- Dataset metadata: 85 episodes, 9995 rows, 10 Hz; this episode has 123 rows.
- State: `[left pose9, right pose9, left finger6, right finger6]`, where pose9 is `inverse(T_previous) @ T_current` encoded as translation in metres followed by rotation columns 0 then 1.
- Action: `[50,30]`; `inverse(T_current) @ T_future` for all fifty steps sharing the same observation anchor; absolute finger angles in degrees. Terminal padded targets are explicit in `action_is_pad`.
- The current dataset task is `Put the two objects into the box.`
- `alignment/alignment_output_grid.csv` maps output anchors to source rows 12..744 inclusive, step 6. The state baseline for the first output is source row 6. Raw CSV header lines are not counted as data rows.
- First output source frame ID is 100550 and hand pose timestamp is 1788435362888367003 ns. Video frame 0 corresponds to this output, not raw source frame 0.

## Absolute frame recovery

The raw `hand_pose_test_true_absolute.csv` and actual training-source `hand_pose.csv` have the same rotations but different origins. For both hands:

`T_training_target = T_test_true_absolute @ Translation(-0.042, 0, 0)`

Across all 749 raw rows / 748 adjacent transitions, this produces maximum translation residual below 1.2e-14 m and rotation residual below 2.8e-6 degrees. See `raw_absolute_consistency_audit.json` for measured controller-to-absolute transforms and initial output targets.

This verifies a numerical frame relation, not the physical location of the target point. Native episode metadata inspected (`config_snapshot`, `session_meta`, `manifest`) provides no controller-to-hand or TCP calibration definition. The converter does not recompute calibration. The 42 mm target's identity as wrist/base/pinch point is unverified. The transformation to the simulator robot frame therefore remains a geometric estimate until separately established.

A readable `controller_to_hand_pose.py` under `/home/kai/umi_data_all_process/umi_data_all_process/` describes a different v3 calibration (+X forward, +Z up, and an optional left/right Rx(+/-90 degrees) robot-frame conversion). Its controller origins differ from the origins fitted to this actual episode, so it must not be silently applied as this dataset's calibration.

## Hand channels and URDF caveat

Verified dataset finger order per hand: `thumb_flex, thumb_aux, index, middle, ring, little`.

Read-only local source evidence `reports/room01_camera_calibration/source_evidence/x2-lower__robot_observation.py.txt` names live Revo2 actuators `thumb, thumb_aux, index, middle, ring, pinky`, with maxima `[59,90,81,81,81,81]` degrees. The UMI converter reads absolute serial angle tenths and divides by 10.

Current URDF limits are approximately 90 degrees for `thumb_metacarpal_joint`, 59 degrees for `thumb_proximal_joint`, and 81 degrees for the other proximal joints. The runtime HAND_SUFFIXES order is metacarpal before proximal. Thus matching names and angle ranges supports mapping dataset thumb_flex to thumb_proximal and thumb_aux to thumb_metacarpal (swap first two when emitting runtime hand order). This remains an inference from names/limits; no direct serial-to-URDF mapping implementation was located in the inspected sources. Do not describe it as hardware-calibrated.

## Video inspection

Head source contains 123 frames at 10 Hz, 12.3 seconds. It starts with red and blue blocks outside the black box and two other blocks already inside. Right hand places the blue block first (released by 6 s); left hand places the red block second (at the box at 10 s). At 12 s and the 12.2 s final frame, all four blocks are in the box and both hands have withdrawn. Sparse inspection found no take-out/reset phase. A continuous simulation stability criterion must be checked on simulation object trajectories.
