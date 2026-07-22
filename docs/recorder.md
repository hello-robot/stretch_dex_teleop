# Recorder: Architecture & CSV Schema

Detail doc for `recorder.py` / `dex_teleop.py`.

## Architecture

The recorder is designed specifically to prevent any latency spikes in the tight real-time control loop:

- **Background threading**: saving high-resolution JPEG images to disk is a blocking operation. `EpisodeRecorder` pushes each image into a `queue.Queue`, consumed by a dedicated background thread that writes it to disk (`cv2.imwrite`) asynchronously.
- **Dependency-free CSV writing**: to avoid `numpy`/`pandas` version conflicts, the recorder uses the built-in Python `csv` module, not a dataframe library.
- **Frames are dropped, not padded, when a goal can't be computed**: a frame is only appended to `trajectory.csv` when the teleop loop successfully computes a commanded configuration for that iteration (ArUco markers visible and IK succeeded). Failed iterations (markers briefly out of view, IK failing, an extreme wrist-orientation change rejected) are silently skipped — `frame_index` doesn't advance and no image is written. This keeps the dataset free of garbage/duplicate frames, but means wall-clock time between two consecutive *recorded* frames can be much larger than the camera's raw capture period.
- **Failed image writes are also dropped, not left dangling**: if a background image write fails (disk full, permissions, etc.), that frame's row is excluded from `trajectory.csv` at `stop_episode()` rather than referencing a file that doesn't exist. A warning lists exactly which frame(s) were dropped and why — `trajectory.csv` never lies about what's actually on disk.


## CSV Schema (`trajectory.csv`)

### Meta Data
- `timestamp`: Time elapsed since the start of the recording (in seconds).
- `frame_index`: Sequential ID of the recorded frame.
- `image_teleop_webcam`: Relative path to the saved external webcam RGB image (e.g. `images/teleop_webcam_000000.jpg`).
- `image_wrist_cam`: Relative path to the saved wrist camera (D405) RGB image (e.g. `wrist_images/wrist_cam_000000.jpg`), or an empty string if no wrist camera was detected. Every row in an episode has this filled in, or none do — never a mix.
- `image_head_cam`: Relative path to the saved head camera (D435i) RGB image (e.g. `head_images/head_cam_000000.jpg`), already rotated 90°. Same empty-string/all-or-nothing behavior as `image_wrist_cam`.

### Measured State (Current Robot Position)
- `base_x`, `base_y`, `base_theta`: The current estimated odometry of the mobile base.
- `lift`, `arm`: The prismatic extension of the lift and arm in meters.
- `wrist_roll`, `wrist_pitch`, `wrist_yaw`: Dexterous wrist joint angles in radians.
- `gripper_finger_right`: Raw Dynamixel position value for the `stretch_gripper` joint.

### Commanded Actions (Teleop Input Goals)
- `base_x_joint`, `base_y_joint`: **Not real commanded values**. This teleop scheme never commands base translation, only rotation, so these two columns are currently just a copy of the measured `base_x`/`base_y` odometry.
- `base_theta_joint`: The absolute base angle target, genuinely derived from the incremental `joint_mobile_base_rotate_by` teleop command — this one *is* a real commanded value.
- `joint_lift`, `joint_arm_l0`: The target extension for the lift and arm in meters.
- `joint_wrist_roll`, `joint_wrist_pitch`, `joint_wrist_yaw`: The target angles for the dexterous wrist in radians.
- `stretch_gripper`: Target raw Dynamixel position value for the gripper.

### Physical Gripper Distances (Metric Conversion)
Downstream models typically expect the physical gripper width in meters, not raw Dynamixel values. The recorder queries the robot's hardware config (`robot.end_of_arm.motors['stretch_gripper'].params['gripper_conversion']`) at runtime to convert.
- `gripper_width_m`: Measured physical distance between the gripper fingers in meters.
- `commanded_gripper_width_m`: Commanded physical distance between the gripper fingers in meters.


## Success/Failure Labeling

After stopping a non-empty recording, you're asked `Was the episode successful? (y/n)` (the window freezes on the last frame; any key but `y`/`n` is ignored). The answer is written as `success.txt` (`"Success"` or `"Failure"`) into the episode directory. Pass `--skip-success` to disable the prompt entirely (no `success.txt` gets written). Quitting with `q` while recording stops and labels it the same way as pressing `r`.

See [converter.md](converter.md) for how this label is used at conversion time.
