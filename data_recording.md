# Stretch Dex Teleop Data Recorder

This branch (`feature/data-recorder`) introduces a lightweight, non-blocking data recorder to the `stretch_dex_teleop` repository. It is designed to capture synchronized robot telemetry, commanded teleoperation actions, and webcam imagery for downstream imitation learning models (such as `stretch_ai` or `LeRobot`).

The recorder itself has no dependency on LeRobot or any ML framework — it only runs on the robot and produces plain CSV + JPEG files. `convert_to_lerobot.py` is the only component that depends on LeRobot, and it's meant to run on a separate development machine, not on the robot.

## Usage

To use the data recorder:
1. Run the teleoperation script as usual: `python dex_teleop.py`
2. A window titled **Dex Teleop** will appear showing your webcam feed and a status overlay.
3. Ensure the **Dex Teleop** window is in focus on your desktop.
4. Press `r` on your keyboard to **Start** recording an episode.
5. Perform your manipulation task.
6. Press `r` again to **Stop** the recording and finalize the episode.
7. Press `q` to safely quit the teleoperation script.

All recorded data will be saved inside the `data/episode_YYYY-MM-DD--HH-MM-SS/` directory.

## Architecture

The recorder is designed specifically to prevent any latency spikes in the tight real-time control loop:
- **Background Threading**: Saving high-resolution JPEG images to a storage drive is a blocking operation. The `EpisodeRecorder` pushes the OpenCV image arrays into a `queue.Queue`, which is consumed by a dedicated background thread that writes them to disk (`cv2.imwrite`) asynchronously.
- **Dependency-Free CSV Writing**: To avoid widespread `numpy` version conflicts and fatal crashes associated with `pandas`, the recorder uses the built-in, lightweight Python `csv` module to generate the trajectory file.
- **Frames are dropped, not padded, when a goal can't be computed**: a frame is only appended to `trajectory.csv` when the teleop loop successfully computes a commanded configuration for that iteration (i.e. the ArUco markers were visible and inverse kinematics succeeded). Iterations where this fails (markers briefly out of view, an IK solve failing, an extreme wrist-orientation change being rejected) are silently skipped — `frame_index` does not advance and no image is written for them. This keeps the dataset free of garbage/duplicate frames, but it also means the wall-clock time between two consecutive *recorded* frames can be much larger than the webcam's capture period. See [Known Limitations](#known-limitations) below.

## CSV Schema (`trajectory.csv`)

The generated dataset contains a wide variety of states synchronized to each frame of the webcam image.

### Meta Data
- `timestamp`: Time elapsed since the start of the recording (in seconds).
- `frame_index`: Sequential ID of the recorded frame.
- `image_teleop_webcam`: Relative path to the saved RGB image (e.g. `images/teleop_webcam_000000.jpg`).

### Measured State (Current Robot Position)
- `base_x`, `base_y`, `base_theta`: The current estimated odometry of the mobile base.
- `lift`, `arm`: The prismatic extension of the lift and arm in meters.
- `wrist_roll`, `wrist_pitch`, `wrist_yaw`: Dexterous wrist joint angles in radians.
- `gripper_finger_right`: Raw Dynamixel position value for the `stretch_gripper` joint.

### Commanded Actions (Teleop Input Goals)
- `base_x_joint`, `base_y_joint`: **Not real commanded values — see [Known Limitations](#known-limitations).** This teleop scheme never commands the mobile base to translate, only to rotate, so these two columns are currently just a copy of the measured `base_x`/`base_y` odometry.
- `base_theta_joint`: The absolute base angle target, genuinely derived from the incremental `joint_mobile_base_rotate_by` teleop command (this one *is* a real commanded value).
- `joint_lift`, `joint_arm_l0`: The target extension for the lift and arm in meters.
- `joint_wrist_roll`, `joint_wrist_pitch`, `joint_wrist_yaw`: The target angles for the dexterous wrist in radians.
- `stretch_gripper`: Target raw Dynamixel position value for the gripper.

### Physical Gripper Distances (Metric Conversion)
Downstream models typically expect the physical width of the gripper in meters rather than the internal arbitrary Dynamixel values. The recorder dynamically queries the robot's hardware configuration (`robot.end_of_arm.motors['stretch_gripper'].params['gripper_conversion']`) at runtime to natively convert these values.
- `gripper_width_m`: The measured physical distance between the gripper fingers in meters.
- `commanded_gripper_width_m`: The commanded physical distance between the gripper fingers in meters.

> **Note on Gripper Saturation:** You may notice that `commanded_gripper_width_m` goes far outside the bounds of the physical gripper (e.g., commanding `25cm` when the gripper maxes out at `~9cm`). This is intentional! The teleoperation logic heavily saturates the Dynamixel commands to guarantee that the fingers fully open, or stall firmly against objects to hold them tightly.

## Converting to LeRobot Dataset Format

To train imitation learning models using Hugging Face's [LeRobot](https://github.com/huggingface/lerobot) framework, the raw trajectory CSV and images must be converted into a `LeRobotDataset` (which encodes images to compressed MP4 and saves states as Parquet tables).

We provide the `convert_to_lerobot.py` script to automate this conversion. **Run this on your development machine, not on the robot** — it's the only piece of this pipeline that imports LeRobot/PyTorch.

### Prerequisites
This script has been developed and validated against `lerobot==0.6.0`. LeRobot's dataset API has changed across versions before (its import path itself changed between versions during this project), so installing an unpinned `lerobot` may not work without changes to this script:
```bash
pip install lerobot==0.6.0
```
*(If you want to push directly to the Hugging Face Hub, ensure you are logged in using `huggingface-cli login` first).*

### Running the Conversion
Run the script and point it to your raw episode directory:
```bash
python convert_to_lerobot.py --episode-dir data/episode_2026-07-13--14-22-05 --repo-id "your-hf-username/stretch_dex_teleop"
```

By default this creates a brand-new local dataset at `~/.cache/huggingface/lerobot/{repo-id}`. Running it again against the same `--repo-id` without `--append` will fail with a clear error rather than overwriting or corrupting anything — see **Building a Multi-Episode Dataset** below if you want to add more episodes to it.

You can set the task description recorded with each frame using `--task` (defaults to `"teleoperation_task"`):
```bash
python convert_to_lerobot.py --episode-dir data/episode_2026-07-13--14-22-05 --repo-id "your-hf-username/stretch_dex_teleop" --task "pick_up_the_mug"
```

### Automatic Frame Rate Measurement
LeRobot requires a single `fps` value per dataset, used to timestamp every frame as `frame_index / fps`. Rather than assuming the webcam's nominal 30 fps target, the converter measures the *actual* recording rate directly from `trajectory.csv`'s `timestamp` column (`(num_frames - 1) / (last_timestamp - first_timestamp)`, rounded to the nearest integer) and uses that instead. It prints the measured rate every run, and prints an explicit warning if it deviates more than 15% from the nominal 30 fps — in practice this deviation is usually large and is almost always explained by dropped frames (see [Architecture](#architecture) above and [Known Limitations](#known-limitations) below), not by the camera or control loop actually running that slowly.

You don't need to do anything for this — it's automatic and requires no flag. There is currently no way to override the measured value from the command line.

### Building a Multi-Episode Dataset
Pass `--append` to add an episode to a dataset that already exists at `--repo-id`, instead of creating a new one:
```bash
python convert_to_lerobot.py --episode-dir data/episode_2026-07-15--13-47-54 --repo-id "your-hf-username/stretch_dex_teleop" --append
```
- Without `--append`, converting into an existing `--repo-id` always fails with a clear error (protects against accidentally mixing episodes into the wrong dataset via a typo'd or reused repo-id).
- With `--append` against a `--repo-id` that does **not** exist yet, it also fails with a clear error (nothing to append to) — drop `--append` to create it first.
- **Important limitation**: a dataset's `fps` is fixed forever by whichever episode created it. Every appended episode's frames are timestamped using that original fps, *not* its own freshly measured rate. If an appended episode's measured rate differs meaningfully from the dataset's fixed fps, the script prints a note telling you so — but it still proceeds, because there is no other option within LeRobot's dataset format. If you're combining episodes recorded under very different conditions (e.g. one with lots of marker dropouts, one with very few), be aware the later episode's timing in the dataset may not reflect what actually happened during that recording.

### Pushing to Hugging Face Hub
To automatically push the compiled dataset to your Hugging Face account immediately after conversion, append the `--push` flag (works the same whether you just created the dataset or appended to it):
```bash
python convert_to_lerobot.py --episode-dir data/episode_2026-07-13--14-22-05 --repo-id "your-hf-username/stretch_dex_teleop" --push
```

## Known Limitations

- **`base_x_joint` / `base_y_joint` are not real commanded values.** This teleop scheme controls the mobile base by rotation only (`joint_mobile_base_rotate_by`) — there is no code path that ever commands a base translation. These two CSV columns, and the corresponding `action` dimensions in the converted LeRobot dataset, currently just mirror the measured `base_x`/`base_y` odometry rather than representing any operator intent. `base_theta_joint` is unaffected and is a genuine derived command. Treat `base_x_joint`/`base_y_joint` as placeholders, not signal, if you're training on this data.
- **Dataset frame timing is an approximation, not frame-accurate.** LeRobot only supports a single `fps` per dataset (see **Automatic Frame Rate Measurement** above). The converter measures and uses the real average rate rather than a hardcoded nominal value, which is a large improvement, but it's still one number applied uniformly across the whole episode. If frame drops were clustered (e.g. one long gap versus many small ones), the per-frame timestamps in the resulting dataset won't reflect that unevenness.
- **Frames are silently dropped during recording** whenever ArUco markers are briefly out of view or an IK solve fails (see [Architecture](#architecture)). This is generally the right behavior (no garbage frames), but it's the main reason measured recording rates tend to come out well below the webcam's nominal 30 fps — don't assume a low measured fps means the camera or robot loop is actually running slowly.
- **`robot_type` is not currently set** in the converted dataset's metadata (`info.json` will show `"robot_type": null`). Downstream tooling that keys off this field won't find it populated.
- **Image write failures during recording aren't validated.** If `cv2.imwrite()` silently fails on the robot (e.g. disk full), the corresponding row in `trajectory.csv` will still reference an image file that doesn't exist — this will only surface later as a file-not-found error when you run the converter, potentially on a different machine.
- **Image resolution is hardcoded** in `convert_to_lerobot.py` (`(3, 1080, 1920)`), matching the webcam's configured resolution in `webcam_teleop_interface.py`. If you change the webcam's resolution, update this script to match, or the converter will fail immediately on the first frame with a clear shape-mismatch error (it won't silently corrupt the dataset — LeRobot validates every frame's shape against the declared feature schema).
