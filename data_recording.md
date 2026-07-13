# Stretch Dex Teleop Data Recorder

This branch (`feature/data-recorder`) introduces a lightweight, non-blocking data recorder to the `stretch_dex_teleop` repository. It is designed to capture synchronized robot telemetry, commanded teleoperation actions, and webcam imagery for downstream imitation learning models (such as `stretch_ai` or `LeRobot`).

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
- `base_x_joint`, `base_y_joint`, `base_theta_joint`: The absolute base targets derived from the incremental `joint_mobile_base_rotate_by` teleop actions.
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

We provide the `convert_to_lerobot.py` script to automate this conversion.

### 1. Prerequisites
You must have `lerobot` installed. If you haven't already:
```bash
pip install lerobot
```
*(If you want to push directly to the Hugging Face Hub, ensure you are logged in using `huggingface-cli login` first).*

### 2. Running the Conversion
Run the script and point it to your raw episode directory:
```bash
python convert_to_lerobot.py --episode-dir data/episode_2026-07-13--14-22-05 --repo-id "your-hf-username/stretch_dex_teleop"
```

To automatically push the compiled dataset to your Hugging Face account immediately after conversion, append the `--push` flag:
```bash
python convert_to_lerobot.py --episode-dir data/episode_2026-07-13--14-22-05 --repo-id "your-hf-username/stretch_dex_teleop" --push
```
