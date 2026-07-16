import csv
import sys
import argparse
import torch
from pathlib import Path
from PIL import Image

try:
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    from lerobot.utils.constants import HF_LEROBOT_HOME
except ImportError as exc:
    print("Failed to import LeRobotDataset:")
    raise

# Nominal fps the webcam targets (see webcam_teleop_interface.py). Used as a
# fallback and as the baseline for the deviation warning below.
NOMINAL_FPS = 30
FPS_DEVIATION_WARNING_THRESHOLD = 0.15


def compute_recording_fps(rows, nominal_fps=NOMINAL_FPS):
    """Estimate the actual frame rate from trajectory.csv's timestamp column.

    trajectory.csv's timestamps reflect real wall-clock time between
    successfully recorded frames, including any gaps from frames the
    recorder skipped upstream (e.g. a failed IK solve or markers out of
    view) -- those frames are silently dropped and never advance
    frame_index. A deviation from the nominal fps here more often means
    "some frames were dropped" than "the camera ran this slowly
    throughout". LeRobot 0.6.0 only supports a single fps per dataset
    (each frame's timestamp is assigned as frame_index / fps), so this is
    still an average approximation, not a frame-accurate fix.
    """
    if len(rows) < 2:
        print(f"Warning: only {len(rows)} frame(s) recorded, cannot measure fps. Using nominal fps={nominal_fps}.")
        return nominal_fps

    first_ts = float(rows[0]['timestamp'])
    last_ts = float(rows[-1]['timestamp'])
    duration = last_ts - first_ts
    if duration <= 0:
        print(f"Warning: recorded timestamps span {duration:.4f}s, cannot measure fps. Using nominal fps={nominal_fps}.")
        return nominal_fps

    measured_fps = (len(rows) - 1) / duration
    fps = max(1, round(measured_fps))

    print(f"Measured recording rate: {measured_fps:.2f} fps over {duration:.2f}s ({len(rows)} frames). "
          f"Using fps={fps} for this dataset (nominal webcam target is {nominal_fps} fps).")

    deviation = abs(measured_fps - nominal_fps) / nominal_fps
    if deviation > FPS_DEVIATION_WARNING_THRESHOLD:
        print(f"WARNING: measured fps deviates {deviation * 100:.0f}% from the nominal {nominal_fps} fps target.")
        print("         This usually means frames were dropped during recording (e.g. markers out of view,")
        print("         failed IK solves) rather than the camera/loop running this slowly throughout.")
        print("         A single average fps is still an approximation of the real per-frame timing.")

    return fps


def convert_episode(episode_dir, repo_id, task_name, push_to_hub):
    episode_path = Path(episode_dir)
    csv_path = episode_path / "trajectory.csv"

    if not csv_path.exists():
        print(f"Error: {csv_path} does not exist.")
        return

    print(f"Converting episode from {episode_dir}...")

    with open(csv_path, 'r') as f:
        rows = list(csv.DictReader(f))

    if not rows:
        print(f"Error: {csv_path} contains no frames.")
        return

    fps = compute_recording_fps(rows)

    # Define features based on our data recorder
    # Ensure image size matches what your webcam produces!
    features = {
        "observation.image": {
            "dtype": "video",
            "shape": (3, 1080, 1920), # (C, H, W)
            "names": ["c", "h", "w"],
        },
        "observation.state": {
            "dtype": "float32",
            "shape": (9,),
            "names": ["base_x", "base_y", "base_theta", "lift", "arm", "wrist_roll", "wrist_pitch", "wrist_yaw", "gripper_width_m"],
        },
        "action": {
            "dtype": "float32",
            "shape": (9,),
            "names": ["base_x", "base_y", "base_theta", "lift", "arm", "wrist_roll", "wrist_pitch", "wrist_yaw", "gripper_width_m"],
        }
    }

    # Initialize the LeRobot dataset
    # By default, it will save to ~/.cache/huggingface/lerobot
    try:
        dataset = LeRobotDataset.create(
            repo_id=repo_id,
            fps=fps,
            features=features,
        )
    except FileExistsError:
        existing_path = HF_LEROBOT_HOME / repo_id
        print(f"Error: a dataset already exists at {existing_path}")
        print(f"  --repo-id '{repo_id}' has already been used to create a LeRobot dataset.")
        print("  Choose a different --repo-id, or remove the existing dataset directory if you want to overwrite it.")
        print("  (Appending additional episodes to an existing dataset is not yet supported by this script.)")
        sys.exit(1)

    for i, row in enumerate(rows):
        # Parse observation state
        state = torch.tensor([
            float(row['base_x']),
            float(row['base_y']),
            float(row['base_theta']),
            float(row['lift']),
            float(row['arm']),
            float(row['wrist_roll']),
            float(row['wrist_pitch']),
            float(row['wrist_yaw']),
            float(row['gripper_width_m'])
        ], dtype=torch.float32)

        # Parse action
        action = torch.tensor([
            float(row['base_x_joint']),
            float(row['base_y_joint']),
            float(row['base_theta_joint']),
            float(row['joint_lift']),
            float(row['joint_arm_l0']),
            float(row['joint_wrist_roll']),
            float(row['joint_wrist_pitch']),
            float(row['joint_wrist_yaw']),
            float(row['commanded_gripper_width_m'])
        ], dtype=torch.float32)

        # Parse image
        img_path = episode_path / row['image_teleop_webcam']
        # LeRobot uses PIL Images for add_frame
        with Image.open(img_path) as image:
            img = image.convert("RGB").copy()

        frame_dict = {
            "observation.image": img,
            "observation.state": state,
            "action": action,
            "task": task_name
        }

        dataset.add_frame(frame_dict)

        if i % 10 == 0:
            print(f"Processed frame {i}...")

    # Save the episode
    dataset.save_episode()
    print(f"Episode saved locally to {dataset.root}")

    if push_to_hub:
        print("Pushing to Hugging Face Hub...")
        dataset.push_to_hub()
        print("Push complete!")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Convert Stretch Dex Teleop CSV data to LeRobotDataset format")
    parser.add_argument("--episode-dir", type=str, required=True, help="Path to the episode directory containing trajectory.csv")
    parser.add_argument("--repo-id", type=str, default="your-username/stretch_dex_teleop", help="Hugging Face repo ID (e.g. username/dataset_name)")
    parser.add_argument("--task", type=str, default="teleoperation_task", help="Text description of the task being performed")
    parser.add_argument("--push", action="store_true", help="Push to Hugging Face Hub after converting")
    args = parser.parse_args()

    convert_episode(args.episode_dir, args.repo_id, args.task, args.push)
