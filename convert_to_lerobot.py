import csv
import argparse
import torch
from pathlib import Path
from PIL import Image

try:
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
except ImportError as exc:
    print("Failed to import LeRobotDataset:")
    raise

def convert_episode(episode_dir, repo_id, task_name, push_to_hub):
    episode_path = Path(episode_dir)
    csv_path = episode_path / "trajectory.csv"
    
    if not csv_path.exists():
        print(f"Error: {csv_path} does not exist.")
        return

    print(f"Converting episode from {episode_dir}...")
    
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
    dataset = LeRobotDataset.create(
        repo_id=repo_id,
        fps=30,
        features=features,
    )

    with open(csv_path, 'r') as f:
        reader = csv.DictReader(f)
        for i, row in enumerate(reader):
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
