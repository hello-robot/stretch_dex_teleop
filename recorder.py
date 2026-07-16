import os
import time
import json
import threading
import queue
import cv2
import csv
from pathlib import Path

class EpisodeRecorder:
    def __init__(self, data_dir="data"):
        self.data_dir = Path(data_dir)
        self.data_dir.mkdir(parents=True, exist_ok=True)
        
        self.is_recording = False
        self.episode_dir = None
        self.image_dir = None
        
        self.records = []
        self.frame_index = 0
        self.start_time = 0.0
        
        self.image_queue = queue.Queue()
        self.writer_thread = None
        self._stop_writer = False
        self._failed_image_writes = set()

    def start_episode(self):
        if self.is_recording:
            print("Already recording.")
            return

        now = time.strftime("%Y-%m-%d--%H-%M-%S")
        self.episode_dir = self.data_dir / f"episode_{now}"
        self.episode_dir.mkdir(parents=True, exist_ok=True)

        self.image_dir = self.episode_dir / "images"
        self.image_dir.mkdir(parents=True, exist_ok=True)

        self.records = []
        self.frame_index = 0
        self.start_time = time.time()
        self.is_recording = True

        self._stop_writer = False
        self._failed_image_writes = set()
        self.writer_thread = threading.Thread(target=self._image_writer_worker)
        self.writer_thread.start()
        print(f"Started recording episode to {self.episode_dir}")

    def stop_episode(self):
        if not self.is_recording:
            return

        self.is_recording = False
        self._stop_writer = True
        if self.writer_thread is not None:
            self.writer_thread.join()

        if self._failed_image_writes:
            print(f"WARNING: {len(self._failed_image_writes)} image write(s) failed during this episode. "
                  f"Dropping the corresponding frame(s) from trajectory.csv so it never references a missing file:")
            for image_filename in sorted(self._failed_image_writes):
                print(f"  {image_filename}")
            self.records = [
                r for r in self.records
                if Path(r['image_teleop_webcam']).name not in self._failed_image_writes
            ]

        if len(self.records) > 0:
            csv_path = self.episode_dir / "trajectory.csv"
            with open(csv_path, 'w', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=self.records[0].keys())
                writer.writeheader()
                writer.writerows(self.records)
            print(f"Saved {len(self.records)} frames to {csv_path}")
        else:
            print("No frames recorded, episode discarded.")

        self.records = []

    def add(self, timestamp, measured_state, commanded_joints, teleop_image):
        if not self.is_recording:
            return

        image_filename = f"teleop_webcam_{self.frame_index:06d}.jpg"
        image_path = self.image_dir / image_filename

        # Push to background thread
        if teleop_image is not None:
            self.image_queue.put((str(image_path), teleop_image.copy(), image_filename))
        
        rel_image_path = f"images/{image_filename}"
        
        # Calculate absolute base joint targets if not present
        base_theta_joint = measured_state['base']['theta']
        if 'joint_mobile_base_rotate_by' in commanded_joints:
             base_theta_joint += commanded_joints['joint_mobile_base_rotate_by']
        elif 'joint_mobile_base_rotation' in commanded_joints:
             base_theta_joint = commanded_joints['joint_mobile_base_rotation']
             
        record = {
            'timestamp': timestamp - self.start_time,
            'frame_index': self.frame_index,
            
            # Measured States
            'base_x': measured_state['base']['x'],
            'base_y': measured_state['base']['y'],
            'base_theta': measured_state['base']['theta'],
            'lift': measured_state['lift']['pos'],
            'arm': measured_state['arm']['pos'],
            'wrist_roll': measured_state['end_of_arm']['wrist_roll']['pos'],
            'wrist_pitch': measured_state['end_of_arm']['wrist_pitch']['pos'],
            'wrist_yaw': measured_state['end_of_arm']['wrist_yaw']['pos'],
            'gripper_finger_right': measured_state['end_of_arm']['stretch_gripper']['pos'],
            
            # Commanded Actions
            'base_x_joint': measured_state['base']['x'],
            'base_y_joint': measured_state['base']['y'],
            'base_theta_joint': base_theta_joint,
            'joint_lift': commanded_joints.get('joint_lift', measured_state['lift']['pos']),
            'joint_arm_l0': commanded_joints.get('joint_arm_l0', measured_state['arm']['pos']),
            'joint_wrist_roll': commanded_joints.get('joint_wrist_roll', measured_state['end_of_arm']['wrist_roll']['pos']),
            'joint_wrist_pitch': commanded_joints.get('joint_wrist_pitch', measured_state['end_of_arm']['wrist_pitch']['pos']),
            'joint_wrist_yaw': commanded_joints.get('joint_wrist_yaw', measured_state['end_of_arm']['wrist_yaw']['pos']),
            'stretch_gripper': commanded_joints.get('stretch_gripper', measured_state['end_of_arm']['stretch_gripper']['pos']),
            'gripper_width_m': self._convert_gripper_pos_to_m(measured_state['end_of_arm']['stretch_gripper']['pos']),
            'commanded_gripper_width_m': self._convert_gripper_pos_to_m(commanded_joints.get('stretch_gripper', measured_state['end_of_arm']['stretch_gripper']['pos'])),
            
            # Modalities
            'image_teleop_webcam': rel_image_path,
        }
        
        self.records.append(record)
        self.frame_index += 1

    def _image_writer_worker(self):
        while not self._stop_writer or not self.image_queue.empty():
            try:
                # Wait for up to 0.1 seconds for a new item
                image_path, image, image_filename = self.image_queue.get(timeout=0.1)
            except queue.Empty:
                continue

            try:
                success = cv2.imwrite(image_path, image)
            except Exception as e:
                # cv2.imwrite() normally reports failure by returning False rather
                # than raising, but guard against unexpected exceptions too -- an
                # uncaught one here would silently kill this thread and leave every
                # later frame in the episode unwritten without any warning.
                print(f"ERROR: exception while writing image {image_path}: {e}")
                success = False
            else:
                if not success:
                    print(f"ERROR: failed to write image {image_path}")

            if not success:
                self._failed_image_writes.add(image_filename)

            self.image_queue.task_done()

    def _convert_gripper_pos_to_m(self, pos):
        if not hasattr(self, 'gripper_conversion') or not self.gripper_conversion:
            return 0.0
        
        c = self.gripper_conversion
        open_m = c.get('open_aperture_m', 0.09)
        closed_m = c.get('closed_aperture_m', 0.0)
        open_r = c.get('open_robotis', 70.0)
        closed_r = c.get('closed_robotis', 0.0)
        
        if open_r == closed_r:
            return 0.0
            
        m = (pos - closed_r) / (open_r - closed_r) * (open_m - closed_m) + closed_m
        return m
