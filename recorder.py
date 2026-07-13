import os
import time
import json
import threading
import queue
import cv2
import pandas as pd
from pathlib import Path

class EpisodeRecorder:
    def __init__(self, data_dir="data", fps=30):
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
            
        if len(self.records) > 0:
            df = pd.DataFrame(self.records)
            csv_path = self.episode_dir / "trajectory.csv"
            df.to_csv(csv_path, index=False)
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
            self.image_queue.put((str(image_path), teleop_image.copy()))
        
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
            
            # Modalities
            'image_teleop_webcam': rel_image_path,
        }
        
        self.records.append(record)
        self.frame_index += 1

    def _image_writer_worker(self):
        while not self._stop_writer or not self.image_queue.empty():
            try:
                # Wait for up to 0.1 seconds for a new item
                image_path, image = self.image_queue.get(timeout=0.1)
                cv2.imwrite(image_path, image)
                self.image_queue.task_done()
            except queue.Empty:
                continue
