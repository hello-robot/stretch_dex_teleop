
import cv2
import time
import subprocess
from pathlib import Path
import webcam as wc
from image_processing_helpers import fit_image_to_screen


camera_name = 'Logitech Webcam C930e'
image_width = 1920
image_height = 1080
fps = 30

num_images_to_collect = 60
time_between_images_sec = 0.5
image_directory = wc.get_calibration_directory(camera_name, image_width, image_height)
image_base_name = 'webcam_calibration_image'

Path(image_directory).mkdir(parents=True, exist_ok=True)

webcam =wc.Webcam(
    camera_name=camera_name, 
    fps=fps,
    image_width=image_width,
    image_height=image_height,
    use_calibration=False,
    show_images=False
)

prev_save_time = time.time()
num_images = 0

while num_images < num_images_to_collect:
    
    color_image, camera_info = webcam.get_next_frame()

    cv2.imshow('image from camera', fit_image_to_screen(color_image))
    cv2.waitKey(1)

    curr_time = time.time()

    if (curr_time - prev_save_time) > time_between_images_sec:
        num_images = num_images + 1
        file_name = image_directory + image_base_name + '_' + str(num_images).zfill(4) + '.png'
        print('save', file_name)
        cv2.imwrite(file_name, color_image)
        prev_save_time = curr_time

