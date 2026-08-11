import numpy as np
import cv2
import pyrealsense2 as rs
import stretch_body.hello_utils as hu


class RealSenseCamera:
    def __init__(self, device_name, color_size=(640, 480), fps=15, rotate_90_clockwise=False):
        realsense_ctx = rs.context()
        connected_devices = {}
        for i in range(len(realsense_ctx.devices)):
            name = realsense_ctx.devices[i].get_info(rs.camera_info.name)
            serial = realsense_ctx.devices[i].get_info(rs.camera_info.serial_number)
            connected_devices[name] = serial

        if device_name not in connected_devices:
            raise RuntimeError(f"Unable to find {device_name} -- is it connected?")

        self.device_name = device_name
        self.rotate_90_clockwise = rotate_90_clockwise

        serial = connected_devices[device_name]
        self.pipeline = hu.setup_realsense_camera(
            serial_number=serial,
            color_size=list(color_size),
            depth_size=list(color_size),  # depth stream is still configured even though we only read color
            fps=fps,
        )

    def get_next_frame(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        image = np.asanyarray(color_frame.get_data())
        if self.rotate_90_clockwise:
            image = cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE)
        return image

    def __del__(self):
        print(f'RealSenseCamera.__del__: stopping the {self.device_name} pipeline.')
        self.pipeline.stop()
