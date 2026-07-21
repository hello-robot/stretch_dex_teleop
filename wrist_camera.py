import numpy as np
import pyrealsense2 as rs
import stretch_body.hello_utils as hu


class WristCamera:
    def __init__(self, color_size=(640, 480), fps=15):
        realsense_ctx = rs.context()
        connected_devices = {}
        for i in range(len(realsense_ctx.devices)):
            name = realsense_ctx.devices[i].get_info(rs.camera_info.name)
            serial = realsense_ctx.devices[i].get_info(rs.camera_info.serial_number)
            connected_devices[name] = serial

        if 'Intel RealSense D405' not in connected_devices:
            raise RuntimeError("Unable to find Intel RealSense D405 -- is it connected?")

        serial = connected_devices['Intel RealSense D405']
        self.pipeline = hu.setup_realsense_camera(
            serial_number=serial,
            color_size=list(color_size),
            depth_size=list(color_size),  # depth stream is still configured even though we only read color
            fps=fps,
        )

    def get_next_frame(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        return np.asanyarray(color_frame.get_data())

    def __del__(self):
        print('WristCamera.__del__: stopping the D405 pipeline.')
        self.pipeline.stop()
