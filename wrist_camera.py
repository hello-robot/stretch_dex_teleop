from realsense_camera import RealSenseCamera


class WristCamera(RealSenseCamera):
    def __init__(self, color_size=(640, 480), fps=15):
        super().__init__(
            device_name='Intel RealSense D405',
            color_size=color_size,
            fps=fps,
            rotate_90_clockwise=False,
        )
