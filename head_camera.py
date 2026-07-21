from realsense_camera import RealSenseCamera


class HeadCamera(RealSenseCamera):
    def __init__(self, color_size=(640, 480), fps=30):
        super().__init__(
            device_name='Intel RealSense D435I',
            color_size=color_size,
            fps=fps,
            rotate_90_clockwise=True,
        )
