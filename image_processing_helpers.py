import os
from Xlib import display
import cv2

# Set the DISPLAY environment variable if not already set
if 'DISPLAY' not in os.environ:
    os.environ['DISPLAY'] = ':0'

def get_screen_resolution():
    screen = display.Display().screen()
    width = screen.width_in_pixels
    height = screen.height_in_pixels
    return width, height

def fit_image_to_screen(img: cv2.typing.MatLike, ratio: float=0.75) -> cv2.typing.MatLike:
    """
    Resizes an image to fit the screen resolution.

    Args:
        img (MatLike): The image to resize.
        ratio (float): The ratio of the screen resolution to use for resizing the image.

    Returns:
        MatLike: The resized image.
    """

    screen_width, screen_height = get_screen_resolution()
    img_width = int(ratio * screen_width)
    img_height = int(ratio * screen_height)
    return cv2.resize(img, [img_width, img_height])
