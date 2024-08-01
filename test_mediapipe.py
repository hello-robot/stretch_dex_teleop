import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision

from webcam import Webcam

def main():
    base_options = python.BaseOptions(model_asset_path='hand_landmarker.task')
    options = vision.HandLandmarkerOptions(base_options=base_options,
                                        num_hands=2)
    detector = vision.HandLandmarker.create_from_options(options)

    cam = Webcam(show_images=False, use_second_camera=False)

    while True:
        image, _ = cam.get_next_frame()
        img = mp.Image(image_format=mp.ImageFormat.SRGB, data=image)
        detection_result = detector.detect(img)
        # print(detection_result)
        print(len(detection_result.hand_landmarks))

if __name__ == '__main__':
    main()
