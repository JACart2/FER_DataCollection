"""Emotion Detection script for ZED Camera.

The video is captured at 20 frames per second (fps).
This script primarily utilizes CPU for facial emotion detection, suitable for laptop use.
The identified faces are sent to FER (MTCNN) for emotion recognition.

Author: Dominic Nguyen
Modified by: John Rosario Cruz

Version: 1/30/2025
"""
# The FER library (Facial Emotion Recognition Library) is used for emotion using MTCNN: 
# MTCNN (Multi-Task Cascaded Convolutional Networks) is a deep learning architecture designed for face detection and alignment.
# It consists of three stages: face detection, facial landmark localization, and face alignment.
from fer import FER

import cv2
import numpy as np

# Pyzed Library is used for the init of the ZED 2i camera.
import pyzed.sl as sl


def detect_emotion(image):
    """Detects emotion given image n-array.

    Args:
        image (numpy.ndarray): the image

    Returns:
        dict: A dictionary representing the top emotion.
            >>> {'Passenger 1': "angry"}

    """
    emotions = {}

    # Initialize the FER (Facial Expression Recognition) detector with MTCNN (Multi-Task Cascaded Convolutional Networks) 
    detector = FER(mtcnn=True)

    # resize grayscale image to mimic RGB dimensionality
    if len(image.shape) == 2:
        image = np.repeat(image[..., np.newaxis], 3, axis=-1)

    # Detect emotion for the faces using the FER detector
    response = detector.detect_emotions(image)

    # Store the detected emotion for the current face in the emotions dictionary
    for i, passenger in enumerate(response):
        confidence = 0
        name = ""
        for emotion in passenger['emotions']:
            if passenger['emotions'][emotion] > confidence:
                confidence = passenger['emotions'][emotion]
                name = emotion

        emotions[f"Passenger {i + 1}:"] = f"{name}: {confidence * 100:.2f}%"

    # the emotions dictionary
    return emotions

def frame_processing(frame):    
    # Detect emotions for each face in the frame
    emotions = detect_emotion(frame)

    print(emotions)


def run_main():
    # config cam stats
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD720  
    init_params.camera_fps = 30

    zed = sl.Camera()
    if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
        print("Error opening ZED camera")
        exit(1)


    # Continuously capture frames from the live feed
    while True:
        # Grab a frame
        if zed.grab() == sl.ERROR_CODE.SUCCESS:
            # Retrieve the left image
            image = sl.Mat()
            zed.retrieve_image(image, sl.VIEW.LEFT_GRAY)

            # Convert to numpy array for easier handling in the test function
            frame_data_raw = image.get_data()

            ## potential improvement
            """
            # Converts the frames to greyscale and equalizes to improve accuracy
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            equalizedFrame = cv2.equalizeHist(gray)
            processedFrame = cv2.cvtColor(equalizedFrame, cv2.COLOR_GRAY2BGR)
            """

            ## implement check if there is a face, if yes, assert its there for 5 iterations, then process
            ## using deepface

            # Call the test function to process the frame
            frame_processing(frame_data_raw)


if __name__ == '__main__':
    run_main()