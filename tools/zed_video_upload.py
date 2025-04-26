"""Upload a video from ZED camera to local dir.

Author: John Rosario Cruz
Version: 2/18/2025
"""
# OpenCV Package
import cv2

# The FER library (Facial Emotion Recognition Library) is used for emotion using MTCNN: 
# MTCNN (Multi-Task Cascaded Convolutional Networks) is a deep learning architecture designed for face detection and alignment.
# It consists of three stages: face detection, facial landmark localization, and face alignment.
from fer import FER

# Used for outputting data to files
import os

# Pyzed Library is used for the init of the ZED 2i camera.
import pyzed.sl as sl

def upload_video():
    # Save Emotion Script Directory -- output file of emotions will be saved here.
    output_path = r"C:\Users\gr8jj\OneDrive\Desktop\SPRING 2025\CS497\FER_DataCollection\tools\output.svo"

    ## initialize cam

    # Create ZED camera object and open it. Initializes ZED Camera (works for ZED 2i)
    init = sl.InitParameters()
    init.camera_resolution = sl.RESOLUTION.HD720
    zed = sl.Camera()
    if not zed.is_opened():
        print("Opening ZED Camera...")
        status = zed.open(init)
        if status != sl.ERROR_CODE.SUCCESS:
            print(repr(status))
            exit()

    runtime = sl.RuntimeParameters()

    ## recording

    # Enable recording with the filename specified in argument
    recordingParameters = sl.RecordingParameters()
    recordingParameters.compression_mode = sl.SVO_COMPRESSION_MODE.H264
    recordingParameters.video_filename = output_path
    err = zed.enable_recording(recordingParameters)

    exit_app = False
    while not exit_app:
        # Each new frame is added to the SVO file
        zed.grab()

    # Disable recording
    zed.disable_recording()

def read_video(path):
    # Create a ZED camera object
    zed = sl.Camera()

    # Set SVO path for playback
    input_path = path
    init_parameters = sl.InitParameters()
    init_parameters.set_from_svo_file(input_path)

    # Open the ZED
    zed = sl.Camera()
    err = zed.open(init_parameters)

    svo_image = sl.Mat()
    exit_app = False
    while not exit_app:
        if zed.grab() == sl.ERROR_CODE.SUCCESS:
            # Read side by side frames stored in the SVO
            zed.retrieve_image(svo_image, sl.VIEW.SIDE_BY_SIDE)
            # Get frame count
            svo_position = zed.get_svo_position()
        elif zed.grab() == sl.ERROR_CODE.END_OF_SVOFILE_REACHED:
            print("SVO end has been reached. Looping back to first frame")
            zed.set_svo_position(0)
            
def main():
    read_video(r"C:\Users\gr8jj\OneDrive\Desktop\SPRING 2025\CS497\FER_DataCollection\tools\output.svo2")


if __name__ == '__main__':
    main()