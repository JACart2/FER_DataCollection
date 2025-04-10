import cv2
import mediapipe as mp
import numpy as np
import time
from datetime import datetime
import pyzed.sl as sl

# Create the box param
frame_width = 640
frame_height = 480
box_width, box_height = 300, 400  # Adjusted to make width short and height longer

# Calculate the top-left corner of the box to center it in the frame
box_x = (frame_width - box_width) // 2 
box_y = (frame_height - box_height) // 2

# Open a file for logging body/face out of frame events
log_file = open("body_face_out_of_frame_log.txt", "a")

# Initialize MediaPipe Pose model
mp_pose = mp.solutions.pose
pose = mp_pose.Pose()

# Initialize ZED camera
zed = sl.Camera()

# Config for ZED camera
init_params = sl.InitParameters()
init_params.camera_resolution = sl.RESOLUTION.HD720 
init_params.camera_fps = 30  
init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE 

# Open the ZED camera
if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
    print("Failed to open ZED camera")
    exit(1)

# Create Mat to hold ZED camera frames
image = sl.Mat()

# Timer variables
last_detection_time = time.time()  
no_detection_duration = 4 

# Body part names for parts above the waist 
landmark_names = [
    'NOSE', 'LEFT_EYE', 'RIGHT_EYE', 'LEFT_EAR', 'RIGHT_EAR',
    'LEFT_SHOULDER', 'RIGHT_SHOULDER', 'LEFT_ELBOW', 'RIGHT_ELBOW',
    'LEFT_WRIST', 'RIGHT_WRIST', 'LEFT_PINKY', 'RIGHT_PINKY', 'LEFT_INDEX',
    'RIGHT_INDEX', 'LEFT_THUMB', 'RIGHT_THUMB'
]

while True:
    # Grab a frame from the ZED camera
    if zed.grab() == sl.ERROR_CODE.SUCCESS:
        # Retrieve the left image from the ZED camera
        zed.retrieve_image(image, sl.VIEW.LEFT)  # Get left camera image
        frame = image.get_data()

    # Resize frame for consistent detection
    frame = cv2.resize(frame, (frame_width, frame_height))

    # Draw the centered box
    cv2.rectangle(frame, (box_x, box_y), (box_x + box_width, box_y + box_height), (255, 0, 0), 2)

    # Convert frame to RGB as MediaPipe works with RGB
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # Process the frame with MediaPipe Pose to detect body landmarks
    results = pose.process(frame_rgb)

    # Check if any point/ landmarks are detected
    if results.pose_landmarks:
        # Iterate over the detected points
        for i, landmark in enumerate(results.pose_landmarks.landmark):
            # Ensure the index is within the range of points
            if i < len(landmark_names):
                # Get the person;s position
                x = int(landmark.x * frame_width)
                y = int(landmark.y * frame_height)

                # Draw the skeleton points
                cv2.circle(frame, (x, y), 5, (0, 255, 0), -1)  # Green for landmarks

                # Check if the person is inside or outside the box
                if x >= box_x and y >= box_y and x <= (box_x + box_width) and y <= (box_y + box_height):
                    # Person is inside the box, next
                    pass
                else:
                    # Print which body part is out of the box
                    part_name = landmark_names[i]
                    print(f"{part_name} is out of the box!")

                    # Log the event in the log file
                    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                    log_file.write(f"{timestamp} - {part_name} out of box\n")

    # Display the resulting frame
    cv2.imshow('Body Detection', frame)

    # Press 'q' to exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the capture and close all windows
zed.close()
cv2.destroyAllWindows()

# Close the log file
log_file.close()

