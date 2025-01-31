# Facial Emotion Detection Script
# Author: Dominic Nguyen
# Version: 5
# Date: 04/09/2024
import cv2
from fer import FER
import os
import pyzed.sl as sl
from collections import defaultdict
from datetime import datetime  

# The video is captured at 20 frames per second (fps).
# Emotions are determined per frame using the Haar cascade classifier provided by OpenCV (cv2.CascadeClassifier).
# The Haar cascade model is pre-trained by OpenCV to detect basic emotions including happiness, sadness, anger, surprise, fear, and disgust.
# This script primarily utilizes CPU for facial emotion detection, suitable for laptop use.
# The Haar cascade model is pre-trained against various patterns and features indicative of the specified emotions,
# enabling it to detect these basic emotions without requiring additional training data.
# OS library is used to perform the act of creating files via the file directory.
# The FER library (Facial Emotion Recognition Library) is used for emotion using MTCNN: 
# MTCNN (Multi-Task Cascaded Convolutional Networks) is a deep learning architecture designed for face detection and alignment.
# It consists of three stages: face detection, facial landmark localization, and face alignment.
# MTCNN accurately detects faces and locates facial landmarks like eyes, nose, and mouth.
# Pyzed Library is used for the initilization of the ZED 2i camera.
# DefaultDict is used to create a dictory to store the emotions into. 
# DateTime libary is used to embed both dates and times into the log file, allowing for more accurate analysis later on. 

# Save Emotion Script Directory -- output file of emotions will be saved here.
save_dir = r"/home/jacart/Desktop/EmotionTest"


# This function utilizes the Haar cascade classifier (opencv) to detect faces. 
# It calculates and scales the image and grayscales it for better accuracy.
def detect_faces(image):
    # Load the pre-trained Haar cascade classifier for frontal faces.
    # This file is part of the OpenCV library and contains pre-trained data for detecting frontal faces.
    face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
    
    # Convert the input image to grayscale as face detection works on grayscale images
    # This allows for less variability when its comes to detection, allowing for more accurate detection.
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    # Detect faces in the grayscale image using the detectMultiScale method of the face cascade
    # scaleFactor: Parameter specifying how much the image size is reduced at each image scale.
    # minNeighbors: Parameter specifying how many neighbors each rectangle should have to retain it.
    faces = face_cascade.detectMultiScale(gray, scaleFactor=1.3, minNeighbors=5)
    
    # Return the coordinates of the detected faces (x, y, width, height)
    return faces

# This function is used to detect emotion
def detect_emotion(image, faces):
    # Create an empty dictionary to store emotions detected for each face. Facial emotions will be appended to this list.
    emotions = {}
    
    # Initialize the FER (Facial Expression Recognition) detector with MTCNN (Multi-Task Cascaded Convolutional Networks) enabled
    # Uses a special face detection method called MTCNN
    # MTCNN does many things at once: finds faces, spots facial features like eyes and nose, and improves accuracy with each step
    # It's based on special math called convolutional networks, which are great at understanding images
    # By enabling MTCNN, we're telling our detector to use this smart method for finding faces
    #This helps us find faces accurately before figuring out their emotions
    detector = FER(mtcnn=True)

    # Convert the ZED image (ZED is a depth-sensing camera) to a format usable by OpenCV
    image_cv2 = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    # Iterate over each detected face
    for i, (x, y, w, h) in enumerate(faces):
        # Crop the face region from the image
        face_img = image_cv2[y:y+h, x:x+w]
        
        # Detect emotion for the face using the FER detector
        emotion, _ = detector.top_emotion(face_img)
        
        # Store the detected emotion for the current face in the emotions dictionary
        emotions[f"Passenger {i+1}"] = emotion
        
        # Draw bounding box around the detected face
        cv2.rectangle(image_cv2, (x, y), (x+w, y+h), (255, 0, 0), 2)
        
        # Put text indicating the emotion on the image
        cv2.putText(image_cv2, f"Passenger {i+1}: {emotion}", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (36, 255, 12), 2)

    # Return the modified image with bounding boxes and emotion labels, along with the emotions dictionary
    return image_cv2, emotions


def main():
    # Print instructions for user
    print("Press 'q' to stop the video capture.")
    print("Press 's' to pause/resume the video capture.")

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

    # Create a VideoWriter object to save the captured video as an MP4 file
    # Captures as 20 fps
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(os.path.join(save_dir, 'output.mp4'), fourcc, 20.0, (1280, 720))

    # List to store emotions detected in each frame
    emotions_per_frame = []  
    # List to store timestamps for each frame
    timestamps = []  
    # Flag to control video capture
    capture_active = True  

    while True:
        if capture_active:
            # Grab a frame from the ZED camera
            zed.grab(runtime)
            image = sl.Mat()
            zed.retrieve_image(image, sl.VIEW.LEFT)

            # Convert ZED image to format usable by OpenCV
            frame = image.get_data()[:, :, :3]

            # Detect faces in the frame
            faces = detect_faces(frame)

            # Detect emotions for each face in the frame
            frame, emotions = detect_emotion(frame, faces)
            emotions_per_frame.append(emotions)

            # Get current timestamp and store it
            current_timestamp = datetime.now()
            timestamps.append(current_timestamp)

            # Display date and time on the frame
            current_date_time = current_timestamp.strftime("%Y-%m-%d %H:%M:%S")
            cv2.putText(frame, f"Date-Time: {current_date_time}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)

            # Write frame to video
            out.write(frame)

            # Display the frame
            cv2.imshow('frame', frame)

        # Check for key presses
        key = cv2.waitKey(50)
        if key & 0xFF == ord('q'):
            break
        elif key & 0xFF == ord('s'):
            # Toggle video capture
            capture_active = not capture_active
            if capture_active:
                print("Video capture resumed.")
            else:
                print("Video capture paused.")

    # Write emotions to a file with date and time
    emotions_file = os.path.join(save_dir, 'emotions.txt')
    with open(emotions_file, 'w') as f:
        # Write emotions detected for each frame
        for i, (emotions, timestamp) in enumerate(zip(emotions_per_frame, timestamps), start=1):
            if any(emotions.values()):  # Check if any emotion is captured
                current_datetime = timestamp.strftime("%Y-%m-%d %H:%M:%S")
                f.write(f"Frame {i} - Date: {current_datetime}:\n")
                for passenger, emotion in emotions.items():
                    f.write(f"{passenger}: {emotion}\n")

        # Write top three emotions for each passenger
        f.write("\nTop Three Emotions for Each Passenger:\n")
        for passenger, emotion_dict in get_top_three_emotions(emotions_per_frame).items():
            filtered_emotions = [emotion for emotion in emotion_dict if emotion is not None]
            f.write(f"{passenger}: {', '.join(filtered_emotions)}\n")

    print("Emotions count saved to:", emotions_file)

    # Close ZED camera and release VideoWriter
    zed.close()
    out.release()
    cv2.destroyAllWindows()

def get_top_three_emotions(emotions_per_frame):
    # Initialize a dictionary to store counts of each emotion PER passenger
    passenger_emotions = defaultdict(lambda: defaultdict(int))

    # Iterate over the list of emotions detected in each frame
    for emotions in emotions_per_frame:
        # Iterate over each passenger's emotions in the current frame
        for passenger, emotion in emotions.items():
            # Increment the count of the detected emotion for the current passenger (If happy is seen once, +1 for happy -- etc.)
            passenger_emotions[passenger][emotion] += 1

    # Initialize a dictionary to store the top three emotions for each passenger
    top_three_emotions = {}
    
    # Iterate over each passenger's emotions and find the top three most frequent emotions
    for passenger, emotions in passenger_emotions.items():
        # Sort emotions based on their counts in descending order and take the top three. 
        # Emotions are sorted based off frequency, the highest seen emotion is outputed first, etc.
        top_three = sorted(emotions, key=emotions.get, reverse=True)[:3]
        # Store the top three emotions for the current passenger
        top_three_emotions[passenger] = top_three

    return top_three_emotions


if __name__ == "__main__":
    main()