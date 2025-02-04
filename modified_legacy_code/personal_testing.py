import cv2

from fer import FER
import time
import datetime

import csv
import numpy as np
import pyzed.sl as sl

## faces


paths = [r'C:\Users\gr8jj\OneDrive\Desktop\SPRING 2025\CS497\fake_faces\neutral_face.jpg',
r'C:\Users\gr8jj\OneDrive\Desktop\SPRING 2025\CS497\fake_faces\angry_face.jpg',
r'C:\Users\gr8jj\OneDrive\Desktop\SPRING 2025\CS497\fake_faces\disgusted_face.jpg',
r'C:\Users\gr8jj\OneDrive\Desktop\SPRING 2025\CS497\fake_faces\happy_face.jpg',
r'C:\Users\gr8jj\OneDrive\Desktop\SPRING 2025\CS497\fake_faces\surprised_face.jpg']


def detect_faces(image):
    """Utilizes the Haar cascade classifier (opencv) to detect faces.
        It calculates and scales the image and grayscales it for better accuracy.

    Args:
        image (numpy.ndarray): Image object (one frame) generated by ZED camera

    Returns:
        numpy.ndarray: coordinates of the detected face
            >>> [[681 265 492 492]]

    """
    # load pre-trained model (haarcascade)
    face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

    # Detect faces in the grayscale image using the detectMultiScale method of the face cascade
    # scaleFactor: Parameter specifying how much the image size is reduced at each image scale.
    # minNeighbors: Parameter specifying how many neighbors each rectangle should have to retain it.
    faces = face_cascade.detectMultiScale(image, scaleFactor=1.3, minNeighbors=5)

    # Return the coordinates of the detected faces (x, y, width, height)
    return faces


def detect_emotion(image, faces):
    """Detects emotion given image n-array, and face coords.

    Args:
        image (numpy.ndarray): the image

        faces (numpy.ndarray): coordinates of the face

    Returns:
        tuple: The modified image to include analysis & a dictionary representing the top emotion.
            >>> ([[12, 123, 543, 53, 534]], {'Passenger 1': "angry"})

    """
    emotions = {}

    # Initialize the FER (Facial Expression Recognition) detector with MTCNN (Multi-Task Cascaded Convolutional Networks) 
    detector = FER(mtcnn=True)

    # # Convert the ZED image to a format usable by OpenCV
    # image_cv2 = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    # Iterate over each detected face
    for i, (x, y, w, h) in enumerate(faces):
        # Crop the face region from the image
        face_img = image[y:y+h, x:x+w]

        # resize grayscale image to mimic RGB dimensionality
        if len(face_img.shape) == 2:
            face_img = np.repeat(face_img[..., np.newaxis], 3, axis=-1)

        # Detect emotion for the face using the FER detector
        emotion, _ = detector.top_emotion(face_img)

        # Store the detected emotion for the current face in the emotions dictionary
        emotions[f"Passenger {i+1}"] = emotion

        # Draw bounding box around the detected face
        cv2.rectangle(image, (x, y), (x+w, y+h), (255, 0, 0), 2)

        # Put text indicating the emotion on the image
        cv2.putText(image, f"Passenger {i+1}: {emotion}", (x, y + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (36, 255, 12), 0)

    # Return the modified image with bounding boxes and emotion labels, along with the emotions dictionary
    return image, emotions


def test_full_local_images():
    print(int(time.time()), 'PRE loop')
    for path in paths:

        image = cv2.imread(path)

        # Check if the image was loaded successfully
        if image is None:
            print("Error: Could not load image.")
        else:
            face_coords = detect_faces(image)
            response = detect_emotion(image, face_coords)
            print(face_coords, 'fc')
            cv2.imshow('Image', response[0])

            # Wait for any key to close the window
            cv2.waitKey(0)
            cv2.destroyAllWindows()

    print(int(time.time()), 'POST loop')


def test_local_csv_data():
    data = []
    print(int(time.time()), 'PRE data')
    ## loading and cleaning face data
    with open(r"C:\Users\gr8jj\OneDrive\Desktop\SPRING 2025\CS497\fake_faces\fer2013.csv\fer2013.csv", mode='r', newline='', encoding='utf-8') as file:
        file.readline()
        reader = csv.reader(file)
        count = 0
        for row in reader:
            count += 1

            raw_pixels = list(map(int, row[1].split()))
            formatted_pixels = np.array(raw_pixels, dtype=np.uint8).reshape((48, 48))
            data.append(formatted_pixels)

            if count >= 3:
                break

    print(int(time.time()), 'POST data')
    ## analysis with face data
    #emotions = []
    print(int(time.time()), 'pre analysis')
    for face in data[:61]:
        response = detect_emotion(face, [[0, 0, 48, 48]])
        cv2.imshow('Image', response[0])

        # Wait for any key to close the window
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        #emotions.append(response[1])

    print(int(time.time()), 'POST analysis')


def __process_1D_array_csv__(path, size, limit=10):
    """Process 1D arrays from a CSV to openCV formatting.

    Args:
        path (str): e.g. C:\\Users\\gr8jj\\OneDrive\\Desktop\\SPRING 2025\\CS497\\fake_faces\\fer2013.csv\\fer2013.csv

        size (int): the pixel structure of the image 48 = (48 x 48), 64 = (64 x 64)

        limit (int): the max number of faces to process.

    Returns:
        list: A list of np.arrays. Each array represents the pixels of an image


    """
    data = []

    # loading and cleaning face data
    with open(path, mode='r', newline='', encoding='utf-8') as file:
        # instantiate csv reader object
        file.readline()
        reader = csv.reader(file)

        count = 0
        for row in reader:
            count += 1

            raw_pixels = list(map(int, row[1].split()))
            formatted_pixels = np.array(raw_pixels, dtype=np.uint8).reshape((size, size))
            data.append(formatted_pixels)

            if count >= limit:
                break

    return data





## Testing vidoe input from ZED ##

def detect_emotion_alt(image):
    """Detects emotion given image n-array, and face coords.

    Args:
        image (numpy.ndarray): the image

    Returns:
        tuple: The modified image to include analysis & a dictionary representing the top emotion.
            >>> ([[12, 123, 543, 53, 534]], {'Passenger 1': "angry"})

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

    # Return the modified image with bounding boxes and emotion labels, along with the emotions dictionary
    return image, emotions


# Function that processes each frame
def frame_processing(frame):    
    # Detect emotions for each face in the frame
    emotion_frame, emotions = detect_emotion_alt(frame)

    print(emotions)


def test_20fps_cam_input():
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
            frame_data = image.get_data()

            # Call the test function to process the frame
            frame_processing(frame_data)


if __name__ == '__main__':
    test_20fps_cam_input()
