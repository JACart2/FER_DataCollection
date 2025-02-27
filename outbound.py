import cv2
from datetime import datetime
import time  # Import time module to track elapsed time

# Load the pre-trained face and body detection models
# These models help detect faces and bodies in the video stream.
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
body_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_fullbody.xml')

# Define the standard box size (adjusted: shorter width, longer length)
# The dimensions of the detection box are set to 300 (width) x 400 (height), with the width being shorter and the height being longer.
frame_width = 640
frame_height = 480
box_width, box_height = 300, 400  # Adjusted width (shorter) and height (longer)

# Calculate the top-left corner of the box to center it in the frame
# The coordinates of the top-left corner are calculated to ensure the box is centered in the video frame.
box_x = (frame_width - box_width) // 2  # Center the box horizontally
box_y = (frame_height - box_height) // 2  # Center the box vertically

# Open a file for logging body/face out of frame events
# A log file is opened to record the times when faces/bodies are detected outside the box.
log_file = open("body_face_out_of_frame_log.txt", "a")

# Start capturing video
# The video capture object (cap) is initialized to capture frames from the default webcam (0).
cap = cv2.VideoCapture(0)

# Set video resolution to a higher value to detect distant objects better
# The resolution is set to 1280x720 to capture higher quality video for better detection at greater distances.
cap.set(3, 1280)  # Set width
cap.set(4, 720)   # Set height

# Timer variables
# Track the last time a face was detected and set the duration to wait for no face (in seconds).
last_face_time = time.time()  # Track the last time a face was detected
no_face_detected_duration = 4 # Duration to wait for no face (in seconds)

while True:
    # Capture frame-by-frame
    # Capture a frame from the video feed.
    ret, frame = cap.read()

    # Check if the frame is valid
    # If the frame is empty or invalid, skip to the next frame.
    if frame is None or frame.size == 0:
        print("Empty frame detected, skipping analysis.")
        continue

    # Resize frame for consistent detection
    # Resize the frame to the defined resolution (640x480) to maintain consistent detection performance.
    frame = cv2.resize(frame, (frame_width, frame_height))

    # Convert frame to grayscale for CascadeClassifier
    # Convert the captured frame to grayscale as it is required for the face and body detection models.
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect faces and bodies with adjusted parameters for better range
    # Apply face detection using the pre-trained face cascade classifier.
    faces = face_cascade.detectMultiScale(
        gray_frame,
        scaleFactor=1.3,  # Increase for better range
        minNeighbors=3,   # Lower for more sensitivity
        minSize=(50, 50)  # Adjust for better face detection range
    )

    # Apply body detection using the pre-trained body cascade classifier.
    bodies = body_cascade.detectMultiScale(
        gray_frame,
        scaleFactor=1.3,  # Increase for better range
        minNeighbors=3,   # Lower for more sensitivity
        minSize=(50, 100) # Adjust for better body detection range
    )

    # Draw the centered box (for reference)
    # Draw a rectangle in the center of the frame to represent the detection area (the box).
    cv2.rectangle(frame, (box_x, box_y), (box_x + box_width, box_y + box_height), (255, 0, 0), 2)

    # Track if a face is detected
    # A flag to indicate if a face is detected in the current frame.
    face_detected = False

    # Check faces and bodies together
    # Combine the face and body detections into one list for easier processing.
    all_detections = []
    
    # Add detected faces to the list
    for (x, y, w, h) in faces:
        all_detections.append((x, y, w, h, 'face'))
        face_detected = True  # Face detected, reset the timer

    # Add detected bodies to the list
    for (x, y, w, h) in bodies:
        all_detections.append((x, y, w, h, 'body'))

    # If no face is detected for 5 seconds, log the event
    # If no face is detected for the specified duration, log that the person is outside the box.
    if not face_detected:
        if time.time() - last_face_time >= no_face_detected_duration:
            print("Nothing detected")
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            log_file.write(f"{timestamp} - Person is outside the box\n")

    else:
        # Reset the timer if a face is detected
        last_face_time = time.time()

    # Track whether any face or body is inside or out of the box
    # For each detection (either face or body), check if it is inside the box or outside.
    for (x, y, w, h, label) in all_detections:
        # Draw the rectangle around the detected face/body
        # Draw a green rectangle for faces and a red rectangle for bodies.
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0) if label == 'face' else (0, 0, 255), 2)

        # Check if the face or body is inside the box (all corners of the bounding box must be inside)
        if x >= box_x and y >= box_y and (x + w) <= (box_x + box_width) and (y + h) <= (box_y + box_height):
            print("Person is inside the box!")
        else:
            print("Person is out of the box!")
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            log_file.write(f"{timestamp} - {label.capitalize()} out of box\n")

    # Display the resulting frame
    # Display the current frame with the face/body and box annotations.
    cv2.imshow('Body Detection', frame)

    # Press 'q' to exit
    # If the 'q' key is pressed, exit the loop and close the window.
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the capture and close all windows
# Release the video capture object and close all OpenCV windows.
cap.release()
cv2.destroyAllWindows()

# Close the log file
# Close the log file when the program ends.
log_file.close()

