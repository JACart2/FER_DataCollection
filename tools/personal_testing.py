"""Testing multi-threading implementation with OpenCV

Author: John Rosario Cruz
Version: 3/5/2025
"""
import cv2
from threading import Thread
import threading
from queue import Queue

from fer import FER
from datetime import datetime


# Reopen the video file to restart reading
camera = cv2.VideoCapture(cv2.CAP_DSHOW)
camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)

# Set video frame rate and display delay
fps = 25
frame_delay = int(1000 / fps)

# Prepare sentiment labels and data storage
emotion_labels = ['Angry', 'Disgust', 'Fear', 'Happy', 'Sad', 'Surprise', 'Neutral']
emotion_data = []

# Using a limited-size queue for asynchronous processing
frame_queue = Queue(maxsize=15)
secondary_queue = Queue(maxsize=15)

stop_event = threading.Event()

def process_frames():

    detector = FER(mtcnn=True)

    while not stop_event.is_set():
        emotions = {}
        frame = frame_queue.get()
        
        response = detector.detect_emotions(frame)
        # Store the detected emotion for the current face in the emotions dictionary
        for i, passenger in enumerate(response):
            confidence = 0
            name = ""
            for emotion in passenger['emotions']:
                if passenger['emotions'][emotion] > confidence:
                    confidence = passenger['emotions'][emotion]
                    name = emotion

            emotions[f"Passenger {i + 1}:"] = f"{name}: {confidence * 100:.2f}%"

        print(emotions, 'primary')

def secondary_process_frames():
    detector = FER(mtcnn=True)

    while not stop_event.is_set():
        emotions = {}
        frame = secondary_queue.get()
        
        response = detector.detect_emotions(frame)
        # Store the detected emotion for the current face in the emotions dictionary
        for i, passenger in enumerate(response):
            confidence = 0
            name = ""
            for emotion in passenger['emotions']:
                if passenger['emotions'][emotion] > confidence:
                    confidence = passenger['emotions'][emotion]
                    name = emotion

            emotions[f"Passenger {i + 1}:"] = f"{name}: {confidence * 100:.2f}%"
        
        print(emotions, 'secondary')


def main():

    # Start the thread that processes the frame
    thread_process = Thread(target=process_frames, daemon=True)
    secondary_thread_process = Thread(target=secondary_process_frames, daemon=True)
    thread_process.start()
    secondary_thread_process.start()

    # main loop
    times = [datetime.now().timestamp()]
    primary = True

    try:
        while True:
            ret, frame = camera.read()

            if not ret:
                break ## video feed is terminated

            if primary:
                if not frame_queue.full():
                    frame_queue.put(frame)
                else:
                    _ = frame_queue.get()  # discard oldest frame
                    secondary_queue.put(frame)
            else:
                if not secondary_queue.full():
                    secondary_queue.put(frame)
                else:
                    _ = secondary_queue.get()  # discard oldest frame
                    frame_queue.put(frame)
            
            primary = not primary
    except KeyboardInterrupt:
        ## catch ctrl-c to close threads
        pass

    times.append(datetime.now().timestamp())

    # Notification thread ends
    stop_event.set()

    # Wait for thread to end
    thread_process.join()
    secondary_thread_process.join()

    # Clean and save data
    cv2.destroyAllWindows()
    camera.release()


if __name__ == '__main__':
    main()