"""Emotion Recognition of input video feed using multi-threading.

Author: John Rosario Cruz
Based off "Facial-Recognition" by: Donghao Liu
Version: 2/20/2025
"""
import cv2
from threading import Thread
import threading
from queue import Queue

from fer import FER
from datetime import datetime
import numpy as np
import pyzed.sl as sl
import time
import copy
import re

from openai_call import call

class EmotionRecognition():
    def __init__(self):
        # config cam stats
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.VGA  
        init_params.camera_fps = 30

        self.camera = sl.Camera()

        if self.camera.open(init_params) != sl.ERROR_CODE.SUCCESS:
            print("Error opening ZED camera")
            exit(1)

        # Set video frame rate and display delay
        fps = 30
        self.frame_delay = float(1 / fps)

        # Prepare sentiment labels and data storage
        self.emotion_data = []
        self.detector = FER(mtcnn=True)


        # Using a limited-size queue for asynchronous processing
        self.frame_queue = Queue(maxsize=15)
        self.secondary_queue = Queue(maxsize=15)
        self.stop_event = threading.Event()
        self.alert = False


    def monitor(self, frame):
        """Monitor the events coming from the threads.
        
        Args:
            frame (np matrix): The frame tensor representing the image.

        Returns:
            str: The message saying ALERT ALERT ALERT
        """
        ## needs to catch a frame to send to openai
        emotion_copy = copy.deepcopy(self.emotion_data)
        self.emotion_data = []

        emotion_count = [re.match(r'(\w+):', item).group(1) for item in emotion_copy]

        
        ## find the top emotion
        top_emotion = "neutral"
        count = 0
        for emotion in set(emotion_count):
            if emotion_count.count(emotion) >= count:
                count = emotion_count.count(emotion)
                top_emotion = emotion

        ## find average confidence of each frame of the top emotion
        confidence = []
        for frame in emotion_copy:
            if top_emotion in frame:
                confidence.append(int(frame[-3:-1]))
        
        ## catch no faces being seen in the array
        if not confidence:
            return
        
        average_confidence = int(sum(confidence)/len(confidence))

        print(top_emotion)
        print(average_confidence)
        ## lowering this number will increase chatgpt calls
        if average_confidence >= 60:
            if top_emotion in ["fear", "sad", "surprise", "angry", "disgust"]:
                print(call(frame))
                self.alert = not self.alert
        

    def process_frames(self):
        """Primary thread for processing frames from frame queue from main.
        """
        while not self.stop_event.is_set():

            emotions = {}
            frame = self.frame_queue.get()
            
            response = self.detector.detect_emotions(frame)
            # Store the detected emotion for the current face in the emotions dictionary
            for i, passenger in enumerate(response):
                confidence = 0
                name = ""
                for emotion in passenger['emotions']:
                    if passenger['emotions'][emotion] > confidence:
                        confidence = passenger['emotions'][emotion]
                        name = emotion

                emotions[f"Passenger {i + 1}:"] = f"{name}: {int(confidence * 100)}%"

            if emotions:
                self.emotion_data.append(emotions['Passenger 1:'])
                print(emotions, "PRIMARY thread")
            

    def secondary_process_frames(self):
        """Secondary thread with queue to process frames from main.
        """
        while not self.stop_event.is_set():
            emotions = {}
            frame = self.secondary_queue.get()
            
            response = self.detector.detect_emotions(frame)
            # Store the detected emotion for the current face in the emotions dictionary
            for i, passenger in enumerate(response):
                confidence = 0
                name = ""
                for emotion in passenger['emotions']:
                    if passenger['emotions'][emotion] > confidence:
                        confidence = passenger['emotions'][emotion]
                        name = emotion

                emotions[f"Passenger {i + 1}:"] = f"{name}: {int(confidence * 100)}%"
            
            if emotions:
                self.emotion_data.append(emotions['Passenger 1:'])
                print(emotions, "SECONDARY thread")


    def main(self):
        # Start the thread that processes the frame
        thread_process = Thread(target=self.process_frames, daemon=True)
        secondary_thread_process = Thread(target=self.secondary_process_frames, daemon=True)
        thread_process.start()
        secondary_thread_process.start()

        # main loop
        times = [datetime.now().timestamp()]
        primary = True

        try:
            frame_count = 0
            while not self.alert:
                # Grab a frame
                if self.camera.grab() == sl.ERROR_CODE.SUCCESS:
                    # Retrieve the left image
                    image = sl.Mat()
                    self.camera.retrieve_image(image, sl.VIEW.LEFT)

                    # Convert to numpy array for easier handling in the test function
                    frame = image.get_data()[:, :, :3]

                    if primary:
                        if not self.frame_queue.full():
                            self.frame_queue.put(frame)
                        else:
                            _ = self.frame_queue.get()  # discard oldest frame
                            self.secondary_queue.put(frame)
                    else:
                        if not self.secondary_queue.full():
                            self.secondary_queue.put(frame)
                        else:
                            _ = self.secondary_queue.get()  # discard oldest frame
                            self.frame_queue.put(frame)
                    
                    primary = not primary

                # monitor every 30 frames (3-4 seconds)
                if frame_count >= 30:
                    frame_count = 0
                    #self.monitor(frame)
                else:
                    frame_count += 1

                time.sleep(self.frame_delay)

        except KeyboardInterrupt:
            ## catch ctrl-c to close threads
            pass

        times.append(datetime.now().timestamp())

        # Notification thread ends
        self.stop_event.set()

        # Wait for thread to end
        thread_process.join()
        secondary_thread_process.join()

        # Clean and save data
        cv2.destroyAllWindows()
        self.camera.close()


if __name__ == '__main__':
    EmotionRecognitionObject = EmotionRecognition()
    EmotionRecognitionObject.main()
