"""Emotion Recognition of input video feed using multi-threading.

Author: John Rosario Cruz
Based off "Facial-Recognition" by: Donghao Liu
Based off "FaceRec_Emotion" by: Dominic Nguyen
Version: 4/24/2025
"""
## ROS2 packages
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

## Multi-Threading
from threading import Thread
import threading
from queue import Queue

## Facial Expression Recognition (see README)
from fer import FER

## Utility
import copy
import re
import time

## Stubbed integrations for cart action
#from openai_call import call_openai


class EmotionRecognition(Node):
    """
    Description
    ------------
        Script for monitoring passenger emotions through a ZED camera. Also includes 
        trigger methods for causing actions in the cart if needed.

    Attributes
    ----------
        bridge (obj): Used for converting ROS2 Image objects to an image matrix.

        subscription (obj): The subscription to the ROS2 topic for images. 
            Triggers listern_callback() on image receipt.

        frame_delay (float): The OPTIONAL time in seconds that 
            the main loop will wait to keep time with ZED FPS.

        primary (boolean): Determines which queue to use for processing. 
            This alternates between which queue/thread is given image data.

        emotion_data (list): Stores emotion data for frames.

        detector (obj): The FER detector. Used to detect emotion & faces in frames.

        frame_queue (queue): The first queue of frames coming from the ZED camera.

        secondary_queue (queue): The second queue of frames coming from the ZED camera.

        stop_event (boolean): Forces the threads to begin termination.
            Forces listener_callback() to stop processing.

        thread_process (thread): The primary thread for processing. This method
            uses process_frames().

        secondary_thread_process (thread): The secondary thread for processing.
            This method uses secondary_process_frames().

    Methods:
    -------
        listener_callback()
            Triggered on image receipt from ROS2 topic. Populates thread queues and periodically
            calls monitor() based on self.emotion_data size.

        trigger_stop()
            Stubbed out integration with ROS2 publish to stop the cart (or slowdown?)

        trigger_admin_alert()
            Trigger to HCI/Backend for alerting the admin to check on a passenger/cart.

        trigger_user_prompt()
            Trigger to HCI/Backend for prompting the user if they are okay.

        monitor()
            Analyze and clear all emotions in emotion_data to determine if openai call is required.

        process_frames()
            Primary thread for processing frames from frame queue from main.

        secondary_process_frames()
            Secondary thread with queue to process frames from main.
    """

    def __init__(self):
        ## configure camera setup
        super().__init__('emotion_recognition_node')
        self.bridge = CvBridge()

        # subscription
        self.subscription = self.create_subscription(
            Image,
            '/zed_rear/zed_node/left_raw/image_raw_color', 
            self.listener_callback,
            10
        )

        # optional frame throttling
        fps = 15
        self.frame_delay = float(1 / fps)
        self.current_time = 0.0

        # processing vars
        self.primary = True

        # sentiment labels
        self.emotion_data = []
        self.detector = FER(mtcnn=True)

        # threading init
        self.frame_queue = Queue(maxsize=15)
        self.secondary_queue = Queue(maxsize=15)
        self.stop_event = threading.Event()

        # starting threads
        self.thread_process = Thread(target=self.process_frames, daemon=True)
        self.secondary_thread_process = Thread(target=self.secondary_process_frames, daemon=True)
        self.thread_process.start()
        self.secondary_thread_process.start()

    def listener_callback(self, msg):
        """Callback method that handles the incoming images from the ROS2 topic.

        Args:
            msg (obj): Image object from ROS2 topic.
        """
        # prevent unnecessary processing if ending script
        print('received image!')

        # force image throttle
        current_time = time.time()
        if current_time - self.current_time < self.frame_delay:
            return
        else:
            self.current_time = current_time

        if not self.stop_event.is_set():
            try:
                rgba_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
                rgb_image = cv2.cvtColor(rgba_image, cv2.COLOR_BGRA2BGR)

                ## manual image flip until ROS2 topic does this
                rgb_image = cv2.flip(rgb_image, 0)

                ## alternating between threads
                if self.primary:
                    if not self.frame_queue.full():
                        self.frame_queue.put(rgb_image)
                    else:
                        self.secondary_queue.put(rgb_image)
                else:
                    if not self.secondary_queue.full():
                        self.secondary_queue.put(rgb_image)
                    else:
                        self.frame_queue.put(rgb_image)

                self.primary = not self.primary

                # monitor occassionally
                if len(self.emotion_data) >= 30:
                    self.monitor(rgb_image)

            except Exception as e:
                self.get_logger().error(f"Error processing image: {e}")

    def trigger_stop(self):
        """Stubbed out integration with ROS2 publish to stop the cart (or slowdown?).
        """
        print('Stop-Process: Called')

    def trigger_admin_alert(self):
        """Trigger to HCI/Backend for alerting the admin to check on a passenger/cart.
        """
        print('Admin-Alert-Process: Called')

    def trigger_user_prompt(self):
        """Trigger to HCI/Backend for prompting the user if they are okay.
        """
        print('User-Prompt-Process: Called')

    def monitor(self, frame):
        """Monitor the events coming from the threads.

        Args:
            frame (np matrix): The frame tensor representing the image.
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

        print('')
        print(top_emotion, 'top_emotion')

        ## find average confidence of each frame of the top emotion
        confidence = []
        for emotion in emotion_copy:
            if top_emotion in emotion:
                confidence.append(int(emotion[-3:-1]))

        ## catch no faces being seen in the array
        if not confidence:
            print('no faces were seen in the frame?, l-164')
            return

        average_confidence = int(sum(confidence)/len(confidence))

        print(top_emotion)
        print(average_confidence)
        ## lowering this number will increase chatgpt calls
        if average_confidence >= 65:
            if top_emotion in ["fear", "sad", "surprise", "angry", "disgust"]:
                #response = call_openai(frame)
                ## remove this below once openai is integrated
                response = 'U'
                self.trigger_user_prompt()
                if response in ['U', 'I']:
                    self.trigger_stop()
                    self.trigger_admin_alert()

                ## optional kill switch for script V
                #self.alert = not self.alert

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


def main():
    """Main callpoint of the class.
    """
    ## waiting for backend service to spin up their topic
    time.sleep(20)
    rclpy.init()
    emotion_node = EmotionRecognition()

    try:
        rclpy.spin(emotion_node)
    except KeyboardInterrupt:
        pass
    finally:
        ## ending threads, ending listener_callback() functionality
        emotion_node.stop_event.set()
        emotion_node.thread_process.join()
        emotion_node.secondary_thread_process.join()
        emotion_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
