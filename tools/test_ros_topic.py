import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ZedImageSubscriber(Node):
    def __init__(self):
        super().__init__('zed_image_subscriber')

        # Initialize CvBridge to convert ROS messages to OpenCV images
        self.bridge = CvBridge()

        # Create a subscription to the ZED camera left image topic
        self.subscription = self.create_subscription(
            Image,  # ROS message type
            '/zed/left/image_rect_color',  # Topic name (ZED camera's left image topic)
            self.image_callback,  # Callback function
            10  # QoS (Quality of Service) profile - set to 10 messages in the buffer
        )

        # Log that the subscription is active
        self.get_logger().info("Subscribed to ZED camera left image topic.")

    def image_callback(self, msg):
        try:
            # Convert the ROS 2 image message to an OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

            # Display the image using OpenCV
            cv2.imshow("ZED Camera Left Image", cv_image)
            cv2.waitKey(1)  # Wait for a key press to update the window

        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")

def main(args=None):
    # Initialize ROS 2 client library
    rclpy.init(args=args)

    # Create an instance of the ZedImageSubscriber node
    node = ZedImageSubscriber()

    # Spin the node to keep it alive and process callbacks (image messages)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up and shut down the node
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
