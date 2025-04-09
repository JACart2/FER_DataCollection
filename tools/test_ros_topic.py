import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ZedImageSubscriber(Node):

    def __init__(self):
        super().__init__('zed_left_image_subscriber')
        
        # Create a CvBridge object
        self.bridge = CvBridge()

        print('creating sub')
        # Create a subscription to the left image topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/zed2i/left/image_raw',  # Topic name for ZED 2i left image
            self.listener_callback,
            10
        )
        print(self.subscription, 'sub obj')
        print(dir(self.subscription), 'options')
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('Received an image')

        try:
            # Convert ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Save the image to disk
            cv2.imwrite("/tmp/zed_left_image.jpg", cv_image)
            self.get_logger().info('Image saved to /tmp/zed_left_image.jpg')

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')


def main(args=None):
    rclpy.init(args=args)
    zed_image_subscriber = ZedImageSubscriber()
    rclpy.spin(zed_image_subscriber)
    
    zed_image_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
