import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

from fer import FER

class ZedImageSubscriber(Node):

    def __init__(self):
        super().__init__('zed_left_image_subscriber')
        
        # Create a CvBridge object
        self.bridge = CvBridge()

        # print('creating sub')
        # Create a subscription to the left image topic
        self.subscription = self.create_subscription(
            Image,
            '/zed/zed_node/left_raw/image_raw_color',  # Topic name for ZED 2i left image
            self.listener_callback,
            10
        )
        # print(self.subscription, 'sub obj')
        # print(dir(self.subscription), 'options')
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('Received an image')
        # print(dir(msg))
        # print(dir(self.bridge), 'bridge')

        # print(f"Encoding: {msg.encoding}")

        try:
            # Convert ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

            # print(f"shape: {cv_image.shape}, dtype: {cv_image.dtype}")

            no_alpha_image = cv2.cvtColor(cv_image, cv2.COLOR_BGRA2BGR)
            
            detector = FER(mtcnn=True)

            response = detector.detect_emotions(no_alpha_image)

            print(response, 'emotions')

            # cv2.imshow("image", no_alpha_image)
            # cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')


def main(args=None):
    rclpy.init(args=args)
    zed_image_subscriber = ZedImageSubscriber()
    rclpy.spin(zed_image_subscriber)
    
    zed_image_subscriber.destroy_node()
    rclpy.shutdown()

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
