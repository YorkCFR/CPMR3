import sys
import cv2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class OpenCVCamera(Node):

    def __init__(self, rostopic='/mycamera/image_raw'):
        super().__init__('opencv_camera')

        try:
            self._camera =  cv2.VideoCapture(0, cv2.CAP_V4L2)
        except Exception as e:
            self.get_logger().error(f'Unable to open camera 0')
            sys.exit(1)

        self._bridge = CvBridge()
        self._publisher = self.create_publisher(Image, rostopic, 1)

    def stream(self):
        try:
            while True:
                _, frame = self._camera.read()
                image_message = self._bridge.cv2_to_imgmsg(frame, "bgr8")

                self._publisher.publish(image_message)

                rclpy.spin_once(self, timeout_sec=0)
        except Exception as e:
            self.get_logger().error(f'CameraStreamNode capture error {e}')

def main(args=None):
    rclpy.init(args=args)
    node = OpenCVCamera()
    try:
        node.stream()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
