import sys
import cv2
import rclpy
import json
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo

class OpenCVCamera(Node):

    def __init__(self, rostopic='/mycamera/image_raw', infotopic = '/mycamera/camera_info', calibration='calib.json'):
        super().__init__('opencv_camera_calibrated')

        try:
            self._camera =  cv2.VideoCapture(0, cv2.CAP_V4L2)
        except Exception as e:
            self.get_logger().error(f'Unable to open camera 0')
            sys.exit(1)

        try:
            with open(calibration, 'r') as f:
                self._calibration = json.load(f)
        except Exception as e:
            self.get_logger().error(f'Unable to open calibration file {calibration} {e}')
            sys.exit(1)

        self._bridge = CvBridge()
        self._image_publisher = self.create_publisher(Image, rostopic, 1)
        self._info_publisher = self.create_publisher(CameraInfo, infotopic, 1)

    def stream(self):
        images = 0
        info_rate = 10
        ds = self._calibration['D'][0]
        ks = self._calibration['K'][0] + self._calibration['K'][1] + self._calibration['K'][2]
        try:
            while True:
                _, frame = self._camera.read()
                image_message = self._bridge.cv2_to_imgmsg(frame, "bgr8")

                self._image_publisher.publish(image_message)

                rclpy.spin_once(self, timeout_sec=0)

                if (images % info_rate) == 0:
                    camerainfo_msg = CameraInfo()
                    camerainfo_msg.distortion_model = "plumb_bob"
                    camerainfo_msg.height = frame.shape[0]
                    camerainfo_msg.width = frame.shape[1] 
                    camerainfo_msg.d = ds
                    camerainfo_msg.k = ks
                    self._info_publisher.publish(camerainfo_msg)
                images = images + 1

         
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
