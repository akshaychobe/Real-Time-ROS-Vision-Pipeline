import rclpy, cv2, yaml, os, numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from ament_index_python.packages import get_package_share_directory

def load_yaml(p):
    with open(p, 'r') as f:
        return yaml.safe_load(f)

class StereoRectify(Node):
    def __init__(self):
        super().__init__('stereo_rectify_node')
        self.bridge = CvBridge()

        # Subscribe to raw stereo images
        self.left_sub  = Subscriber(self, Image, '/camera/left/image_raw')
        self.right_sub = Subscriber(self, Image, '/camera/right/image_raw')
        self.ts = ApproximateTimeSynchronizer([self.left_sub, self.right_sub], queue_size=10, slop=0.02)
        self.ts.registerCallback(self.cb)

        # Load intrinsics/extrinsics from package share (installed files)
        share = get_package_share_directory('stereo_perception')
        intr_path = os.path.join(share, 'calib', 'stereo_intrinsics.yaml')
        extr_path = os.path.join(share, 'calib', 'stereo_extrinsics.yaml')
        intr = load_yaml(intr_path)
        extr = load_yaml(extr_path)

        self.K1, self.D1 = np.array(intr['K1'], dtype=np.float64), np.array(intr['D1'], dtype=np.float64)
        self.K2, self.D2 = np.array(intr['K2'], dtype=np.float64), np.array(intr['D2'], dtype=np.float64)
        self.R,  self.T  = np.array(extr['R'],  dtype=np.float64), np.array(extr['T'],  dtype=np.float64)

        # We will compute maps on the first frame using that frame's size
        self.maps_ready = False
        self.map1x = self.map1y = self.map2x = self.map2y = None

        self.pub_l = self.create_publisher(Image, '/stereo/left_rect', 10)
        self.pub_r = self.create_publisher(Image, '/stereo/right_rect', 10)

    def _build_maps(self, image_size):
        w, h = image_size  # (width, height)
        size = (w, h)
        R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(
            self.K1, self.D1, self.K2, self.D2, size, self.R, self.T, alpha=0)
        self.map1x, self.map1y = cv2.initUndistortRectifyMap(self.K1, self.D1, R1, P1, size, cv2.CV_32FC1)
        self.map2x, self.map2y = cv2.initUndistortRectifyMap(self.K2, self.D2, R2, P2, size, cv2.CV_32FC1)
        self.maps_ready = True
        self.get_logger().info(f"Rectification maps built for size {size}")

    def cb(self, lmsg, rmsg):
        l = self.bridge.imgmsg_to_cv2(lmsg, 'bgr8')
        r = self.bridge.imgmsg_to_cv2(rmsg, 'bgr8')
        h, w = l.shape[:2]
        if not self.maps_ready:
            self._build_maps((w, h))
        lrect = cv2.remap(l, self.map1x, self.map1y, cv2.INTER_LINEAR)
        rrect = cv2.remap(r, self.map2x, self.map2y, cv2.INTER_LINEAR)
        self.pub_l.publish(self.bridge.cv2_to_imgmsg(lrect, 'bgr8'))
        self.pub_r.publish(self.bridge.cv2_to_imgmsg(rrect, 'bgr8'))

def main():
    rclpy.init()
    rclpy.spin(StereoRectify())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
