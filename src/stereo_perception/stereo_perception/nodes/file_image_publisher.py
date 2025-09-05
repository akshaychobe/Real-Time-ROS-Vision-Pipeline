import rclpy, cv2, os
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory

class FileImagePublisher(Node):
    def __init__(self):
        super().__init__('file_image_publisher')
        self.pub_l = self.create_publisher(Image, '/camera/left/image_raw', 10)
        self.pub_r = self.create_publisher(Image, '/camera/right/image_raw', 10)
        self.bridge = CvBridge()

        share = get_package_share_directory('stereo_perception')
        self.left_path  = os.path.join(share, 'data', 'left.png')
        self.right_path = os.path.join(share, 'data', 'right.png')

        self.l = cv2.imread(self.left_path)
        self.r = cv2.imread(self.right_path)
        if self.l is None or self.r is None:
            self.get_logger().error(f'left/right not found at: {self.left_path} , {self.right_path}')
            return

        self.timer = self.create_timer(0.1, self.tick)  # ~10 Hz

    def tick(self):
        self.pub_l.publish(self.bridge.cv2_to_imgmsg(self.l, 'bgr8'))
        self.pub_r.publish(self.bridge.cv2_to_imgmsg(self.r, 'bgr8'))

def main():
    rclpy.init(); rclpy.spin(FileImagePublisher()); rclpy.shutdown()

if __name__ == '__main__':
    main()
