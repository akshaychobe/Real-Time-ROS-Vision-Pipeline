import rclpy, numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from cv_bridge import CvBridge

class DepthToCloud(Node):
    def __init__(self):
        super().__init__('pointcloud_node')
        self.bridge = CvBridge()
        self.sub_depth = self.create_subscription(Image,'/stereo/depth',self.depth_cb,10)
        self.pub_cloud = self.create_publisher(PointCloud2,'/stereo/points_raw',10)
        self.fx=700.0; self.fy=700.0; self.cx=320.0; self.cy=240.0

    def depth_cb(self, msg):
        depth = self.bridge.imgmsg_to_cv2(msg,'32FC1')
        h,w = depth.shape
        xs,ys = np.meshgrid(np.arange(w), np.arange(h))
        Z = depth
        valid = np.isfinite(Z)
        X = (xs-self.cx)*Z/self.fx
        Y = (ys-self.cy)*Z/self.fy
        pts = np.stack([X[valid],Y[valid],Z[valid]],axis=-1).astype(np.float32)
        header = Header(); header.stamp = self.get_clock().now().to_msg(); header.frame_id='stereo_optical_frame'
        cloud = point_cloud2.create_cloud_xyz32(header, pts.tolist())
        self.pub_cloud.publish(cloud)

def main():
    rclpy.init(); rclpy.spin(DepthToCloud()); rclpy.shutdown()
if __name__ == '__main__': main()
