import rclpy, cv2, numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class DepthSGBM(Node):
    def __init__(self):
        super().__init__('depth_sgbm_node')
        self.bridge = CvBridge()
        self.sub_l = self.create_subscription(Image,'/stereo/left_rect', self.left_cb,10)
        self.sub_r = self.create_subscription(Image,'/stereo/right_rect',self.right_cb,10)
        self.pub_disp  = self.create_publisher(Image,'/stereo/disparity',10)
        self.pub_depth = self.create_publisher(Image,'/stereo/depth',10)
        self.left = self.right = None
        self.matcher = cv2.StereoSGBM_create(minDisparity=0,numDisparities=128,blockSize=5,
                                             P1=8*3*5**2,P2=32*3*5**2,
                                             mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)
        self.fx = 700.0; self.baseline_m = 0.12

    def left_cb(self, m):
        self.left = cv2.cvtColor(self.bridge.imgmsg_to_cv2(m,'bgr8'), cv2.COLOR_BGR2GRAY); self.compute()
    def right_cb(self, m):
        self.right = cv2.cvtColor(self.bridge.imgmsg_to_cv2(m,'bgr8'), cv2.COLOR_BGR2GRAY); self.compute()

    def compute(self):
        if self.left is None or self.right is None: return
        disp = self.matcher.compute(self.left,self.right).astype(np.float32)/16.0
        disp[disp<=0] = np.nan
        depth = (self.fx*self.baseline_m)/disp
        vis = np.nan_to_num(disp, nan=0.0)
        vis = (vis/np.nanmax(vis)*255).astype('uint8')
        self.pub_disp.publish(self.bridge.cv2_to_imgmsg(vis,'mono8'))
        self.pub_depth.publish(self.bridge.cv2_to_imgmsg(depth.astype(np.float32),'32FC1'))

def main():
    rclpy.init(); rclpy.spin(DepthSGBM()); rclpy.shutdown()
if __name__ == '__main__': main()
