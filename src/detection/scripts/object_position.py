import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import Int32MultiArray

'''ros2 launch realsense2_camera rs_launch.py enable_rgbd:=true enable_sync:=true align_depth.enable:=true enable_color:=true enable_depth:=true 
'''
# class declaration

class ObjectPosition(Node):
    def __init__(self):
        super().__init__('object_position')
        self.get_logger().info('object_position node up!')

        self.bridge = CvBridge()
        self.depth_image = None  # Inicializa la imagen de profundidad para evitar errores antes de recibir datos

        # subscribers
        self.depth_sub = self.create_subscription(
            Image, '/camera/camera/aligned_depth_to_color/image_raw', self.depth_callback, 10)
        
        self.position_sub = self.create_subscription(
            Int32MultiArray, '/object_position_image', self.position_callback, 10)

        # publishers
        self.position_pub = self.create_publisher(PointStamped, '/object_position', 10)

    # recover depth image
    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    # recover position in image
    def position_callback(self, msg):
        if self.depth_image is None:
            return

        if len(msg.data) < 2:
            return
        
        u, v = msg.data # position in pixels
        height, width = self.depth_image.shape[:2]

        # check if the values are in the range of thr depth
        if not (0 <= u < width and 0 <= v < height):
            self.get_logger().warn(f'Invalid position ({u}, {v}) - Image size: {width}x{height}')
            return
    
        # find the depth  in meters
        Z = float(self.depth_image[v, u]) /1000 # from mm to meters
        
        if Z <= 0.0:
            return


        # intrinsic parameters of the camera
        # f_x and f_y are the focal lengths expressed in pixels
        # c_x and c_y are the optical centers expressed in pixels
        f_x = 391.7060546875
        f_y = 391.7060546875
        c_x = 320.6274719238281
        c_y = 242.14434814453125

        # convert to coordinates 
        X = ((u - c_x) * Z / f_x)
        Y = ((v - c_y) * Z / f_y)
        self.get_logger().info(f'X: {X}')
        self.get_logger().info(f'Y: {Y}')
        self.get_logger().info(f'Z: {Z}')

        point_msg = PointStamped()
        point_msg.header.frame_id = "camera_link"
        point_msg.point.x = float(X)
        point_msg.point.y = float(Y)
        point_msg.point.z = float(Z)
        self.position_pub.publish(point_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectPosition()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
