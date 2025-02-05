import rclpy 
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import cv2
import numpy as np

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.get_logger().info('Camera node up!')

        self.bridge = CvBridge()

        # Subscribers
        self.image_sub = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.image_callback, 10)
        self.depth_sub = self.create_subscription(
            Image, '/camera/camera/depth/image_raw', self.depth_callback, 10)

        # Publishers
        self.image_pub = self.create_publisher(Image, '/camera/processed_image', 10)
        self.position_pub = self.create_publisher(PointStamped, '/camera/object_position', 10)

        self.depth_image = None

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def image_callback(self, msg):
        self.get_logger().info('Image received')
        
        #  ROS image to OpenCV
        image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        
        # HSV color space
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # blue color range
        lower_blue = np.array([100, 150, 50])
        upper_blue = np.array([140, 255, 255])
        
        # Create a mask for blue objects
        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Get image dimensions
        height, width = image.shape[:2]
        
        # Process detected objects
        for contour in contours:
            if cv2.contourArea(contour) > 500:  # Filter small areas
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                
                # Compute the center of the detected object
                u, v = x + w // 2, y + h // 2
                
                # Ensure coordinates are within bounds
                if self.depth_image is not None and 0 <= u < width and 0 <= v < height:
                    Z = self.depth_image[v, u]
                    if not np.isnan(Z) and Z > 0.1:  # Ensure valid depth
                        # Camera intrinsic parameters (example values, should be replaced with real ones)
                        f_x, f_y = 525.0, 525.0  # Focal lengths in pixels
                        c_x, c_y = 319.5, 239.5  # Principal points
                        
                        # Compute real-world coordinates
                        X = (u - c_x) * Z / f_x
                        Y = (v - c_y) * Z / f_y
                        
                        # Publish object position
                        point_msg = PointStamped()
                        point_msg.header.stamp = self.get_clock().now().to_msg()
                        point_msg.header.frame_id = 'camera_link'
                        point_msg.point.x = X
                        point_msg.point.y = Y
                        point_msg.point.z = Z
                        self.position_pub.publish(point_msg)
                        self.get_logger().info(f'Object Position: X={X:.2f}, Y={Y:.2f}, Z={Z:.2f}')
        
        # Convert the processed image back to ROS format
        processed_msg = self.bridge.cv2_to_imgmsg(image, 'bgr8')
        self.image_pub.publish(processed_msg)
        

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
