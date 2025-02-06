import rclpy 
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import Int32MultiArray




# camera class

class DetectObject(Node):
    def __init__(self):
        super().__init__('detect_object')
        self.get_logger().info('detect_object node up!')

        self.bridge = CvBridge()

        # subscribers
        self.image_sub = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.image_callback, 10)


        # publishers

        self.image_pub = self.create_publisher(Image, '/camera/processed_image', 10)
        self.position_pub = self.create_publisher(Int32MultiArray, '/object_position_image', 10)
    # recover RGB image
    def image_callback(self, msg):  
        self.image_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.track_object()

    def track_object(self, msg=None):
        # convert image to hsv
        hsv = cv2.cvtColor(self.image_image, cv2.COLOR_BGR2HSV)

        # detect the color blue
        lower_blue = np.array([100, 150, 50])
        upper_blue = np.array([140, 255, 255])

        # create a mask
        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        #find the contours of the image
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # dimensions fo the image
        height, width = self.image_image.shape[:2]
        u, v = 0, 0
        # create a box on the blue object
         
        # process the contours
        for contour in contours:
            if cv2.contourArea(contour) > 1000: # filter small areas (eliminate noise) bigger than 500 pixels
                x, y, w, h = cv2.boundingRect(contour) # get the coordinates of the rectangle
                cv2.rectangle(self.image_image, (x, y), (x + w, y + h), (0, 255, 0), 2) # print the rectangle
                # center of the object
                u, v = x + w // 2, y + h // 2  # find the center of the rectangle
                self.get_logger().info(f'(pos object in the image (px)) {u}, {v}') # print the center of the object
        
        # publish image
        new_image = self.bridge.cv2_to_imgmsg(self.image_image, encoding='bgr8')
        self.image_pub.publish(new_image)

        # publish position
        msg = Int32MultiArray()
        msg.data = [u, v] # coordenates of the object (in pixels)
        self.position_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DetectObject()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
