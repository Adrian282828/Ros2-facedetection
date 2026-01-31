import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import os
import numpy as np

class ImageProcessor(Node):
    def __init__(self):
        super().__init__('my_node')
        model_path = os.path.expanduser('~/haarcascade_frontalface_default.xml')
        self.faceCascade = cv2.CascadeClassifier(model_path)
        self.bridge = CvBridge()
        
        # Subscriber: Get raw image
        self.subscription = self.create_subscription(
            Image, '/image_raw', self.listener_callback, 10)
            
        # Publisher: Send compressed image back to ROS
        self.publisher = self.create_publisher(CompressedImage, '/image_with_faces/compressed', 10)
        self.get_logger().info("Face Detection Node Started")

    def listener_callback(self, data):
        imCV = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        gray = cv2.cvtColor(imCV, cv2.COLOR_BGR2GRAY)
        faces = self.faceCascade.detectMultiScale(gray, 1.1, 5)
        
        for (x, y, w, h) in faces:
            cv2.rectangle(imCV, (x, y), (x+w, y+h), (0, 255, 0), 2)
            
        # Compress to JPEG for WSL(no lagging)  (i tried to use the nomral image but my computer kept freezing)
        _, buffer = cv2.imencode('.jpg', imCV)
        msg = CompressedImage()
        msg.header = data.header
        msg.format = "jpeg"
        msg.data = np.array(buffer).tobytes()
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImageProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
