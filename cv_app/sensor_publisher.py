import rclpy
from rclpy.node import Node
import cv2
import os
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from cv_bridge import CvBridge, CvBridgeError
import ros2_numpy

class ImagePublisher(Node):
    def __init__(self,folder_path):
        super().__init__("image_publisher")

        self.topic = 'arena_camera_node/images'
        self.queue = 10

        self.publisher = self.create_publisher(Image, self.topic, self.queue)
        self.bridge = CvBridge()
        self.folder_path = folder_path
        self.index = 0
        self.files = [f for f in os.listdir(folder_path) if f.endswith('.png')]
        if not self.files:
            self.get_logger().error('No PNG images found in the specified folder.')
            return
        self.files.sort()
        self.timer = self.create_timer(0.05, self.timer_callback) # mimic 20Hz
        self.get_logger().info('Publishing...')
        

    def timer_callback(self):        
        image_path = os.path.join(self.folder_path, self.files[self.index])
        frame = cv2.imread(image_path)
        if frame is not None:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher.publish(msg)
            
            self.get_logger().info(f'Published image: {self.files[self.index]}') 
        else:
            self.get_logger().info('No more images to publish.')
        
        self.index += 1
        if self.index >= len(self.files):
            self.index = 0  


def main(args=None):
    rclpy.init(args=args)
    folder_path = './src/cv_app/cv_app/rgb_exp1_graffiti/rgb'
    ip = ImagePublisher(folder_path)
    if ip.files:
        rclpy.spin(ip)
    

    ip.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
