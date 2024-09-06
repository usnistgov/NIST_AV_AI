import rclpy
from rclpy.node import Node
import cv2
import os
from std_msgs.msg import Float64

class SpeedPublisher(Node):
    '''
        This class emulates vehicle speed
    '''
    def __init__(self):
        super().__init__("speed_publisher")

        self.topic = 'vehicle_speed/cmd_vel'
        self.speed_pub = self.create_publisher(Float64, self.topic, 10)
        self.speed = 0.0
        self.timer = self.create_timer(0.5, self.publish_speed) # mimic 20Hz
        
    def publish_speed(self):        
        msg = Float64()
        # Resets speed after reaching 30
        if self.speed <= 30.0:
            self.speed += 1.0      
        else:
            self.speed = 0.0

        msg.data = self.speed  
        self.speed_pub.publish(msg)  
        self.get_logger().info(f'Publishing vehicle speed: {msg.data}')
   


def main(args=None):
    rclpy.init(args=args)
    speed_publisher = SpeedPublisher()
    rclpy.spin(speed_publisher)
    speed_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
