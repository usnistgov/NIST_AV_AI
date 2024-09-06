import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String

from std_msgs.msg import Float64

class SpeedSubscriber(Node):

    def __init__(self, name):
        super().__init__(name)
        
        self.topic = 'vehicle_speed/cmd_vel'
        self.queue = 10
        
        # subscribe to the camera sensor stream
        self.sub = self.create_subscription(
            Float64,
            self.topic,
            self.speed_logger_callback,
            self.queue
        )
        self.get_logger().info('****speed subscriber****')

    def speed_logger_callback(self, msg): 
        """
        Callback function for the subscriber to the vehicle's speed.
        
        """
        self.get_logger().info(f'Recieved Speed: {msg.data} m/s')

    
       
 
def main(args=None):
    rclpy.init(args=args) 
    node = SpeedSubscriber("speed_logger")
    rclpy.spin(node)
   
    node.destroy_node()
    rclpy.shutdown()
  
if __name__ == '__main__':
    main()  