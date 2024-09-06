from cv_app.cv_head.cv_head import UltralyticsDetector
import rclpy
from rclpy.node import Node
from rclpy.qos import SensorDataQoS
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import cv2

class ImageSubscriber(Node):

    def __init__(self, name):
        super().__init__(name)
        
        # create two mutually exclusive callback groups
        # one for the subscriber and one for the computer vision application
        self.group1 = MutuallyExclusiveCallbackGroup()
        self.group2 = MutuallyExclusiveCallbackGroup()

        qos_profile = SensorDataQoS()
        
        self.current_frame = None
        
        self.topic = 'arena_camera_node/images'
        self.queue = 10
        
        # subscribe to the camera sensor stream
        self.sub = self.create_subscription(
            Image,
            self.topic,
            self.camera_image_subscriber,
            qos_profile,
            self.queue,
            callback_group=self.group1
        )
        
        # timer to execute the cv detector every 0.1 seconds
        self.timer = self.create_timer(0.05, 
                                       self.client_cv_application, 
                                       callback_group=self.group2)

        self.get_logger().info('****subscriber****')

        # initialize cv bridge
        self.bridge = CvBridge()

        # initialize the computer vision class
        self.cv_head = UltralyticsDetector(
            './src/cv_app/cv_app/autonomous-driving',
            save_img=True, 
            save_prediction=True
        )

        # initialize publisher for annotated image
        self.annotated_publisher = self.create_publisher(Image, '/annotated_output/Images', 10)

        # initialize publisher for stop command
        self.stopcmd_publisher = self.create_publisher(String, '/stopsign_detector/cmd_stop', 10)

        # counter to keep track of files 
        self.counter = 0
    
    def publish_on_vehicle(self, annotated_image, stop_cmd):
        """Function to publish data on the vehicle for motion control(only braking at this point).
        
        Performs the following tasks:
        1. Publishes annotated image data after classification on topic 'annotated_output/Images'
        2. Publishes stop command using logic in cv_head.detector on topic '/stopsign_detector/cmd_stop'

        """
        pass


    def client_cv_application(self):
        """Callback function to execute the computer vision application.
        
        Performs the following tasks:
        1. Gets the current frame from the camera sensor stream.
        2. Performs computer vision tasks.
        3. Increments the counter after processing the image.
        """
        # get the current frame
        # perform computer vision tasks
        if self.current_frame is None:
            return
        #self.current_frame = self.bridge.imgmsg_to_cv2(data, 'bgr8')

        # annotated image and stop parameter
        _image, _stop = self.cv_head.detector(self.current_frame, "img" + str(self.counter))   
        #print(self.counter, _image.shape, _stop)
        
        # increment counter after processing the image
        self.counter += 1

        # Stop command
        stop_msg = String()
        stop_msg.data = _stop
        self.stopcmd_publisher.publish(stop_msg)
        self.get_logger().info('Published stop message...')

        # Annotated images
        annotated_msg = self.bridge.cv2_to_imgmsg(_image, encoding="bgr8")
        self.annotated_publisher.publish(annotated_msg)
        self.get_logger().info('Published annotated image...') 
        # call function to publish on vehicle
        #publish_on_vehicle(_image, _stop)

    def camera_image_subscriber(self, data): 
        """Callback function for the subscriber to the camera sensor stream.
        
        Args:
            data (Image): The image data received from the camera sensor stream.
        """
        current_frame = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        current_frame = cv2.rotate(current_frame, cv2.ROTATE_180)
        self.current_frame = current_frame[:620, :]

        
        # # perform computer vision tasks
        # self.cv_head.detector(current_frame, "img" + str(self.counter))   
        
        # # increment counter after processing the image
        # self.counter += 1
       
 
def main(args=None):
    rclpy.init(args=args) 
    node = ImageSubscriber("image_subscriber")
    
    # Use a MultiThreadedExecutor to allow callbacks in different groups to run concurrently
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
  
if __name__ == '__main__':
    main()  