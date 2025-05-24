import rclpy #allows you to use ROS fuctionalities in our script
from rclpy.node import Node 
from std_msgs.msg import String

class BasicPublisher(Node):

    def __init__(self):
        super().__init__("basic_publisher") 
        # Create a publisher within a ROS Node; 
        # allows to send a message to a given topic
        self.publisher_ = self.create_publisher(String, "channel01",10)
        self.frequency_ = 5.0
        self.get_logger().info(f"publishing at {self.frequency_} Hz")
        self.create_timer(self.frequency_,self.timerCallback)

    def timerCallback(self):
        msg = String() # Create an instance of the String method. 
        # The scope of this instance will be in this method.  
        # The instance will be destroyed once the method is done
        msg.data = f"Wazzauppp Fellas"
        self.publisher_.publish(msg)

def main():
    rclpy.init()
    basic_publisher = BasicPublisher()
    rclpy.spin(basic_publisher)
    basic_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()


