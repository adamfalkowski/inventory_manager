import rclpy # allows you to used the ROS2 functionlity in this script
from rclpy.node import Node
from std_msgs.msg import String

# Inherits the Node Class
class BasicSubscriber(Node):
    def __init__(self):
        super().__init__("basic_subscriber")
        self.subscriber_ = self.create_subscription(String, "channel01", self.msgCallback, 10)

    def msgCallback(self, msg):
        self.get_logger().info(f"I heard { msg.data} ")

def main():
    rclpy.init()
    basic_subscriber = BasicSubscriber()
    # Keeps the node active and continiously processing incoming events
    rclpy.spin(basic_subscriber)
    # If you press CTRL+C then you will stop the spin and then you can destroy the node
    basic_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()