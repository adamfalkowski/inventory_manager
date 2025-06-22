import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv

# Publishes saved image to the 'raw_picture' channel
class PicturePublisher(Node):
    def __init__(self):
        super().__init__("picture_publisher")
        self.publisher_ = self.create_publisher(Image, 'raw_picture', 10)
        self.publishing_frequency_ = 30
        self.timer_ = self.create_timer((1 / self.publishing_frequency_),self.timer_callback)
        self.camera_index = 0
        self.bridge_ = CvBridge()

    def timer_callback(self):
        self.markers_picture = cv.imread('/home/adam-falkowski/Pictures/markers.jpg')
        img_msg = self.bridge_.cv2_to_imgmsg(self.markers_picture, encoding='bgr8')
        self.publisher_.publish(img_msg)
        self.get_logger().info(f'Publishing Image on channel /raw_picture with frequency of {self.publishing_frequency_} hertz')

def main():
    rclpy.init()
    picture_publisher = PicturePublisher()
    rclpy.spin(picture_publisher)
    picture_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

