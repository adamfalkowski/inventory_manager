import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv

# Publishes live image from Webcam to the 'raw_camera' channel
class CameraPublisher(Node):
    def __init__(self):
        super().__init__("camera_publisher")
        self.publisher_ = self.create_publisher(Image, 'raw_camera', 10)
        self.publishing_frequency_ = 30
        self.timer_ = self.create_timer((1 / self.publishing_frequency_),self.timer_callback)
        self.camera_index = 0

        try:
            self.capture_ = cv.VideoCapture(self.camera_index)
            if not self.capture_.isOpened():
                raise ValueError(f"Wrong or incorrect camera index!  Index: {self.camera_index}  - either does not exist or is incorrect!")
        except ValueError as e:
            print("Camera Index Issue:", e)
        self.bridge_ = CvBridge()

    def timer_callback(self):
        (isRead, frame) = self.capture_.read()
        if isRead:
            # Convert a openCV Image to a ROS2 image message (from sensor_msg.msg) so that it can be published on the topic
            # You need to use the cv bridge to convert this.  Returns an image message object
            # BRG is the default for CV Images (blue, red, green), 8 bits, so a max of 2^8 = 255
            img_msg = self.bridge_.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher_.publish(img_msg)
            self.get_logger().info(f'Publishing Image on channel /raw_camera with frequency of {self.publishing_frequency_} hertz')
        else:
            self.get_logger().error('Failed to grab frame!')

def main():
    rclpy.init()
    camera_publisher = CameraPublisher()
    rclpy.spin(camera_publisher)
    camera_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

