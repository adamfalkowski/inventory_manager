import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from ultralytics import YOLO
from message_interfaces.msg import ItemDetection
from geometry_msgs.msg import Point

class ObjectDetection(Node):
    def __init__(self):
        super().__init__('object_detection')

        # Subscribe to the raw camera topic which publishes raw images from webcam
        # The frequency of subscription depends on how ofte the topic get a new message published.
        self.subscriber = self.create_subscription(Image, 'raw_camera', self.subscriber_callback, 10)

        # Create a publishet to publish the object detection on the item/detection topic
        self.publisher = self.create_publisher(ItemDetection, 'item/detection', 10)

        self.bridge = CvBridge()

        self.model = YOLO('yolov8n.pt')

        self.get_logger().info("Object Detection Node started")

    def subscriber_callback(self, img):

        # Convert ROS2 image into OpenCV Image
        try:
            retrieved_image = self.bridge.imgmsg_to_cv2(img, desired_encoding='bgr8')
            
        except Exception as e:
            self.get_logger().error(f"Issues retrieving the image: {e}")
            return
        
        # Calling the YOLO objects __call__() method:
        # "Running YOLO"
        results = self.model(retrieved_image, device='cpu',verbose=False)

        for result in results:
            for box, cls, conf in zip(result.boxes.xyxy, result.boxes.cls, result.boxes.conf):

                label = self.model.names[int(cls)]
                confidence = round(float(conf),2)
                x1, y1, x2, y2 = box.tolist()

                center_x = (x1 + x2) / 2
                center_y = (y1 + y2) / 2 
        
                # Publish custom item detection message
                msg = ItemDetection()
                msg.class_name = label
                msg.confidence = confidence
                msg.center = Point()
                msg.center.x = center_x
                msg.center.y = center_y
                self.publisher.publish(msg)
                self.get_logger().info(f"Publishing to item/detections topic {msg}")

def main():
    rclpy.init()
    object_detection = ObjectDetection()
    rclpy.spin(object_detection)
    object_detection.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()












