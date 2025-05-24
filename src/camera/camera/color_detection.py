import rclpy, yaml, os, numpy as np, cv2 as cv
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ament_index_python import get_package_share_directory
from models.color_enum import DetectionColor
from message_interfaces.msg import ItemColor
from models.item_color_data import ColorData



class ColorDetection(Node):
    def __init__(self):
        super().__init__("color_detection")

        # All parameters used by the node, along with their defaul values.  
        # General params are loaded via ROS2 Parameters, Color Specific params are loaded via pyyaml    
        self.declare_parameter('num_colors', 3)
        self.declare_parameter('show_mask', False)

        color_detection_colors_file_path = os.path.join(
            get_package_share_directory('camera'),
            'config',
            'color_detection_colors.yaml'
            )
        
        with open(color_detection_colors_file_path) as file:
            try:
                self.color_detection_params = yaml.safe_load(file)
                # Add ROS2 Parameter to dictionary
                self.color_detection_params['num_colors'] = self.get_parameter('num_colors').value
                self.color_detection_params['show_mask'] = self.get_parameter('show_mask').value
                self.get_logger().info(f"color detection colors parameters successfully loaded! {self.color_detection_params}")
            except:
                self.get_logger().error(f"Error Parsing the parameter files!")

        # Subscribe to the raw image topic
        self.subscriber = self.create_subscription(Image, 'raw_image', self.subscriber_callback, 10)
        self.bridge = CvBridge()
        self.retrieved_image = np.zeros((200, 300), dtype=np.uint8)

        # Publish the color detected
        self.publisher = self.create_publisher(ItemColor, 'item/color', 10)
        self.publishing_frequency = 10
        self.timer = self.create_timer((1 / self.publishing_frequency),self.publish_item)

        # Array will hold an array for each color detected.
        self.color_group = []

        

    def subscriber_callback(self, img):
        
        try:
            self.retrieved_image = self.bridge.imgmsg_to_cv2(img, desired_encoding='bgr8')
            
        except Exception as e:
            self.get_logger().error(f"Issues retrieving the image: {e}")
            return
        
        # Convert to HSV
        self.hsv_retrieved_image = cv.cvtColor(self.retrieved_image, cv.COLOR_BGR2HSV)

        for color, color_params in self.color_detection_params['colors'].items(): # Use the items() method used when you want to loop through the Key and Value
        
            # Get color bounds
            lower_limit, upper_limit  = self.findColor(color, color_params['color_id'], color_params['hsv'], color_params['variance'])

            # Apply a mask
            mask = None
            for lower_limit, upper_limit in zip(lower_limit, upper_limit):
                temp_mask = cv.inRange(self.hsv_retrieved_image, lower_limit, upper_limit)
                mask = temp_mask if mask is None else cv.bitwise_or(mask, temp_mask)
            
            # Option to show the mask
            if self.color_detection_params['show_mask']:
                cv.imshow("mask",mask)
                cv.waitKey(100)

            self.contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
            # Contours are a python list of contours.  Each contour is a numpy array of the cordiantes of the end points of the contour (this is what chain approx simple does_)
            item_count = len( [contour for contour in self.contours if cv.contourArea(contour) > color_params['min_area']] ) 

            for contour in self.contours:
                if int(cv.contourArea(contour)) > color_params['min_area']:
                    m = cv.moments(contour)
                    cx = m["m10"] / m["m00"]
                    cy = m["m01"] / m["m00"]
                    #circle = cv.circle(self.retrieved_image, (cx,cy), 3, (0,255,0), -1)
                    #cv.imshow('circle',circle)
                    #cv.waitKey(100)
            
            self.color_group.append(ColorData(color, item_count, cx, cy))


    def findColor(self, color: str, color_id: int, hsv: list, variance: list) -> tuple:

        try:
            hue, sat, val = hsv
        except IndexError as e:
            self.get_logger().error(f"Detection index of: {e} out of bounds")
            return None
        
        hue_tolerance = int(179 * variance[0])
        sat_tolerance = int(255 * variance[1])
        val_tolerance = int(255 * variance[2])

        if color_id == DetectionColor.RED.value: # Red is special since its centered around HSV value of 0 or 179, need to wrap around if you want to apply tolerance
            lower_limit_1 = np.array([
                0, 
                max(sat - sat_tolerance, 0),
                max(val - val_tolerance, 0)
            ])
            upper_limit_1 = np.array([
                min(hue + hue_tolerance, 179),
                min(sat + sat_tolerance, 255),
                min(val + val_tolerance, 255)
            ])

            lower_limit_2 = np.array([
                max(179 - hue_tolerance, 0) ,
                max(sat - sat_tolerance, 0),
                max(val - val_tolerance, 0)
            ])

            upper_limit_2 = np.array([
                179,
                min(sat + sat_tolerance, 255),
                min(val + val_tolerance, 255)
            ])
            
            lower_limit = [lower_limit_1, lower_limit_2]
            upper_limit = [upper_limit_1, upper_limit_2]
            
        
        else:
            lower_limit = [ np.array([
                max(hue - hue_tolerance, 0),
                max(sat - sat_tolerance, 0),
                max(val - val_tolerance, 0)
                ]) ]
            upper_limit = [ np.array([
                min(hue + hue_tolerance, 179),
                min(sat + sat_tolerance, 255),
                min(val + val_tolerance, 255)
                ]) ]
        
        return [lower_limit, upper_limit]
        

    def publish_item(self):
        for item in self.color_group:
            msg = ItemColor()
            msg.color = item.color
            msg.item_count = item.item_count
            msg.cx = item.cx
            msg.cy = item.cy
            self.publisher.publish(msg)
            self.get_logger().info(f'Publishing Item on topic /item/color with frequency of {self.publishing_frequency} hertz')


def main():
    rclpy.init()
    color_detection = ColorDetection()
    # Keeps the node active and continiously processing incoming events
    rclpy.spin(color_detection)
    cv.destroyAllWindows()
    # If you press CTRL+C then you will stop the spin and then you can destroy the node
    color_detection.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()




