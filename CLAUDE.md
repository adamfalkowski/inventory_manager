# Project
- ROS2 Jazzy workspace for an inventory manager.  
- Small movable robot that scans a specific space with a small camera and using object detction with YOLO is able to know how everything is organized.  A future feature of incorporating Amazon Alexa is available.
- Packages live under src/ — each has package.xml, CMakeLists.txt or setup.py.
- Custom messages defined in the message_interfaces package

# Conventions
- C++ nodes use rclcpp, Python nodes use rclpy — don't mix in one package
- Custom messages defined in the message_interfaces package