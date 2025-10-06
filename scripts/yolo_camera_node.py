#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

try:
    from ultralytics import YOLO
    ULTRALYTICS_AVAILABLE = True
except ImportError:
    ULTRALYTICS_AVAILABLE = False


class YoloCameraNode(Node):
    def __init__(self):
        super().__init__('yolo_camera_node')
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Initialize YOLO models if available
        if ULTRALYTICS_AVAILABLE:
            try:
                self.detection_model = YOLO("yolo11n.pt")
                self.segmentation_model = YOLO("yolo11n-seg.pt")
                self.get_logger().info("YOLO models loaded successfully")
            except Exception as e:
                self.get_logger().warn(f"Failed to load YOLO models: {e}")
                self.detection_model = None
                self.segmentation_model = None
        else:
            self.get_logger().warn("Ultralytics not available. Install with: pip install ultralytics")
            self.detection_model = None
            self.segmentation_model = None
        
        # Publishers
        self.det_image_pub = self.create_publisher(
            Image, 
            '/ultralytics/detection/image', 
            10
        )
        self.seg_image_pub = self.create_publisher(
            Image, 
            '/ultralytics/segmentation/image', 
            10
        )
        
        # Subscriber
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10
        )
        
        self.get_logger().info("YOLO Camera Node started")

    def image_callback(self, msg):
        """Callback function to process image and publish annotated images."""
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Process detection if model is available and there are subscribers
            if (self.detection_model is not None and 
                self.det_image_pub.get_subscription_count() > 0):
                
                det_result = self.detection_model(cv_image)
                det_annotated = det_result[0].plot(show=False)
                
                # Convert back to ROS Image and publish
                det_image_msg = self.bridge.cv2_to_imgmsg(det_annotated, encoding='bgr8')
                det_image_msg.header = msg.header  # Preserve timestamp
                self.det_image_pub.publish(det_image_msg)
            
            # Process segmentation if model is available and there are subscribers
            if (self.segmentation_model is not None and 
                self.seg_image_pub.get_subscription_count() > 0):
                
                seg_result = self.segmentation_model(cv_image)
                seg_annotated = seg_result[0].plot(show=False)
                
                # Convert back to ROS Image and publish
                seg_image_msg = self.bridge.cv2_to_imgmsg(seg_annotated, encoding='bgr8')
                seg_image_msg.header = msg.header  # Preserve timestamp
                self.seg_image_pub.publish(seg_image_msg)
                
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")


def main(args=None):
    rclpy.init(args=args)
    
    node = YoloCameraNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()