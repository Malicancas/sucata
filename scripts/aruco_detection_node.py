#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped, PoseStamped
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import numpy as np
import os

try:
    from ultralytics import YOLO
    ULTRALYTICS_AVAILABLE = True
except ImportError:
    ULTRALYTICS_AVAILABLE = False


class ArUcoDetectionNode(Node):
    def __init__(self):
        super().__init__('aruco_detection_node')
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Path to the trained ArUco model
        self.aruco_model_path = "/home/ze/Documents/ros2ws/ArUco v.3.0.v1i.yolov11/train2/weights/best.pt"
        
        # Initialize ArUco YOLO model if available
        if ULTRALYTICS_AVAILABLE:
            try:
                if os.path.exists(self.aruco_model_path):
                    self.aruco_model = YOLO(self.aruco_model_path)
                    self.get_logger().info("Custom ArUco detection model loaded successfully")
                else:
                    self.aruco_model = None
                    self.get_logger().error(f"ArUco model not found at: {self.aruco_model_path}")
            except Exception as e:
                self.get_logger().error(f"Failed to load ArUco model: {e}")
                self.aruco_model = None
        else:
            self.get_logger().error("Ultralytics not available. Install with: pip install ultralytics")
            self.aruco_model = None
        
        # Publishers
        self.aruco_image_pub = self.create_publisher(
            Image,
            '/aruco_detection/annotated_image',
            10
        )
        self.aruco_position_pub = self.create_publisher(
            PointStamped,
            '/aruco_detection/position',
            10
        )
        self.aruco_status_pub = self.create_publisher(
            Bool,
            '/aruco_detection/detected',
            10
        )
        # Publisher para o docking (NOVO)
        self.dock_pose_pub = self.create_publisher(
            PoseStamped,
            '/detected_dock_pose',  # Tópico que o Nav2 vai subscrever
            10
        )
        
        # Subscriber
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        # Detection parameters
        self.confidence_threshold = 0.5
        self.aruco_detected = False
        
        self.get_logger().info("ArUco Detection Node started")

    def image_callback(self, msg):
        """Callback function to process image and detect ArUcos."""
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Process ArUco detection if model is available
            if self.aruco_model is not None:
                self.process_aruco_detection(cv_image, msg.header)
            else:
                self.get_logger().warn("ArUco model not available for detection")
                
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

    def process_aruco_detection(self, cv_image, header):
        """Process ArUco detection and publish dock pose"""
        if self.aruco_model is None:
            return cv_image
        
        try:
            # Run YOLO detection
            results = self.aruco_model(cv_image, conf=0.3)
            
            if len(results[0].boxes) > 0:
                # Get the best detection
                best_detection = results[0].boxes[0]
                confidence = float(best_detection.conf)
                
                if confidence > 0.3:
                    # Extract bounding box
                    x1, y1, x2, y2 = best_detection.xyxy[0].cpu().numpy()
                    center_x = (x1 + x2) / 2
                    center_y = (y1 + y2) / 2
                    
                    # Create dock pose from ArUco detection
                    dock_pose = PoseStamped()
                    dock_pose.header = header
                    dock_pose.header.frame_id = "camera_link"  # ou o frame da tua câmara
                    
                    # Estimar posição 3D baseada na deteção 2D
                    # (ajusta estes valores conforme a tua configuração)
                    dock_pose.pose.position.x = (center_x - cv_image.shape[1]/2) * 0.001  # Conversão pixel->metro
                    dock_pose.pose.position.y = (center_y - cv_image.shape[0]/2) * 0.001
                    dock_pose.pose.position.z = 0.5  # Distância estimada
                    
                    # Orientação (ajustar conforme necessário)
                    dock_pose.pose.orientation.x = 0.0
                    dock_pose.pose.orientation.y = 0.0
                    dock_pose.pose.orientation.z = 0.0
                    dock_pose.pose.orientation.w = 1.0
                    
                    # Publicar pose do dock
                    self.dock_pose_pub.publish(dock_pose)
                    
                    # Keep track of the best detection
                    if confidence > self.confidence_threshold:
                        self.aruco_detected = True
                        self.best_center_x = int((x1 + x2) / 2)
                        self.best_center_y = int((y1 + y2) / 2)
                        self.best_confidence = confidence
                    else:
                        self.aruco_detected = False
            
            # Publish the best detection position if any ArUco was found
            if self.aruco_detected:
                position_msg = PointStamped()
                position_msg.header = header
                position_msg.point.x = float(self.best_center_x)
                position_msg.point.y = float(self.best_center_y)
                position_msg.point.z = float(self.best_confidence)
                self.aruco_position_pub.publish(position_msg)
            
            # Publish detection status
            status_msg = Bool()
            status_msg.data = self.aruco_detected
            self.aruco_status_pub.publish(status_msg)
            
            # Create and publish annotated image if there are subscribers
            if self.aruco_image_pub.get_subscription_count() > 0:
                annotated_frame = results[0].plot(show=False)
                
                # Add extra information to image
                if self.aruco_detected:
                    cv2.putText(annotated_frame, f"ARUCO DETECTED! (conf: {self.best_confidence:.2f})", 
                               (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    cv2.putText(annotated_frame, f"Center: ({self.best_center_x}, {self.best_center_y})", 
                               (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                else:
                    cv2.putText(annotated_frame, "Searching for ArUco...", 
                               (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                
                # Publish annotated image
                annotated_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
                annotated_msg.header = header
                self.aruco_image_pub.publish(annotated_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error in ArUco detection: {e}")
        
        return cv_image


def main(args=None):
    rclpy.init(args=args)
    
    node = ArUcoDetectionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()