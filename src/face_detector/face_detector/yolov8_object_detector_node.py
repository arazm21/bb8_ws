import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import time

class YOLOObjectDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_object_detector')
        
        # Subscribe to camera images
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        
        self.publisher = self.create_publisher(
            Image,
            '/camera/image_objects',
            10)
        
        self.bridge = CvBridge()
        
        # Load ONNX model
        pkg_share = os.path.dirname(os.path.abspath(__file__))
        model_dir = os.path.join(pkg_share, '..', 'models')
        onnx_model = os.path.join(model_dir, 'yolov8s.onnx')
        
        # Check if ONNX model exists
        if not os.path.exists(onnx_model):
            self.get_logger().error(f"ONNX model not found: {onnx_model}")
            raise FileNotFoundError(f"ONNX model not found: {onnx_model}")
        
        # Initialize OpenCV DNN with ONNX model
        self.net = cv2.dnn.readNetFromONNX(onnx_model)
        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
        
        # YOLOv8 parameters
        self.input_size = (320, 320)  # Updated to match your exported model
        self.confidence_threshold = 0.5
        self.nms_threshold = 0.4
        
        # COCO class names
        self.class_names = [
            'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck', 'boat',
            'traffic light', 'fire hydrant', 'stop sign', 'parking meter', 'bench', 'bird', 'cat',
            'dog', 'horse', 'sheep', 'cow', 'elephant', 'bear', 'zebra', 'giraffe', 'backpack',
            'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee', 'skis', 'snowboard', 'sports ball',
            'kite', 'baseball bat', 'baseball glove', 'skateboard', 'surfboard', 'tennis racket',
            'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple',
            'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake',
            'chair', 'couch', 'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop',
            'mouse', 'remote', 'keyboard', 'cell phone', 'microwave', 'oven', 'toaster', 'sink',
            'refrigerator', 'book', 'clock', 'vase', 'scissors', 'teddy bear', 'hair drier',
            'toothbrush'
        ]
        
        # Performance metrics
        self.prev_time = time.time()
        self.fps = 0.0
        
        self.get_logger().info(f'YOLOv8 ONNX Object Detector Node initialized with model: {onnx_model}')

    def preprocess(self, frame):
        """Preprocess frame for YOLOv8 inference"""
        # YOLOv8 ONNX expects input in format: [1, 3, 640, 640]
        input_tensor = cv2.dnn.blobFromImage(
            frame, 
            scalefactor=1.0/255.0,  # Normalize to [0,1]
            size=self.input_size, 
            swapRB=True,  # BGR to RGB
            crop=False,
            ddepth=cv2.CV_32F
        )
        return input_tensor

    def postprocess(self, outputs, frame_shape):
        """Post-process YOLOv8 ONNX outputs"""
        h, w = frame_shape[:2]
        
        # YOLOv8 ONNX output shape for 320x320: [1, 84, 2100]
        # 84 = 4 (bbox coordinates) + 80 (class scores)
        # 2100 = (320/8)*(320/8)*3 + (320/16)*(320/16)*3 + (320/32)*(320/32)*3
        predictions = outputs[0]  # Remove batch dimension: [84, 2100]
        predictions = predictions.T  # Transpose to [2100, 84]
        
        boxes = []
        scores = []
        class_ids = []
        
        # Extract bounding boxes and class predictions
        for prediction in predictions:
            # First 4 values are bbox coordinates (center_x, center_y, width, height)
            # Coordinates are normalized to input size (640x640)
            cx, cy, w_box, h_box = prediction[:4]
            
            # Remaining 80 values are class scores
            class_scores = prediction[4:]
            class_id = np.argmax(class_scores)
            confidence = class_scores[class_id]
            
            if confidence > self.confidence_threshold:
                # Convert normalized coordinates to pixel coordinates
                # Scale from input_size back to original frame size
                x_scale = w / self.input_size[0]
                y_scale = h / self.input_size[1]
                
                # Convert center coordinates to corner coordinates
                x1 = int((cx - w_box / 2) * x_scale)
                y1 = int((cy - h_box / 2) * y_scale)
                box_w = int(w_box * x_scale)
                box_h = int(h_box * y_scale)
                
                # Ensure coordinates are within frame bounds
                x1 = max(0, min(x1, w))
                y1 = max(0, min(y1, h))
                box_w = max(0, min(box_w, w - x1))
                box_h = max(0, min(box_h, h - y1))
                
                boxes.append([x1, y1, box_w, box_h])
                scores.append(float(confidence))
                class_ids.append(class_id)
        
        # Apply Non-Maximum Suppression (NMS)
        if len(boxes) > 0:
            indices = cv2.dnn.NMSBoxes(
                boxes, 
                scores, 
                self.confidence_threshold, 
                self.nms_threshold
            )
            if len(indices) > 0:
                # OpenCV 4.x returns indices as a 2D array
                if isinstance(indices, np.ndarray) and len(indices.shape) == 2:
                    indices = indices.flatten()
                return [(boxes[i], scores[i], class_ids[i]) for i in indices]
        
        return []

    def draw_detections(self, frame, detections):
        """Draw bounding boxes and labels on frame"""
        for box, score, class_id in detections:
            x, y, w, h = box
            
            # Generate color based on class_id
            color = (
                int(class_id * 50) % 255,
                int(class_id * 80) % 255,
                int(class_id * 110) % 255
            )
            
            # Draw bounding box
            cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
            
            # Create label
            if class_id < len(self.class_names):
                label = f"{self.class_names[class_id]}: {score:.2f}"
            else:
                label = f"Class {class_id}: {score:.2f}"
            
            # Calculate label size for background
            (label_w, label_h), baseline = cv2.getTextSize(
                label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1
            )
            
            # Draw label background
            cv2.rectangle(
                frame, 
                (x, y - label_h - baseline - 5), 
                (x + label_w, y), 
                color, 
                -1
            )
            
            # Draw label text
            cv2.putText(
                frame, 
                label, 
                (x, y - baseline - 5), 
                cv2.FONT_HERSHEY_SIMPLEX, 
                0.5, 
                (255, 255, 255), 
                1
            )

    def image_callback(self, msg: Image):
        try:
            # Convert ROS Image to OpenCV frame
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # FPS calculation
            now = time.time()
            dt = now - self.prev_time
            self.prev_time = now
            if dt > 0:
                self.fps = 1.0 / dt
            
            # Inference
            proc_start = time.time()
            
            # Preprocess
            input_tensor = self.preprocess(frame)
            self.net.setInput(input_tensor)
            
            # Forward pass
            outputs = self.net.forward()
            
            # Post-process
            detections = self.postprocess(outputs, frame.shape)
            
            proc_time = (time.time() - proc_start) * 1000  # ms
            
            # Draw detections
            self.draw_detections(frame, detections)
            
            # Draw performance metrics
            cv2.putText(frame, f"FPS: {self.fps:.1f}", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(frame, f"Inference: {proc_time:.1f} ms", (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(frame, f"Objects: {len(detections)}", (10, 90),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # Publish annotated frame
            out_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            out_msg.header = msg.header
            self.publisher.publish(out_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')
            import traceback
            self.get_logger().error(f'Traceback: {traceback.format_exc()}')

def main(args=None):
    rclpy.init(args=args)
    node = YOLOObjectDetectorNode()
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()