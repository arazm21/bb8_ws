import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import ncnn
import os
import time

class YOLONCNNObjectDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_ncnn_object_detector')
        
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
        
        # Load NCNN model
        pkg_share = os.path.dirname(os.path.abspath(__file__))
        model_dir = os.path.join(pkg_share, '..', 'models')
        ncnn_model_dir = os.path.join(model_dir, 'yolov8n_ncnn_model')
        param_file = os.path.join(ncnn_model_dir, 'model.ncnn.param')
        bin_file = os.path.join(ncnn_model_dir, 'model.ncnn.bin')
        
        # Check if NCNN model files exist
        if not os.path.exists(param_file):
            self.get_logger().error(f"NCNN param file not found: {param_file}")
            raise FileNotFoundError(f"NCNN param file not found: {param_file}")
        
        if not os.path.exists(bin_file):
            self.get_logger().error(f"NCNN bin file not found: {bin_file}")
            raise FileNotFoundError(f"NCNN bin file not found: {bin_file}")
        
        # Initialize NCNN network
        self.net = ncnn.Net()
        self.net.load_param(param_file)
        self.net.load_model(bin_file)
        
        # YOLOv8 parameters
        self.input_size = (320, 320)
        self.confidence_threshold = 0.4
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
        
        self.get_logger().info(f'YOLOv8 NCNN Object Detector Node initialized with model: {ncnn_model_dir}')

    def preprocess(self, frame):
        """Preprocess frame for YOLOv8 NCNN inference"""
        # Resize frame to input size
        resized = cv2.resize(frame, self.input_size)
        
        # Convert BGR to RGB and normalize to [0,1]
        rgb_frame = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        normalized = rgb_frame.astype(np.float32) / 255.0
        
        # Convert HWC to CHW format (channels first)
        chw = np.transpose(normalized, (2, 0, 1))
        
        return chw

    def run_inference(self, input_data):
        """Run NCNN inference"""
        try:
            # Create NCNN Mat from input data - ensure contiguous array
            input_data = np.ascontiguousarray(input_data, dtype=np.float32)
            mat_in = ncnn.Mat(input_data)
            
            # Create extractor
            ex = self.net.create_extractor()
            
            # Set input
            ret = ex.input("in0", mat_in)
            if ret != 0:
                self.get_logger().error(f"Failed to set input: {ret}")
                return None
            
            # Extract output
            ret, mat_out = ex.extract("out0")
            if ret != 0:
                self.get_logger().error(f"NCNN inference failed with return code: {ret}")
                return None
            
            # Convert NCNN Mat to numpy array
            output = np.array(mat_out)
            
            # Debug: Print output shape
            self.get_logger().debug(f"NCNN output shape: {output.shape}")
            
            return output
            
        except Exception as e:
            self.get_logger().error(f"Error in NCNN inference: {str(e)}")
            return None

    def postprocess(self, output, frame_shape):
        """Post-process YOLOv8 NCNN outputs"""
        if output is None:
            return []
            
        h, w = frame_shape[:2]
        
        # Handle different output shapes
        if len(output.shape) == 3:
            # Remove batch dimension if present: [1, 84, 2100] -> [84, 2100]
            if output.shape[0] == 1:
                output = output.squeeze(0)
        elif len(output.shape) == 1:
            # If 1D, reshape based on expected dimensions
            # For 320x320 input, expect 84 * 2100 = 176400 elements
            expected_size = 84 * 2100
            if output.shape[0] == expected_size:
                output = output.reshape(84, 2100)
            else:
                self.get_logger().error(f"Unexpected output size: {output.shape}")
                return []
        
        # Ensure we have the right shape: [84, 2100]
        if output.shape[0] == 84:
            predictions = output.T  # Transpose to [2100, 84]
        elif output.shape[1] == 84:
            predictions = output    # Already [2100, 84]
        else:
            self.get_logger().error(f"Unexpected output shape: {output.shape}")
            return []
        
        boxes = []
        scores = []
        class_ids = []
        
        # Extract bounding boxes and class predictions
        for prediction in predictions:
            # First 4 values are bbox coordinates (center_x, center_y, width, height)
            # These are normalized to input size (320x320)
            cx, cy, w_box, h_box = prediction[:4]
            
            # Remaining 80 values are class scores
            class_scores = prediction[4:84]  # Make sure we only take 80 classes
            class_id = np.argmax(class_scores)
            confidence = class_scores[class_id]
            
            if confidence > self.confidence_threshold:
                # Convert normalized coordinates to pixel coordinates
                # NCNN coordinates are normalized to input size, not 0-1
                # So we need to scale from input_size back to original frame size
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
        
        # Debug: Log detection count before NMS
        self.get_logger().debug(f"Detections before NMS: {len(boxes)}")
        
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
                final_detections = [(boxes[i], scores[i], class_ids[i]) for i in indices]
                self.get_logger().debug(f"Final detections after NMS: {len(final_detections)}")
                return final_detections
        
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
            input_data = self.preprocess(frame)
            
            # Run NCNN inference
            output = self.run_inference(input_data)
            
            # Post-process
            detections = self.postprocess(output, frame.shape)
            
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
    node = YOLONCNNObjectDetectorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()