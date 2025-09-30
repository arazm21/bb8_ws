import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import time

class DNNFaceDetectorNode(Node):
    def __init__(self):
        super().__init__('dnn_face_detector')

        # Subscribe to camera images
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        self.publisher = self.create_publisher(
            Image,
            '/camera/image_faces',
            10)

        self.bridge = CvBridge()

        # Load DNN model
        pkg_share = os.path.dirname(os.path.abspath(__file__))
        model_dir = os.path.join(pkg_share, '..', 'models')
        proto = os.path.join(model_dir, 'deploy.prototxt')
        model = os.path.join(model_dir, 'res10_300x300_ssd_iter_140000.caffemodel')
        self.net = cv2.dnn.readNetFromCaffe(proto, model)

        # Performance metrics
        self.prev_time = time.time()
        self.fps = 0.0

    def image_callback(self, msg: Image):
        # Convert ROS Image to OpenCV frame
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        (h, w) = frame.shape[:2]

        # FPS calculation
        now = time.time()
        dt = now - self.prev_time
        self.prev_time = now
        if dt > 0:
            self.fps = 1.0 / dt

        # Inference
        proc_start = time.time()
        blob = cv2.dnn.blobFromImage(cv2.resize(frame, (300, 300)), 1.0,
                                     (300, 300), (104.0, 177.0, 123.0))
        self.net.setInput(blob)
        detections = self.net.forward()
        proc_time = (time.time() - proc_start) * 1000  # ms

        # Draw detections
        for i in range(detections.shape[2]):
            conf = detections[0, 0, i, 2]
            if conf < 0.5:
                continue
            box = detections[0, 0, i, 3:7] * [w, h, w, h]
            sx, sy, ex, ey = box.astype('int')
            cv2.rectangle(frame, (sx, sy), (ex, ey), (0, 255, 0), 2)
            text = f"{conf * 100:.1f}%"
            # Draw confidence level
            label = f"{conf:.2f}"
            cv2.putText(frame, label, (int(sx), int(sy) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

        # Overlay performance metrics
        cv2.putText(frame, f"FPS: {self.fps:.1f}", (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(frame, f"Time: {proc_time:.1f} ms", (10, 45),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # Publish annotated frame
        out_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        out_msg.header = msg.header
        self.publisher.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DNNFaceDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
