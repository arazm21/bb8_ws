import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os


class FaceDetectorNode(Node):
    def __init__(self):
        super().__init__('face_detector')
        # Subscribe to camera images (match your topic namespace)
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Publisher for annotated image stream
        self.publisher = self.create_publisher(
            Image,
            '/camera/image_faces',
            10)

        # Bridge between ROS and OpenCV
        self.bridge = CvBridge()

        # Locate Haar cascade classifier (frontal face)
        try:
            cascade_path = cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
        except AttributeError:
            cascade_path = '/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml'
        if not os.path.exists(cascade_path):
            self.get_logger().error(f'Cascade file not found: {cascade_path}')
            return

        self.face_cascade = cv2.CascadeClassifier(cascade_path)
        if self.face_cascade.empty():
            self.get_logger().error(f'Failed to load cascade at: {cascade_path}')
        else:
            self.get_logger().info(f'Loaded Haar cascade from: {cascade_path}')

    def image_callback(self, msg: Image):
        # Convert ROS Image message to OpenCV image (BGR8)
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'CV Bridge error: {e}')
            return

        # Convert to grayscale for detection
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Detect faces
        faces = self.face_cascade.detectMultiScale(
            gray,
            scaleFactor=1.1,
            minNeighbors=5,
            minSize=(30, 30)
        )

        # Draw rectangles around detected faces
        for (x, y, w, h) in faces:
            cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # Convert annotated image back to ROS Image message
        try:
            annotated_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            annotated_msg.header = msg.header
            self.publisher.publish(annotated_msg)
        except Exception as e:
            self.get_logger().error(f'CV Bridge error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = FaceDetectorNode()
    if node.face_cascade is None or node.face_cascade.empty():
        # Initialization failed, shutdown
        rclpy.shutdown()
        return
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
