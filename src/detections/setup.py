from setuptools import setup
import os
from glob import glob

package_name = 'detections'
model_dir = 'models'

all_model_files = []
for root, _, files in os.walk(model_dir):
    target_dir = os.path.join('share', package_name, root)
    src_files = [os.path.join(root, f) for f in files]
    all_model_files.append((target_dir, src_files))


setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        # Install package.xml
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # include models folder with subfolders
        *all_model_files,
        # Include utils
        (os.path.join('share', package_name, 'utils'), glob('utils/*')),
        # Install launch files
        ('share/' + package_name + '/launch', ['launch/yolo_ncnn.launch.py',]),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'opencv-python',
        'numpy',
        'ncnn',   # Optional — only if installed via pip or wheel
    ],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Object detection nodes using YOLOv8 and DNN in ROS2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'face_detector_node = detections.face_detector_node:main',
            'face_detector_dnn_node = detections.face_detector_dnn_node:main',
            'yolov8_object_detector_node = detections.yolov8_object_detector_node:main',
            'yolov8_object_detection_ncnn_node = detections.yolov8_object_detection_ncnn_node:main',
        ],
    },
)
