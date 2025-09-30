from setuptools import find_packages, setup

package_name = 'face_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        ('share/face_detector/launch', [
        'launch/face_detector.launch.py',
        'launch/face_detector_dnn.launch.py',   # include new DNN launch
        ]),
        ('share/face_detector/models', [
            'models/deploy.prototxt',
            'models/res10_300x300_ssd_iter_140000.caffemodel',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bb8',
    maintainer_email='bb8@todo.todo',
    
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'face_detector = face_detector.face_detector_node:main',
            'face_detector_dnn = face_detector.face_detector_dnn_node:main',
        ],
    },
)
