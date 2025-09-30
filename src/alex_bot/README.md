# alex_robot

## how to run code

before you begin, make a dev_ws directory in home, in dev_ws create src, clone the repo there. install all of the packages in requirements.txt. if some fail, it is ok.

### simulate movement in gazebo and rviz2

get to src directory and in one command window run

`colcon build --symlink-install`
`source install/setup.bash` (instead of this you can run
>`echo "source /path/to/your/ros2_ws/install/setup.bash" >> ~/.bashrc`
>`source ~/.bashrc`

once)
`ros2 launch alex_bot launch_sim.launch.py`

in another run

`rviz2 -d ABSPATH`

for arduino launch go to downloads

`./arduino-ide_nightly-20250311_Linux_64bit.AppImage --no-sandbox`

-----
`ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888`  --  on one terminal
`ros2 run alex_bot motor_command_publisher.py` -- on another terminal
`ros2 launch alex_bot ydlidar_launch_view.py` -- not in reccomended when double tabbing
`ros2 run alex_bot imu_to_tf.py ` -- to tranform imu data to rviz TF
`ros2 launch alex_bot round_bot.launch.py ` -- launch file

ros2 run joy joy_node
ros2 run alex_bot joystick_motor_publisher.py 

`chmod +x custom_nodes/joystick_motor_publisher.py` when making new nodes

run this before running plugging esp32.

    lsof | grep /dev/ttyUSB0
    sudo kill -9 5051

do this to kill the proccess if upload is not happening.
micro ros agent can not be running while uploading.

---

detection run:

    terminal 1:
    ros2 run camera_ros camera_node 

    terminal 2:

    source ~/ultralytics-venv/bin/activate


    python -u -m detections.yolov8_object_detection_ncnn_node --ros-args --log-level debug

    terminal 3:
    ros2 run rqt_image_view rqt_image_view 
