colcon build
source ~/ros2_ws/install/setup.bash


ros2 launch off_highway_uss sender_launch.py 
ros2 launch ros2_socketcan socket_can_sender.launch.py


ros2 launch off_highway_uss receiver_launch.py 
ros2 launch ros2_socketcan socket_can_receiver.launch.py

ros2 topic list = all topics are publishing data

ros2 run bosch_uss_status bosch_uss
