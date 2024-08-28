The two packages in this repository need to be cloned in the ros2 workspace. 

For setting CAN connection, read in detail [here](https://www.notion.so/paltech-gmbh/Implement-ROS2-node-for-Bosch-Sensors-to-stop-the-robot-7b8cedcde19441f2879d112612fc8328?pvs=4).

### For building the ```off_highway_sensor_drivers``` and ```ros2_socketcan``` pacakge (in separate terminals for each package):

Run the following:

```bash
colcon build
```

```bash
source ~/ros2_ws/install/setup.bash
```

With everything setup, follow the below steps:

1. Build and source the workspace, same as above, for each below terminal commands.

3. In each separate termainal, launch the off_highway_uss sender and receiver launch files alongside launch the ros2_socketcan sender and receiver files:

```bash
ros2 launch off_highway_uss sender_launch.py 
```

```bash
ros2 launch ros2_socketcan socket_can_sender.launch.py
```

```bash
ros2 launch off_highway_uss receiver_launch.py 
```

```bash
ros2 launch ros2_socketcan socket_can_receiver.launch.py
```

3. Before running the ROS node, try to see the topics being published: 

```bash
ros2 topic list
```

4. Specifically check for the /direct_echos topic:

```bash
ros2 topic echo /direct_echos
```

5. Now, run the ROS2 node as below in a new terminal after building and sourcing the workspace: 

```bash
ros2 run bosch_uss_status ce_mode
```

### CONNECTING WITH ROBOT 

1. Connect the wifi first (basestation...)

2. Type the following command to not have to build each terminal for launching:
```bash
byobu
```

3. Set the ssh connection.

For byobu:

fn + f2 = new terminal

fn + f3 = switch terminals

in separate byobu terminals:

repeat the same process as mentioned above.

