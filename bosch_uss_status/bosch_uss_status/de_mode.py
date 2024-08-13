#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from off_highway_uss_msgs.msg import DirectEchos  
from std_msgs.msg import String  
import numpy as np

# -- DIRECT ECHO MODE --
class BoschUss(Node):
    def __init__(self):
        super().__init__('direct_echos_listener')
        self.subscription = self.create_subscription(
            DirectEchos,
            '/direct_echos',
            self.listener_callback,
            10
        )
        self.control_publisher = self.create_publisher(String, '/robot_control', 10)
        
        self.readings = {
            0: {'first': [], 'first_filtered': []},
            1: {'first': [], 'first_filtered': []},
            2: {'first': [], 'first_filtered': []}
        }

        self.get_logger().info('Listening to /direct_echos topic')
 
    def listener_callback(self, msg):
        for echo in msg.direct_echos:
            if echo.id == 3:
                continue
        
            self.readings[echo.id]['first'].append(echo.first.distance)
            self.readings[echo.id]['first_filtered'].append(echo.first_filtered.distance)
            
        # after 5 readings
        if all(len(self.readings[sensor_id]['first']) >= 5 for sensor_id in self.readings):
            self.process_readings()
            
            # Pop the first reading after processing for all sensors
            for sensor_id in self.readings:
                self.readings[sensor_id]['first'].pop(0)
                self.readings[sensor_id]['first_filtered'].pop(0)
 
    def process_readings(self):
        medians = {}

        for sensor_id in self.readings:
            self.readings[sensor_id]['first'] =[0 if num == 1002 else num for num in self.readings[sensor_id]['first']]
            self.readings[sensor_id]['first_filtered'] =[0 if num == 1002 else num for num in self.readings[sensor_id]['first_filtered']]
            
            # self.readings[sensor_id]['first'] =[0 if num == 1003 else num for num in self.readings[sensor_id]['first']]
            # self.readings[sensor_id]['first_filtered'] =[0 if num == 1003 else num for num in self.readings[sensor_id]['first_filtered']]
            
            print(f"Sensor ID: {sensor_id} First Echo: {self.readings[sensor_id]['first']}  First Filtered Echo: {self.readings[sensor_id]['first_filtered']}")

            first_filtered_combined = self.readings[sensor_id]['first'] #+ self.readings[sensor_id]['first_filtered']
            
            medians[sensor_id] = np.median(first_filtered_combined)
            
        print(f"Medians: {medians}\n")
        
        self.apply_conditions(medians)
        
    def apply_conditions(self, medians):
        slow_down = False
        stop = False
        normal= False

        median_sensor1 = medians[0]
        median_sensor2 = medians[1]
        median_sensor3 = medians[2]
        
        if 350 <= median_sensor1 < 550 or 350 <= median_sensor2 < 550 or 350 <= median_sensor3 < 550:
            slow_down = True
        if 0 <= median_sensor1 < 350 or 0 <= median_sensor2 < 350 or 0 <= median_sensor3 < 350:
            stop = True
        elif median_sensor1 == 1003 or median_sensor2 == 1003 or median_sensor3 == 1003:
            normal = True
        
        
        if stop:
            self.send_control_command('****************STOP*******************')
        elif slow_down:
            self.send_control_command('***************SLOW DOWN*****************')
        elif normal:
            self.send_control_command('***************NORMAL SPEED*****************')
        
        
    def send_control_command(self, command):
        msg = String()
        msg.data = command
        self.control_publisher.publish(msg)
        print(f'Sent control command: {command}')
 
def main(args=None):
    rclpy.init(args=args)
    node = BoschUss()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
 
if __name__ == '__main__':
    main()
