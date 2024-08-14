import rclpy
from rclpy.node import Node
from off_highway_uss_msgs.msg import Objects
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import numpy as np
from nav_msgs.msg import Odometry

class CEMOde(Node):
    def __init__(self):
        super().__init__('object_subscriber')
        
        self.ce_sub = self.create_subscription(
            Objects,
            '/objects',
            self.listener_callback,
            10)

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        self.get_logger().info('Listening to /objects topic')
        
        self.vel_pub = self.create_publisher(Twist, "/osorno2/uss_vel", 10)
        self.control_publisher = self.create_publisher(String, '/robot_control', 10)
           
        self.readings = []  
    
        self.current_linear_speed = 0.0
        self.current_angular_speed = 0.0
 

    def listener_callback(self, msg):
        current_reading = []
        
        for obj in msg.objects:           
            dist1 = obj.position_first.x
            current_reading.append(dist1)
        
        # print("coming readings ....", self.readings)   
        
        self.readings.append(current_reading)
        
        if len(self.readings) > 5:
            self.readings.pop(0)
        
        if len(self.readings) == 5:
            self.process_readings()

    def process_readings(self):
        stop = False
        slow_down = False

        all_positions_x = [x for reading in self.readings for x in reading]
        
        if not all_positions_x:
            return
        
        # print("---- All positions: ", all_positions_x)
        
        median_dist = np.median(all_positions_x)
        
        print("---- Median distance: ", median_dist)

        if 1.00 <= median_dist < 6.00:
            slow_down = True
        elif 0 <= median_dist < 1.00:
            stop = True
        
        if stop:
            # self.send_control_command('****************STOP*******************')
            self.stop_robot()
        elif slow_down:
            # self.send_control_command('****************SLOW DOWN*******************')
            self.slow_robot()
        
    def send_control_command(self, command):
        msg = String()
        msg.data = command
        self.control_publisher.publish(msg)
        print(f'Sent control command: {command}')

    def odom_callback(self, msg):
        self.current_linear_speed = msg.twist.twist.linear.x
        self.current_angular_speed = msg.twist.twist.angular.z
    
    def stop_robot(self):
        print("**************************STOP ROBOT**************************")
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
 
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        self.vel_pub.publish(msg)
 
    def slow_robot(self):
        print("**************************SLOW DOWN ROBOT**************************")
        msg = Twist()
        
        # --- next ---
        # msg.linear.x = 0.3  
        msg.linear.x = self.current_linear_speed * 0.5  # Slow down to 50% of the current speed
        
        msg.linear.y = 0.0
        msg.linear.z = 0.0
 
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        # msg.angular.z = 0.0  
        msg.angular.z = self.current_angular_speed * 0.5  # Slow down to 50% of the current speed
 
        self.vel_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    ce_mode = CEMOde()
    rclpy.spin(ce_mode)
    ce_mode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
