#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time
from std_msgs.msg import Float64
from ros_gz_interfaces.msg import EntityWrench
from geometry_msgs.msg import Pose, Twist

class CommanderNode(Node):
    def __init__(self):
        super().__init__('commander')
        
        self.cart_pose = Pose()
        self.pole_pose = Pose()
        self.pole_twist = Twist()
        self.y_angular = 0
        self.cart_pose_x = 0
        
        # Initialize PID variables
        self.yaw_angle = 0
        self.target_yaw_angle = 0
        self.target_cart_pose_x = 0
        self.integral_position_error = 0
        self.integral_yaw_error = 0
        self.last_error_yaw = 0
        self.last_error_pos = 0
        
        # PID gains
        self.Kp_y = 18
        self.Ki_y = 19.34
        self.Kd_y = 7.75
        self.Kp_p = 0
        self.Ki_p = 6.18
        self.Kd_p = 5.5
        
        self.time_interval = 0.005
        self.last_time = time.time()
        
        # Create subscriber
        self.subscription = self.create_subscription(
            LinkStates,
            '/gazebo/link_states',
            self.get_cart_pose,
            10
        )
        
        # Create publisher
        self.pub_cart = self.create_publisher(Float64, '/cart_controller/command', 10)
        
        # Create timer for control loop
        self.timer = self.create_timer(self.time_interval, self.commander_loop)

    def get_cart_pose(self, data):
        try:
            ind = data.name.index('cart_pole::cart_link')
            self.cart_pose = data.pose[ind]

            ind_pitch = data.name.index('cart_pole::pole_link')
            self.pole_twist = data.twist[ind_pitch]

            self.cart_pose_x = self.cart_pose.position.x
            self.y_angular = self.pole_twist.angular.y
        except ValueError:
            # Handle case where link names are not found
            pass

    def commander_loop(self):
        current_time = time.time()
        actual_interval = current_time - self.last_time
        self.last_time = current_time
        
        # Use actual time interval for better accuracy
        time_interval = actual_interval if actual_interval > 0 else self.time_interval

        self.yaw_angle += self.y_angular * time_interval
        error_yaw = self.target_yaw_angle - self.yaw_angle
        self.integral_yaw_error += (error_yaw + self.last_error_yaw) * time_interval / 2
        effort_yaw = -(self.Kp_y * error_yaw + 
                       self.Ki_y * self.integral_yaw_error +
                       self.Kd_y * (error_yaw - self.last_error_yaw) / time_interval)
        
        error_pos = self.target_cart_pose_x - self.cart_pose_x
        self.integral_position_error += (error_pos + self.last_error_pos) * time_interval / 2
        effort_pos = -(self.Kp_p * error_pos + 
                       self.Ki_p * self.integral_position_error +
                       self.Kd_p * (error_pos - self.last_error_pos) / time_interval)    
        
        effort = effort_yaw + effort_pos    
        self.last_error_yaw = error_yaw
        self.last_error_pos = error_pos
        
        # Publish effort as Float64 message
        msg = Float64()
        msg.data = effort
        self.pub_cart.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    commander_node = CommanderNode()
    
    try:
        rclpy.spin(commander_node)
    except KeyboardInterrupt:
        pass
    finally:
        commander_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()