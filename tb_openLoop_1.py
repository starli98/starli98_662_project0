#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import csv

class OpenLoopController(Node):
    def __init__(self):
        super().__init__('tb_openLoop')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        self.timer_period = 0.1  # seconds
        self.duration = 10  # total duration in seconds
        self.velocity = 0.3  # m/s
        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.get_logger().info('Open Loop Controller Started')
        self.pose_data = []
        
    def timer_callback(self):
        elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        msg = Twist()
        if elapsed_time <= self.duration:
            msg.linear.x = self.velocity  # constant velocity
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing velocity: %f m/s' % msg.linear.x)
        else:
            # Stop the robot
            msg.linear.x = 0.0
            self.publisher_.publish(msg)
            self.get_logger().info('Stopping the robot.')
            # Cancel the timer after stopping
            self.timer.cancel()
            # Save the pose data
            self.save_pose_data()
            
    def odom_callback(self, msg):
        elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        position = msg.pose.pose.position
        self.pose_data.append([elapsed_time, position.x, position.y, position.z])
        self.get_logger().info('Time: %.2f s, Position -> x: %.2f m, y: %.2f m' % (elapsed_time, position.x, position.y))
        
    def save_pose_data(self):
        filename = 'pose_data.csv'
        with open(filename, 'w', newline='') as csvfile:
            pose_writer = csv.writer(csvfile)
            pose_writer.writerow(['Time', 'X', 'Y', 'Z'])
            pose_writer.writerows(self.pose_data)
        self.get_logger().info('Pose data saved to %s' % filename)

def main(args=None):
    rclpy.init(args=args)
    open_loop_controller = OpenLoopController()
    rclpy.spin(open_loop_controller)
    open_loop_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
