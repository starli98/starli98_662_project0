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
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # Parameters for Scenario 2
        self.v_max = 0.9  # Maximum velocity in m/s
        self.acceleration = 0.3  # Acceleration in m/s^2
        self.s_constant = 1.0  # Distance during constant velocity in meters

        # Calculated times and distances
        self.t1 = self.v_max / self.acceleration  # Time to reach v_max
        self.s1 = 0.5 * self.acceleration * self.t1**2  # Distance during acceleration
        self.t3 = self.v_max / self.acceleration  # Time to decelerate to stop
        self.s3 = self.s1  # Distance during deceleration (same as s1)

        self.t2 = self.s_constant / self.v_max  # Time during constant velocity
        self.t_total = self.t1 + self.t2 + self.t3  # Total time

        self.start_time = self.get_clock().now()
        self.get_logger().info('Open Loop Controller for Scenario 2 Started')
        self.pose_data = []

    def timer_callback(self):
        elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        msg = Twist()

        if elapsed_time <= self.t1:
            # Acceleration phase
            current_velocity = self.acceleration * elapsed_time
            if current_velocity > self.v_max:
                current_velocity = self.v_max
            msg.linear.x = current_velocity
            self.publisher_.publish(msg)
            self.get_logger().info('Accelerating: Velocity = %.2f m/s' % msg.linear.x)
        elif self.t1 < elapsed_time <= (self.t1 + self.t2):
            # Constant velocity phase
            msg.linear.x = self.v_max
            self.publisher_.publish(msg)
            self.get_logger().info('Cruising: Velocity = %.2f m/s' % msg.linear.x)
        elif (self.t1 + self.t2) < elapsed_time <= self.t_total:
            # Deceleration phase
            time_into_deceleration = elapsed_time - (self.t1 + self.t2)
            current_velocity = self.v_max - self.acceleration * time_into_deceleration
            if current_velocity < 0.0:
                current_velocity = 0.0
            msg.linear.x = current_velocity
            self.publisher_.publish(msg)
            self.get_logger().info('Decelerating: Velocity = %.2f m/s' % msg.linear.x)
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
        filename = 'pose_data_scenario2.csv'
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

