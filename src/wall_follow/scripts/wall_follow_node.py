#! /usr/bin/env python3

import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped



class WallFollow(Node):
    """ 
    Implement Wall Following on the car
    """
    def __init__(self):
        super().__init__('wall_follow_node')

        self.BEAM_ANGLE_A = 39.0
        self.BEAM_ANGLE_B = 90.0
        self.DESIRED_DISTANCE = 1.15
        self.VELOCITY = 3.0

        self.Kp = 0.18
        self.Kd = 0.01
        self.Ki = 0.0

        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        self.scan_subscriber = self.create_subscription(LaserScan, lidarscan_topic, self.scan_callback, 10)
        self.drive_publisher = self.create_publisher(AckermannDriveStamped, drive_topic, 10)

        self.integral = 0.0
        self.start_time = 0.0
        self.prev_time = 0.0
        self.curr_time = 0.0
        self.prev_error = 0.0
        self.error = 0.0

        

    def get_range(self, laser_msg, angle):
        """
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

        Args:
            laser_msg: incoming LaserScan message
            angle: between angle_min and angle_max of the LiDAR

        Returns:
            range: range measurement in meters at the given angle

        """

        if angle >= laser_msg.angle_min and angle <= laser_msg.angle_max:
            index = int((angle - laser_msg.angle_min) / laser_msg.angle_increment)

            if laser_msg.ranges[index] == float('inf') or laser_msg.ranges[index] == float('nan'):
                return laser_msg.range_max

            return laser_msg.ranges[index]

        else:
            return 0.0

    def get_error(self, laser_msg, dist):
        """
        Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()

        Args:
            laser_msg: incoming LaserScan message
            dist: desired distance to the wall
        """
        distance_a = self.get_range(laser_msg, np.radians(self.BEAM_ANGLE_A))
        #self.get_logger().info(f"Distance A: {distance_a}")
        distance_b = self.get_range(laser_msg, np.radians(self.BEAM_ANGLE_B))
        #self.get_logger().info(f"Distance B: {distance_b}")
        theta = np.radians(abs(self.BEAM_ANGLE_A - self.BEAM_ANGLE_B))
        alpha = np.arctan2((distance_a * np.cos(theta) - distance_b), (distance_a * np.sin(theta)))
        D_t = distance_b * np.cos(alpha)
        lookahead = 3.0
        D_t_plus_1 = D_t + lookahead * np.sin(alpha)
        self.get_logger().info(f"D_t_plus_1: {D_t_plus_1}")
        

        self.prev_error = self.error
        self.error = dist - D_t_plus_1
        self.get_logger().info(f"Error: {self.error}")

    def pid_control(self, error, velocity):
        """
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error
            velocity: desired velocity

        Returns:
            None
        """
        angle = 0.0

        self.integral += self.error

        self.prev_time = self.curr_time
        self.curr_time = self.get_clock().now().nanoseconds * 1e-9 

        derivative = (self.error - self.prev_error) / (self.curr_time - self.prev_time)

        angle = self.Kp * self.error + self.Kd * derivative
        #self.get_logger().info(f"Steering Angle: {angle}")

        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = -angle

        if np.abs(np.radians(angle)) >= 0.0 and np.abs(np.radians(angle)) < 10.0:
            drive_msg.drive.speed = self.VELOCITY
        elif np.abs(np.radians(angle)) >= 10.0 and np.abs(np.radians(angle)) < 20.0:
            drive_msg.drive.speed = self.VELOCITY * 0.66
        else:
            drive_msg.drive.speed = self.VELOCITY * 0.33

        self.drive_publisher.publish(drive_msg)

    def scan_callback(self, msg):
        """
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        """
        lidar_ranges = np.array(msg.ranges)

        self.get_error(msg, self.DESIRED_DISTANCE)
        #self.get_logger().info(f"Error: {self.error}")
        self.pid_control(self.error, self.VELOCITY)


def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    wall_follow_node = WallFollow()
    rclpy.spin(wall_follow_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wall_follow_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()