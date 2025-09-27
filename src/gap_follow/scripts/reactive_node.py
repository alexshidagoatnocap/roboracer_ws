#! /usr/bin/env python3

import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

class ReactiveFollowGap(Node):
    """ 
    Implement Wall Following on the car
    This is just a template, you are free to implement your own node!
    """
    def __init__(self):
        super().__init__('reactive_node')

        self.velocity = 1.5
        self.max_distance_threshold = 10.0
        self.bubble_radius = 1
        self.processed_ranges = None

        # Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        self.lidar_sub = self.create_subscription(LaserScan, lidarscan_topic, self.lidar_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, drive_topic, 10)

    def preprocess_lidar(self, ranges):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)

            Returns: numpy array of preprocessed ranges
        """
        proc_ranges = np.array(ranges)

        for index in range(len(proc_ranges)):
            if proc_ranges[index] == float('inf') or proc_ranges[index] == float('nan'):
                proc_ranges[index] = 0.0

        proc_ranges[proc_ranges > self.max_distance_threshold] = self.max_distance_threshold # make sure no distances are greater than max_distance
        return proc_ranges

    def find_max_gap(self):
        """ Return the start index & end index of the max gap in processed_ranges
        """
        # Find nearest lidar point and surround points with bubble of zero length ranges
        nearest_range = float('inf')
        nearest_range_index = -1

        for i in range(len(self.processed_ranges)):
            if self.processed_ranges[i] < nearest_range:
                nearest_range = self.processed_ranges[i]
                nearest_range_index = i

        if nearest_range_index != -1:
            for i in range(nearest_range_index, nearest_range_index + self.bubble_radius):
                    self.processed_ranges[i] = 0.0

            for i in range(nearest_range_index, nearest_range_index - self.bubble_radius, -1):
                    self.processed_ranges[i] = 0.0

        # Find max length sequence of non-zero lengths
        max_gap_index_start = 0
        max_gap_counter = 0
        current_gap_index_start = 0
        current_gap_counter = 0

        for i in range(len(self.processed_ranges)):
            if self.processed_ranges[i] == 0.0:
                if current_gap_counter > max_gap_counter:
                    max_gap_counter = current_gap_counter
                    max_gap_index_start = current_gap_index_start
                current_gap_counter = 0
            else:
                if current_gap_counter == 0:
                    current_gap_index_start = i
                current_gap_counter += 1

        return (max_gap_index_start, max_gap_index_start + max_gap_counter)

    def find_best_point(self, start_i, end_i):
        """Start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of best point in ranges
	    Naive: Choose the furthest point within ranges and go there
        """
        if start_i < 0 or end_i > len(self.processed_ranges):
            return None

        best_point = start_i
        for i in range(start_i, end_i):
            if self.processed_ranges[i] > self.processed_ranges[best_point]:
                best_point = i

        return best_point

    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        ranges = np.array(data.ranges)
        self.processed_ranges = self.preprocess_lidar(ranges)

        # TODO:
        #Find closest point to LiDAR

        #Eliminate all points inside 'bubble' (set them to zero) 
        #Find max length gap 
        max_gap = self.find_max_gap()

        #Find the best point in the gap 
        best_point = self.find_best_point(max_gap[0], max_gap[1])

        #Publish Drive message
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = data.angle_min + best_point * data.angle_increment
        drive_msg.drive.speed = self.velocity
        self.drive_pub.publish(drive_msg)


def main(args=None):
    rclpy.init(args=args)
    print("Gap Follow Initialized")
    reactive_node = ReactiveFollowGap()
    rclpy.spin(reactive_node)

    reactive_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()