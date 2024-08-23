#!/usr/bin/env python3

import rospy
import csv
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32MultiArray
from message_filters import Subscriber, ApproximateTimeSynchronizer

class DataSaver:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('data_saver')

        # Create subscribers
        self.odom_sub = Subscriber('/synced_odom', Odometry)
        self.cmd_sub = Subscriber('/base_board/controller_cmd', Int32MultiArray)

        # Synchronize the messages with an approximate time policy
        self.ats = ApproximateTimeSynchronizer([self.odom_sub, self.cmd_sub], queue_size=10, slop=0.1, allow_headerless=True)
        self.ats.registerCallback(self.callback)

        # Open CSV file for writing
        self.csv_file = open('output.csv', 'w')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['x', 'y', 'z', 'yaw', 'accel', 'steer'])

        rospy.loginfo("Data saver node started.")
        rospy.spin()

    def callback(self, odom_msg, cmd_msg):
        # Extract data from Odometry message
        rospy.loginfo("Saving the topics.")

        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y
        z = odom_msg.pose.pose.position.z
        # Assuming the orientation is given in quaternion and converting to yaw
        _, _, yaw = self.quaternion_to_euler(odom_msg.pose.pose.orientation)

        # Extract data from controller_cmd message
        if len(cmd_msg.data) >= 2:
            accel = cmd_msg.data[0]
            steer = cmd_msg.data[1]
        else:
            accel = 0
            steer = 0

        # Write data to CSV
        self.csv_writer.writerow([x, y, z, yaw, accel, steer])

    def quaternion_to_euler(self, quaternion):
        # Convert quaternion to Euler angles (yaw)
        import tf.transformations as tft
        _, _, yaw = tft.euler_from_quaternion([
            quaternion.x,
            quaternion.y,
            quaternion.z,
            quaternion.w
        ])
        return _, _, yaw

    def __del__(self):
        # Close CSV file on exit
        self.csv_file.close()

if __name__ == '__main__':
    DataSaver()
