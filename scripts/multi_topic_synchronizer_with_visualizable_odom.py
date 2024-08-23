#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import Imu
from novatel_oem7_msgs.msg import BESTVEL, HEADING2
from message_filters import Subscriber, ApproximateTimeSynchronizer
import tf

class OdomPublisher:
    def __init__(self):
        # Initialize node
        rospy.init_node('multi_topic_synchronizer')

        # Create publishers
        self.pose_pub = rospy.Publisher('/synced_pose', PoseStamped, queue_size=10)
        self.odom_pub = rospy.Publisher('/synced_odom', Odometry, queue_size=10)
        self.visualizable_odom_pub = rospy.Publisher('/visualizable_odom', Odometry, queue_size=10)

        # Create subscribers
        self.odom_sub = Subscriber('/novatel/oem7/odom', Odometry)
        self.heading_sub = Subscriber('/novatel/oem7/heading2', HEADING2)
        self.bestvel_sub = Subscriber('/novatel/oem7/bestvel', BESTVEL)

        # Synchronize the messages with an approximate time policy
        self.ats = ApproximateTimeSynchronizer([self.odom_sub, self.heading_sub, self.bestvel_sub], queue_size=1, slop=0.1)
        self.ats.registerCallback(self.callback)

        # Variable to store initial odometry position
        self.initial_position = None

        rospy.loginfo("Synchronizer node started.")
        rospy.spin()

    def callback(self, odom_msg, heading_msg, bestvel_msg):
        # rospy.loginfo("Synchronized callback triggered!")
        # rospy.loginfo("Odometry timestamp: %s", str(odom_msg.header.stamp))
        # rospy.loginfo("Heading2 timestamp: %s", str(heading_msg.header.stamp))
        # rospy.loginfo("BESTVEL timestamp: %s", str(bestvel_msg.header.stamp))

        # Store the initial position of the first odom message
        if self.initial_position is None:
            self.initial_position = (odom_msg.pose.pose.position.x,
                                     odom_msg.pose.pose.position.y,
                                     odom_msg.pose.pose.position.z)
        
        # Extract position from odom_msg
        x = odom_msg.pose.pose.position.x - self.initial_position[0]
        y = odom_msg.pose.pose.position.y - self.initial_position[1]
        z = odom_msg.pose.pose.position.z - self.initial_position[2]

        # Extract velocities from bestvel_msg
        x_vel = bestvel_msg.hor_speed
        y_vel = bestvel_msg.ver_speed
        yaw = -heading_msg.heading/180*3.14 + 3.14/2

        # Create a new Pose message
        pose_msg = PoseStamped()
        pose_msg.header = odom_msg.header
        pose_msg.pose =  odom_msg.pose.pose

        # Convert yaw to a quaternion
        quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
        pose_msg.pose.orientation.x = quaternion[0]
        pose_msg.pose.orientation.y = quaternion[1]
        pose_msg.pose.orientation.z = quaternion[2]
        pose_msg.pose.orientation.w = quaternion[3]

        # Publish Pose message
        self.pose_pub.publish(pose_msg)

        # Create and publish new Odometry message
        new_odom_msg = Odometry()
        new_odom_msg.header = odom_msg.header
        new_odom_msg.child_frame_id = odom_msg.child_frame_id

        # Pose is based on odom_msg and pose_msg
        new_odom_msg.pose.pose = odom_msg.pose.pose

        # Set the velocity (linear and angular)
        new_odom_msg.twist.twist.linear.x = x_vel
        new_odom_msg.twist.twist.linear.y = y_vel
        # new_odom_msg.twist.twist.angular.z = yaw

        # Publish new Odometry message
        self.odom_pub.publish(new_odom_msg)

        # Create and publish visualizable Odometry message
        visualizable_odom_msg = Odometry()
        visualizable_odom_msg.header = odom_msg.header
        visualizable_odom_msg.child_frame_id = odom_msg.child_frame_id

        # Pose based on normalized position
        visualizable_pose_msg = Pose()
        visualizable_pose_msg.position.x = x
        visualizable_pose_msg.position.y = y
        visualizable_pose_msg.position.z = z

        # Convert yaw to a quaternion
        quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
        visualizable_pose_msg.orientation.x = quaternion[0]
        visualizable_pose_msg.orientation.y = quaternion[1]
        visualizable_pose_msg.orientation.z = quaternion[2]
        visualizable_pose_msg.orientation.w = quaternion[3]

        visualizable_odom_msg.pose.pose = visualizable_pose_msg
        visualizable_odom_msg.twist.twist.linear.x = x_vel
        visualizable_odom_msg.twist.twist.linear.y = y_vel
        # visualizable_odom_msg.twist.twist.angular.z = yaw

        # Publish visualizable Odometry message
        self.visualizable_odom_pub.publish(visualizable_odom_msg)

if __name__ == '__main__':
    OdomPublisher()
