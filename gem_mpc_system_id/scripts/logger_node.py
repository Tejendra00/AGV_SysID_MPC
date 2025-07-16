#!/usr/bin/env python3

import rospy
import csv
import os
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDrive
import tf.transformations

class SimpleDataLogger:
    def __init__(self):
        self.csv_file = os.path.expanduser("~/gem_data_log.csv")
        self.last_cmd = None

        self.sub_odom = rospy.Subscriber("/gem/base_footprint/odom", Odometry, self.odom_callback)
        self.sub_cmd = rospy.Subscriber("/gem/ackermann_cmd", AckermannDrive, self.cmd_callback)

        with open(self.csv_file, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                "timestamp",
                "steering_angle", "speed",
                "odom_x", "odom_y", "odom_yaw",
                "linear_vel_x", "angular_vel_z"
            ])

        rospy.loginfo("CSV logger initialized at %s", self.csv_file)

    def cmd_callback(self, msg):
        self.last_cmd = msg

    def odom_callback(self, msg):
        if self.last_cmd is None:
            return  # Wait for a command first

        try:
            pos = msg.pose.pose.position
            ori = msg.pose.pose.orientation
            lin = msg.twist.twist.linear
            ang = msg.twist.twist.angular
            (_, _, yaw) = tf.transformations.euler_from_quaternion((ori.x, ori.y, ori.z, ori.w))

            timestamp = rospy.get_time()

            with open(self.csv_file, "a", newline="") as f:
                writer = csv.writer(f)
                writer.writerow([
                    timestamp,
                    self.last_cmd.steering_angle, self.last_cmd.speed,
                    pos.x, pos.y, yaw,
                    lin.x, ang.z
                ])

            rospy.loginfo_throttle(5, "Logged: steer=%.2f speed=%.2f x=%.2f y=%.2f yaw=%.2f",
                                   self.last_cmd.steering_angle, self.last_cmd.speed, pos.x, pos.y, yaw)

        except Exception as e:
            rospy.logwarn("Logging error: %s", str(e))


if __name__ == "__main__":
    rospy.init_node("simple_data_logger")
    SimpleDataLogger()
    rospy.loginfo("GEM Data Logger Started")
    rospy.spin()

