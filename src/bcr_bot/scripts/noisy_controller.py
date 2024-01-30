#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
import numpy as np
from tf.broadcaster import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math
from tf.transformations import quaternion_from_euler

class NoisyController:

    def __init__(self):
        rospy.init_node("noisy_controller")
        self.wheel_radius = rospy.get_param("~wheel_radius", 0.033)
        self.wheel_separation = rospy.get_param("~wheel_separation", 0.17)

        rospy.loginfo("Tekerlek yaricapi %f" % self.wheel_radius)
        rospy.loginfo("iki teker arasi mesafe %f" % self.wheel_separation)

        self.left_wheel_prev_pos = 0.0
        self.right_wheel_prev_pos = 0.0
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        rospy.Subscriber("joint_states", JointState, self.jointCallback)
        self.odom_pub = rospy.Publisher("/odom_noisy", Odometry, queue_size=10)

        self.speed_conversion = np.array([[self.wheel_radius/2, self.wheel_radius/2],
                                          [self.wheel_radius/self.wheel_separation, -self.wheel_radius/self.wheel_separation]])

        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = "odom"
        self.odom_msg.child_frame_id = "base_footprint_ekf"
        self.odom_msg.pose.pose.orientation.w = 1.0

        self.br = TransformBroadcaster()

        self.prev_time = rospy.Time.now()

    def jointCallback(self, msg):
        wheel_encoder_left = msg.position[1] + np.random.normal(0, 0.005)
        wheel_encoder_right = msg.position[0] + np.random.normal(0, 0.005)

        dp_left = wheel_encoder_left - self.left_wheel_prev_pos
        dp_right = wheel_encoder_right - self.right_wheel_prev_pos
        dt = rospy.Time.now() - self.prev_time

        self.left_wheel_prev_pos = msg.position[1]
        self.right_wheel_prev_pos = msg.position[0]
        self.prev_time = rospy.Time.now()

        fi_left = dp_left / dt.to_sec()
        fi_right = dp_right / dt.to_sec()

        linear = (self.wheel_radius * fi_right + self.wheel_radius * fi_left) / 2
        angular = (self.wheel_radius * fi_right - self.wheel_radius * fi_left) / self.wheel_separation

        d_s = (self.wheel_radius * dp_right + self.wheel_radius * dp_left) / 2
        d_theta = (self.wheel_radius * dp_right - self.wheel_radius * dp_left) / self.wheel_separation
        self.theta += d_theta
        self.x += d_s * math.cos(self.theta)
        self.y += d_s * math.sin(self.theta)

        q = quaternion_from_euler(0, 0, self.theta)
        self.odom_msg.header.stamp = rospy.Time.now()
        self.odom_msg.pose.pose.position.x = self.x
        self.odom_msg.pose.pose.position.y = self.y
        self.odom_msg.pose.pose.orientation.x = q[0]
        self.odom_msg.pose.pose.orientation.y = q[1]
        self.odom_msg.pose.pose.orientation.z = q[2]
        self.odom_msg.pose.pose.orientation.w = q[3]
        self.odom_msg.twist.twist.linear.x = linear
        self.odom_msg.twist.twist.angular.z = angular
        self.odom_pub.publish(self.odom_msg)

        self.br.sendTransform((self.x, self.y, 0),
                              quaternion_from_euler(0, 0, self.theta),
                              rospy.Time.now(),
                              "base_footprint_noisy",
                              "odom")

def main():
    NoisyController()
    rospy.spin()

if __name__ == '__main__':
    main()
