#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

class KalmanFilter:

    def __init__(self):
        rospy.init_node("kalman_filter")
        self.odom_sub = rospy.Subscriber("/odom_noisy", Odometry, self.odomCallback, queue_size=10)
        self.imu_sub = rospy.Subscriber("imu", Imu, self.imuCallback, queue_size=10)
        self.odom_pub = rospy.Publisher("/odom_kalman", Odometry, queue_size=10)
        
        self.mean = 0.0
        self.variance = 1000.0

        self.motion_variance = 4.0
        self.measurement_variance = 0.5

        self.imu_angular_z = 0.0

        self.is_first_odom = True
        self.last_angular_z = 0.0
        self.motion = 0.0

        self.kalman_odom = Odometry()

    def odomCallback(self, odom):
        self.kalman_odom = odom

        if self.is_first_odom:
            self.last_angular_z = odom.twist.twist.angular.z
            self.is_first_odom = False
            self.mean = odom.twist.twist.angular.z
            return
        
        self.motion = odom.twist.twist.angular.z - self.last_angular_z

        self.statePrediction()
        self.measurementUpdate()

        self.last_angular_z = odom.twist.twist.angular.z

        self.kalman_odom.twist.twist.angular.z = self.mean
        self.odom_pub.publish(self.kalman_odom)

    def imuCallback(self, imu):
        self.imu_angular_z = imu.angular_velocity.z

    def measurementUpdate(self):
        self.mean = (self.measurement_variance * self.mean + self.variance * self.imu_angular_z) \
         / (self.variance + self.measurement_variance)
                     
        self.variance = (self.variance * self.measurement_variance) \
                       / (self.variance + self.measurement_variance)

    def statePrediction(self):
        self.mean = self.mean + self.motion
        self.variance = self.variance + self.motion_variance

def main():
    KalmanFilter()
    rospy.spin()

if __name__ == '__main__':
    main()
