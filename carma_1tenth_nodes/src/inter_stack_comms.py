#!/usr/bin/env python

import rospy
import math
import sys
import tf2_ros

from cav_msgs.msg import RobotEnabled
from autoware_msgs.msg import VehicleStatus
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import NavSatFix

from std_msgs.msg import Header
from std_msgs.msg import Bool
from std_msgs.msg import Float64
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import Point32
from geometry_msgs.msg import PolygonStamped
from geometry_msgs.msg import Transform
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

laser_pub = rospy.Publisher('/carma_1tenth/scaled/scan', LaserScan, queue_size=1)
pose_pub  = rospy.Publisher('/localization/current_pose', PoseStamped, queue_size=1)
twist_pub = rospy.Publisher('/hardware_interface/vehicle/twist', TwistStamped, queue_size=1)
odom_pub  = rospy.Publisher('/carma_1tenth/base/odom', Odometry, queue_size=1)
tf_pub    = tf2_ros.TransformBroadcaster()
re_pub    = rospy.Publisher('/hardware_interface/controller/robot_status', RobotEnabled, queue_size=1)
vs_pub    = rospy.Publisher('/hardware_interface/vehicle_status', VehicleStatus, queue_size=1)
gps_pub   = rospy.Publisher('/carma_1tenth/gps_pos', NavSatFix, queue_size=1)


def dummy_pub():
    out1 = RobotEnabled()
    out1.robot_enabled = True
    out1.robot_active = True
    out2 = VehicleStatus()
    out2.header.stamp = rospy.Time.now()
    out2.drivemode = 1
    out2.steeringmode = 1
    re_pub.publish(out1)
    vs_pub.publish(out2)


def scale_lidar(data):
    out = LaserScan()
    out.header = data.header
    out.header.frame_id = 'carma_laser'
    out.angle_min = data.angle_min
    out.angle_max = data.angle_max
    out.angle_increment = data.angle_increment
    out.time_increment = data.time_increment
    out.scan_time = data.scan_time
    out.range_min = data.range_min * 10.0
    out.range_max = data.range_max * 10.0
    for p in data.ranges:
        out.ranges.append(p * 10.0)
    laser_pub.publish(out)


def scale_odometry(data):
    odom = Odometry()
    odom.header.frame_id = 'odom'
    odom.child_frame_id = 'carma_base_link'
    odom.header.stamp = rospy.Time.now()
    odom.pose = data.pose
    odom.pose.pose.position.x = (odom.pose.pose.position.x + 12.25) * 10.0
    odom.pose.pose.position.y = (odom.pose.pose.position.y + 0.25) * 10.0
    odom.twist = data.twist
    odom.twist.twist.linear.x = odom.twist.twist.linear.x * 10.0
    odom.twist.twist.linear.y = odom.twist.twist.linear.y * 10.0

    tf = TransformStamped(header=Header(
                          frame_id=odom.header.frame_id,
                          stamp=odom.header.stamp),
                          child_frame_id=odom.child_frame_id,
                          transform=Transform(
                          translation=odom.pose.pose.position,
                          rotation=odom.pose.pose.orientation))

    out1 = PoseStamped()
    out1.header = odom.header
    out1.pose = odom.pose.pose

    out2 = TwistStamped()
    out2.header = odom.header
    out2.twist = odom.twist.twist

    out3 = NavSatFix()
    out3.header = odom.header
    out3.longitude = math.atan2(odom.pose.pose.position.y, odom.pose.pose.position.x)
    out3.latitude = math.acos(odom.pose.pose.position.x/(6.3781 * pow(10, 6) * math.cos(out3.longitude)))

    odom_pub.publish(odom)
    tf_pub.sendTransform(tf)
    pose_pub.publish(out1)
    twist_pub.publish(out2)
    dummy_pub()
    gps_pub.publish(out3)


if __name__ == '__main__':
    try:
        rospy.init_node('scale_lidar_odometry', anonymous=True)
        rospy.Subscriber('/car_1/scan', LaserScan, scale_lidar)
        rospy.Subscriber('/car_1/base/odom', Odometry, scale_odometry)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
