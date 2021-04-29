#!/usr/bin/env python

import rospy
import sys

from std_msgs.msg import Float64
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import Point32
from geometry_msgs.msg import PolygonStamped

global curr_speed
global seq

car_name          = str(sys.argv[1])    # namespace handler (if using multi-vehicle setup)

angle_pub         = rospy.Publisher('/{}/commands/servo/position'.format(car_name), Float64, queue_size = 1)
speed_pub         = rospy.Publisher('/{}/commands/motor/speed'.format(car_name),    Float64, queue_size = 1)
footprint_pub     = rospy.Publisher('/{}/footprint'.format(car_name), PolygonStamped, queue_size = 1)

seq               = 0
footprint         = PolygonStamped()

side_A            = Point32()
side_B            = Point32()
side_C            = Point32()
side_D            = Point32()
side_E            = Point32()

curr_speed        = 0.0                 # store current speed for P-type control
start_flag        = False               # display diagnostics message on console (if enabled)

angle_min_rel     = -100.0              # max left command
angle_max_rel     = 100.0               # max right command
angle_min_abs     = 0.0                 # max left VESC servo
angle_max_abs     = 1.0                 # max right VESC servo

speed_min_rel     = -100.0              # min speed command
speed_max_rel     = 100.0               # max speed command
speed_min_abs     = -20000.0            # min speed VESC motor
speed_max_abs     = 20000.0             # max speed VESC motor
speed_change_step = 2000.0              # acceleration P-type control

[side_A.x, side_A.y, side_A.z] = [-0.1, -0.2,  0.0]
[side_B.x, side_B.y, side_B.z] = [ 0.5, -0.2,  0.0]
[side_C.x, side_C.y, side_C.z] = [ 0.6,  0.0,  0.0]
[side_D.x, side_D.y, side_D.z] = [ 0.5,  0.2,  0.0]
[side_E.x, side_E.y, side_E.z] = [-0.1,  0.2,  0.0]

footprint.header.frame_id      = '{}_base_link'.format(car_name)
footprint.polygon.points       = [side_A, side_B, side_C, side_D, side_E]

def footprint_visualizer():
    global seq
    footprint.header.seq = seq
    seq = seq + 1
    footprint.header.stamp = rospy.Time.now()
    footprint_pub.publish(footprint)

def output_angle_mixer(rel_angle):
    output_angle = (rel_angle - angle_min_rel)/(angle_max_rel - angle_min_rel)
    output_angle = output_angle * (angle_max_abs - angle_min_abs)
    return output_angle

def output_speed_mixer(rel_speed):
    global curr_speed
    output_speed = (rel_speed - speed_min_rel)/(speed_max_rel - speed_min_rel)
    output_speed = output_speed * (speed_max_abs - speed_min_abs) - speed_max_abs
    if output_speed >= curr_speed + speed_change_step:
        curr_speed = curr_speed + speed_change_step
    elif output_speed <= curr_speed - speed_change_step:
        curr_speed = curr_speed - speed_change_step
    if abs(curr_speed) < speed_change_step:
        curr_speed = 0.0
    return curr_speed

def command_callback(data):
    angle_req = Float64()
    speed_req = Float64()
    angle_req.data = output_angle_mixer(data.steering_angle)
    speed_req.data = output_speed_mixer(data.speed)
    angle_pub.publish(angle_req)
    speed_pub.publish(speed_req)
    footprint_visualizer()

if __name__ == '__main__':
    try:
        rospy.init_node('move_base', anonymous = True)
        rospy.Subscriber('/{}/multiplexer/command'.format(car_name), AckermannDrive, command_callback)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
