#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf import transformations
import math

# robot state variables
position = Point()
yaw = 0

# machine state
state = 0
# state 0 - rotate to align towards goal
# state 1 - move straight to point
# state 2 - done :)

per_pos_x = 0
x_pre = 0

init_pt = 0
m = 0
# goal
desired_position = Point()
desired_position.x = -3
desired_position.y = 7
desired_position.z = 0

# parameters
yaw_precision = (math.pi / 90)*2 # +/- 4 degree allowed
dist_precision = 0.8

# publishers
pub = None

def change_state(State):
    global state
    state = State
    print 'State changed to [%s]' % state

def clk_pos(msg):
    global position
    # position
    position = msg.pose.pose.position

def clk_yaw(msg):
    global yaw
    quaternion = (
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z,
        msg.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw = euler[2]

def fix_yaw(des_pos):
    global yaw, pub, yaw_precision, state
    desired_yaw = math.atan2(des_pos.y - position.y, des_pos.x - position.x)
    err_yaw = desired_yaw - yaw

    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision:
        twist_msg.angular.z = -0.6 if err_yaw > 0 else 0.3

    pub.publish(twist_msg)

    # state change conditions
    if math.fabs(err_yaw) < yaw_precision:
        print 'Yaw error: [%s]' % err_yaw
        change_state(1)

def go_straight(des_pos):
    global yaw, pub, yaw_precision, state, per_pos_x, init_pt ,m ,x_pre
    desired_yaw = math.atan2(des_pos.y - position.y, des_pos.x - position.x)
    err_yaw = desired_yaw - yaw
    err_pos = math.sqrt(pow(des_pos.y - position.y, 2) + pow(des_pos.x - position.x, 2))
    if (init_pt == 0):
        m = (des_pos.y - position.y)/(des_pos.x - position.x)
        init_pt = 1
    l = 1
    if err_pos > dist_precision:
        if per_pos_x != des_pos.x:
            per_pos_x = math.sqrt(pow(l,2)/(1 + (1/(m*m)))) + position.x
            r = pow(l,2)/(2*(per_pos_x-x_pre))
        twist_msg = Twist()
        twist_msg.linear.x = 0.6
        twist_msg.angular.z = 0.15*(twist_msg.linear.x/r)   # 0.15 half the width of chassis
        pub.publish(twist_msg)
        x_pre = per_pos_x
    else:
        print 'Position error: [%s]' % err_pos
        change_state(2)

    # state change conditions
    #if math.fabs(err_yaw) > yaw_precision:
    #    print 'Yaw error: [%s]' % err_yaw
    #    change_state(0)

def done():
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    pub.publish(twist_msg)


def main():
    global pub

    rospy.init_node('nav_point')

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    sub_odom = rospy.Subscriber('/odom', Odometry, clk_pos)

    sub_imu = rospy.Subscriber('/imu', Imu, clk_yaw)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if state == 0:
            fix_yaw(desired_position)
        elif state == 1:
            go_straight(desired_position)
        elif state == 2:
            done()
            pass
        else:
            rospy.logerr('Unknown state!')
            pass
        rate.sleep()

if __name__ == '__main__':
    main()
