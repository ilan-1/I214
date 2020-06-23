#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf import transformations
import math
import numpy as np
import matplotlib.pyplot as plt

# robot state variable [ pos x (m),pos y(m), yaw(rad), vel(m/s), omega(rad/sec)]
state_v = [0.0 ,0.0 ,0.0 ,0.0 ,0.0]
# Goal
goal = [10.0,10.0]
# publishers
pub = None
# machine state
state = 0
# obstacle coordinate list
ob = np.array([[0.0,0.0],[0.0,0.0],[0.0,0.0],[0.0,0.0],[0.0,0.0]])

i=0

class param:
#parameters
    def __init__(self):
        self.yaw_precision = (math.pi / 90)*2 # +/- 4 degree allowed
        self.dist_precision = 0.8
        self.max_speed = 0.4  # [m/s]
        self.min_speed = -0.3  # [m/s]
        self.max_accel = 5
        self.max_dyawrate = 1000.0 * math.pi / 180.0
        self.max_yawrate = 40.0 * math.pi / 180.0  # [rad/s]
        self.v_reso = 0.2  # [m/s]
        self.yawrate_reso = 4 * math.pi / 180.0  # [rad/s]
        self.dt = 0.1  # [s] Time tick for motion prediction
        self.predict_time = 7.0  # [s]
        self.to_goal_cost_gain = 0.15
        self.speed_cost_gain = 0.4
        self.obstacle_cost_gain = 1.0
        self.robot_radius = 2.0
        self.robot_width = 0.5  # [m] for collision check
        self.robot_length = 1.2  # [m] for collision check

def change_state(State):
    global state
    state = State
    print 'State changed to [%s]' % state

def clk_pos(msg):
    global state_v
    # position
    state_v[0] = msg.pose.pose.position.x
    state_v[1] = msg.pose.pose.position.y
    x = msg.twist.twist.linear.x
    y = msg.twist.twist.linear.y
    state_v[4] = msg.twist.twist.angular.z
    state_v[3] = np.power((x*x+y*y),0.5)

def clk_yaw(msg):
    global state_v
    quaternion = (
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z,
        msg.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    state_v[2] = euler[2]
    #rospy.loginfo(state_v[2])

def fix_yaw(goal,param):
    global state_v, pub, state
    desired_yaw = math.atan2(goal[1] - state_v[1], goal[0] - state_v[0])
    err_yaw = desired_yaw - state_v[2]

    twist_msg = Twist()
    if math.fabs(err_yaw) > param.yaw_precision:
        twist_msg.angular.z = -0.6 if err_yaw > 0 else 0.3

    pub.publish(twist_msg)

    # state change conditions
    if math.fabs(err_yaw) < param.yaw_precision:
        print 'Yaw error: [%s]' % err_yaw
        change_state(1)

def find_obs(msg):
    global state_v, ob
    ob = np.array([[np.inf,np.inf],[np.inf,np.inf],[np.inf,np.inf],[np.inf,np.inf],[np.inf,np.inf]])
    region = {'1':min(min(msg.ranges[:119]), 10.0),
                '2':min(min(msg.ranges[120:240]), 10.0),
                '3':min(min(msg.ranges[241:360]), 10.0),
                '4':min(min(msg.ranges[361:480]), 10.0),
                '5':min(min(msg.ranges[481:600]), 10.0),
                '6':min(min(msg.ranges[601:713]), 10.0),
                '7':min(min(msg.ranges[340:380]), 10.0)}
    if (region['1'] < 10):
        ob_x = state_v[0] + region['1']*np.cos(np.pi/12.0 - state_v[2])
        ob_y = state_v[1] + region['1']*np.sin(np.pi/12.0- state_v[2])
        ob[0][0] = ob_x
        ob[0][1] = ob_y
    '''if (region['2'] < 10):
        ob_x = state_v[0] + region['2']*np.cos(np.pi/8.0- state_v[2])
        ob_y = state_v[1] + region['2']*np.sin(np.pi/8.0- state_v[2])
        ob = np.append(ob , np.array([[ob_x,ob_y]]),axis=0)'''
    if (region['3'] < 10):
        ob_x = state_v[0] + region['3']*np.cos(np.pi/2.4- state_v[2])
        ob_y = state_v[1] + region['3']*np.sin(np.pi/2.4- state_v[2])
        ob[1][0] = ob_x
        ob[1][1] = ob_y

    if (region['4'] < 10):
        ob_x = state_v[0] + region['4']*np.cos(np.pi/1.7142- state_v[2])
        ob_y = state_v[1] + region['4']*np.sin(np.pi/1.7142- state_v[2])
        ob[2][0] = ob_x
        ob[2][1] = ob_y

    '''if (region['5'] < 10):
            ob_x = state_v[0] + region['5']*np.cos(np.pi/1.3333- state_v[2])
            ob_y = state_v[1] + region['5']*np.sin(np.pi/1.3333- state_v[2])
            ob = np.append(ob , np.array([[ob_x,ob_y]]),axis=0)'''
    if (region['6'] < 10):
        ob_x = state_v[0] + region['6']*np.cos(np.pi/1.0909- state_v[2])
        ob_y = state_v[1] + region['6']*np.sin(np.pi/1.0909- state_v[2])
        ob[3][0] = ob_x
        ob[3][1] = ob_y
    if (region['7'] < 10):
        ob_x = state_v[0] + region['7']*np.cos(np.pi/2.0- state_v[2])
        ob_y = state_v[1] + region['7']*np.sin(np.pi/2.0- state_v[2])
        ob[4][0] = ob_x
        ob[4][1] = ob_y


def motion(x, u, dt):
# motion model
    x[2] += u[1] * dt
    x[0] += u[0] * math.cos(x[2]) * dt
    x[1] += u[0] * math.sin(x[2]) * dt
    x[3] = u[0]
    x[4] = u[1]

    return x

def predict_trajectory(x_init, v, o, param):
# predict trajectory
    x = np.array(x_init)
    traj = np.array(x)
    time = 0
    while time <= param.predict_time:
        x = motion(x, [v, o], param.dt)
        traj = np.vstack((traj, x))
        time += param.dt

    return traj

def calc_obstacle_cost(trajectory, ob):
# calculate obstacle cost
    if ob.size != 0:
        ox = ob[:, 0]
        oy = ob[:, 1]
        dx = trajectory[:, 0] - ox[:, None]
        dy = trajectory[:, 1] - oy[:, None]
        r = np.hypot(dx, dy)
        min_r = np.min(r)
        return 1.0 / min_r  # OK
    else :
        return 0.0

def calc_to_goal_cost(trajectory, goal):
# calculate goal cost
    dx = goal[0] - trajectory[-1, 0]
    dy = goal[1] - trajectory[-1, 1]
    error_angle = math.atan2(dy, dx)
    cost_angle = error_angle - trajectory[-1, 2]
    cost = abs(math.atan2(math.sin(cost_angle), math.cos(cost_angle)))

    return cost

def calc_control_and_trajectory(x,dw, param, goal, ob):
#  calculation final input with dynamic window
    x_init = x[:]
    min_cost = float("inf")
    best_u = [0.0, 0.0]
    best_trajectory = np.array([x])

    # evaluate all trajectory with sampled input in dynamic window
    for v in np.arange(dw[0], dw[1], param.v_reso):
        for o in np.arange(dw[2], dw[3], param.yawrate_reso):

            trajectory = predict_trajectory(x_init, v, o, param)

            # calc cost
            to_goal_cost = param.to_goal_cost_gain * calc_to_goal_cost(trajectory, goal)
            speed_cost = param.speed_cost_gain * (param.max_speed - trajectory[-1, 3])
            ob_cost = param.obstacle_cost_gain * calc_obstacle_cost(trajectory, ob)

            final_cost = to_goal_cost + speed_cost + ob_cost


            # search minimum trajectory
            if min_cost >= final_cost:
                min_cost = final_cost
                best_u = [v, o]
                best_trajectory = trajectory
    print(best_u)
    return best_u, best_trajectory

def calc_dynamic_window(x, config):
    """
    calculation dynamic window based on current state x
    """

    # Dynamic window from robot specification
    Vs = [config.min_speed, config.max_speed,
          -config.max_yawrate, config.max_yawrate]

    # Dynamic window from motion model
    Vd = [x[3] - config.max_accel * config.dt,
          x[3] + config.max_accel * config.dt,
          x[4] - config.max_dyawrate * config.dt,
          x[4] + config.max_dyawrate * config.dt]

    #  [vmin, vmax, yaw_rate min, yaw_rate max]
    dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
          max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]
    print(dw)
    return dw


def dwa_control(x, param, goal, ob):
# DWA control
    global state , pub
    dw = calc_dynamic_window(x, param)
    u, trajectory = calc_control_and_trajectory(x,dw, param, goal, ob)

    twist_msg = Twist()
    twist_msg.linear.x = u[0]
    twist_msg.angular.z = -u[1]
    rospy.loginfo(twist_msg)
    print('/////////')
    pub.publish(twist_msg)
    #x[0] = state_v[0]
    #x[1] = state_v[1]
    #x[2] = state_v[2]
    #state_v[3] = u[0]
    #state_v[4] = u[1]
    plot(trajectory,x,ob,goal,param)

    dist_to_goal = math.hypot(state_v[0] - goal[0], state_v[1] - goal[1])
    if dist_to_goal <= param.robot_radius:
            change_state(2)

    desired_yaw = math.atan2(goal[1] - x[1], goal[0] - x[0])
    err_yaw = desired_yaw - x[2]

    if math.fabs(err_yaw) > (param.yaw_precision*50):
        print 'Yaw error: [%s]' % err_yaw
        change_state(0)

def done():
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    pub.publish(twist_msg)
    print("Goal reached :) !!!")

def plot(predicted_trajectory,x,ob,goal,param):
    plt.cla()
    # for stopping simulation with the esc key.
    plt.gcf().canvas.mpl_connect('key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])
    plt.plot(predicted_trajectory[:, 0], predicted_trajectory[:, 1], "-g")
    plt.plot(x[0], x[1], "xr")
    plt.plot(goal[0], goal[1], "xb")
    if ob.size != 0:
        plt.plot(ob[:, 0], ob[:, 1], "ok")
    plot_robot(x[0], x[1], x[2], param)
    plt.axis("equal")
    plt.grid(True)
    plt.pause(0.0001)

def plot_robot(x, y, yaw, param):  # pragma: no cover

    outline = np.array([[-param.robot_length / 2, param.robot_length / 2,
                             (param.robot_length / 2), -param.robot_length / 2,
                             -param.robot_length / 2],
                            [param.robot_width / 2, param.robot_width / 2,
                             - param.robot_width / 2, -param.robot_width / 2,
                             param.robot_width / 2]])
    Rot1 = np.array([[math.cos(yaw), math.sin(yaw)],
                         [-math.sin(yaw), math.cos(yaw)]])
    outline = (outline.T.dot(Rot1)).T
    outline[0, :] += x
    outline[1, :] += y
    plt.plot(np.array(outline[0, :]).flatten(),np.array(outline[1, :]).flatten(), "-k")

def main():
    # initial state [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
    global state_v ,pub ,goal
    Param = param()
    trajectory = np.array(state_v)

    rospy.init_node('nav_point')

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    sub_odom = rospy.Subscriber('/odom', Odometry, clk_pos)

    sub_imu = rospy.Subscriber('/imu', Imu, clk_yaw)

    sub = rospy.Subscriber('/i214/laser/scan', LaserScan, find_obs)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if state == 0:
            fix_yaw(goal,Param)
        elif state == 1:
            dwa_control(state_v,Param, goal, ob)
        elif state == 2:
            done()
            pass
        else:
            rospy.logerr('Unknown state!')
            pass
        rate.sleep()
        print("---------------------")
    plt.show()

if __name__ == '__main__':
    main()
