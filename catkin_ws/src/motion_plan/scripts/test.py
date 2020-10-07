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
from collections import deque

# robot state variable [ pos x (m),pos y(m), yaw(rad), vel(m/s), omega(rad/sec)]
state_v = [0.0 ,0.0 ,0.0 ,0.0 ,0.0]

# Goal
goal = [10.0,10.0]
# publishers
pub = None

laser = np.empty(720,dtype = float)
print('dim of laser:',len(laser))

angles = np.arange(-1.5707999467849731,1.5707999467849731,0.004369401838630438)
print('dim of angles:',len(angles))

r = 0

class param:
#parameters
    def __init__(self):
        self.max_speed = 0.3  # [m/s]
        self.min_speed = -0.2  # [m/s]
        self.max_accel = 5
        self.max_dyawrate = 5
        self.max_yawrate = 0.3  # [rad/s]
        self.v_reso = 0.02  # [m/s]
        self.yawrate_reso = 0.02 # [rad/s]
        self.dt = 0.1  # [s] Time tick for motion prediction
        self.predict_time = 5.0  # [s]
        self.to_goal_cost_gain = 0.1
        self.speed_cost_gain = 0.8
        self.obstacle_cost_gain = 0.1

# sensor call backs
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
    #print('yaw is',state_v[2])

def clbk_laser(msg):
	global laser,r
	if(r == 0):
		print('angle_min :',msg.angle_min)
		print('angle_max :',msg.angle_max)
		print('angle_increment :',msg.angle_increment)
		print('range_max :',msg.range_max)
		print('#####################################')
	laser = msg.ranges
	r = 1

#################################
def plot_scan(trajectory,laser,angles):
    ranges = np.asarray(laser)
    ranges[ranges > 10.0] = 10.0
    ox = np.sin(angles) * ranges[0:719]
    oy = np.cos(angles) * ranges[0:719]
    plt.cla()
    plt.gcf().canvas.mpl_connect('key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])
    plt.scatter(ox,oy,s=0.1)
    plt.plot(trajectory[:, 0], trajectory[:, 1], "-g")
    plt.axis("equal")
    plt.grid(True)
    plt.pause(0.0001)

#################################
#Dynamic window functions

# robot dynamics function
def robot_dynamics(x,u,dt):
# motion model
    x[2] += u[1] * dt
    x[1] += u[0] * math.cos(x[2]) * dt
    x[0] += u[0] * math.sin(x[2]) * dt
    x[3] = u[0]
    x[4] = u[1]
    return x

def predict_trajectory(x_init, v, o, param):
# predict trajectory
    x = np.array(x_init)
    x_dummy = np.array([0.0,0.0,0.0,x_init[3],x_init[4]])
    traj = np.array(x)
    traj_dummy = np.array(x_dummy)
    time = 0
    while time <= param.predict_time:
        x = robot_dynamics(x, [v, o], param.dt)
        x_dummy = robot_dynamics(x_dummy, [v, o], param.dt)
        traj = np.vstack((traj, x))
        traj_dummy = np.vstack((traj_dummy, x_dummy))
        time += param.dt
    return traj_dummy , traj;

def calc_obstacle_cost(trajectory,laser,angles):
# calculate obstacle cost
    ranges = np.asarray(laser)
    ranges[ranges > 10.0] = 10.0
    ox = np.sin(angles) * ranges[0:719]
    oy = np.cos(angles) * ranges[0:719]
    dx = trajectory[:, 0] - ox[:, None]
    dy = trajectory[:, 1] - oy[:, None]
    r = np.hypot(dx, dy)
    min_r = np.min(r)
    #print('obstacle cost : ',1.0/min_r)
    return 1.0 / min_r  # OK

def calc_to_goal_cost(trajectory, goal,x_init):
# calculate goal cost
# coordinate teansform from robot frame tp global frame
    theta = x_init[2]
    #x_mobile = np.cos(theta)*trajectory[-1, 0] - np.sin(theta)*trajectory[-1, 1]
    #y_mobile = np.sin(theta)*trajectory[-1, 0] + np.cos(theta)*trajectory[-1, 1]
    dx = goal[0] - trajectory[-1, 0] #(x_mobile + x_init[0])
    dy = goal[1] - trajectory[-1, 1] #(y_mobile + x_init[1])
    #yaw_mobile = theta + trajectory[-1, 2]
    error_angle = math.atan2(dy, dx)
    cost_angle = error_angle - trajectory[-1,2]#yaw_mobile
    cost = abs(math.atan2(math.sin(cost_angle), math.cos(cost_angle)))
    #print('goal_cost : ',cost)
    return cost

def calc_dynamic_window(x, param):
#   Dynamic window from robot specification
    global state_v,goal
    Vs = [param.min_speed, param.max_speed,
          -param.max_yawrate, param.max_yawrate]

    # Dynamic window from motion model
    Vd = [x[3] - param.max_accel * param.dt,
          x[3] + param.max_accel * param.dt,
          x[4] - param.max_dyawrate * param.dt,
          x[4] + param.max_dyawrate * param.dt]

    #  [vmin, vmax, yaw_rate min, yaw_rate max]
    dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
          max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]
    dist_to_goal = math.hypot(state_v[0] - goal[0], state_v[1] - goal[1])
    if dist_to_goal <= 3:
        dw = [max(Vs[0], Vd[0])/2, min(Vs[1], Vd[1])/2,max(Vs[2], Vd[2])/2, min(Vs[3], Vd[3])/2]
    print(dw)
    return dw

def calc_control_and_trajectory(x,dw, param, goal,laser,angles):
#   calculation final input with dynamic window
    x_init = x[:]
    min_cost = float("inf")
    best_u = [0.0, 0.0]
    best_trajectory = np.array([x])
    min_obs_cost = 0
    min_goal_cost = 0
    x_for_prediction = [0.0,0.0,0.0,x_init[3],x_init[4]] # moving to robot frame

    # evaluate all trajectory with sampled input in dynamic window
    for v in np.arange(dw[0], dw[1], param.v_reso):
        for o in np.arange(dw[2], dw[3], param.yawrate_reso):

            #trajectory_dummy = predict_trajectory(x_for_prediction, v, o, param)
            trajectory_dummy,trajectory_real = predict_trajectory(x_init, v, o, param)

            # calc cost
            to_goal_cost = param.to_goal_cost_gain * calc_to_goal_cost(trajectory_real,goal,x_init)
            speed_cost = param.speed_cost_gain * (param.max_speed - trajectory_real[-1, 3])
            ob_cost = param.obstacle_cost_gain * calc_obstacle_cost(trajectory_dummy,laser,angles)

            final_cost =  speed_cost + ob_cost + to_goal_cost

            # search minimum trajectory
            if min_cost >= final_cost:
                min_obs_cost = ob_cost
                min_goal_cost = to_goal_cost
                min_cost = final_cost
                best_u = [v, o]
                best_trajectory = trajectory_dummy
    print('best predicted velocity:',best_u)
    print('obstacle cost:',min_obs_cost)
    print('goal cost:',min_goal_cost)
    return best_u, best_trajectory

def dwa_control(x, param, goal,laser,angles):
# DWA control
    global state_v, pub
    dw = calc_dynamic_window(x, param)
    u, trajectory = calc_control_and_trajectory(x,dw, param, goal,laser,angles)

    twist_msg = Twist()
    twist_msg.linear.x = u[0]
    twist_msg.angular.z = u[1]
    pub.publish(twist_msg)

    plot_scan(trajectory,laser,angles)

    print('/////////')
    return trajectory

def main():
    global state_v,laser,angles,pub
    rospy.init_node('nav_point')
    Param = param()

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    sub_odom = rospy.Subscriber('/odom', Odometry, clk_pos)
    sub_imu = rospy.Subscriber('/imu', Imu, clk_yaw)
    sub = rospy.Subscriber('/i214/laser/scan', LaserScan, clbk_laser)
    rate = rospy.Rate(10)

    i = 1

    while not rospy.is_shutdown():
        dist_to_goal = math.hypot(state_v[0] - goal[0], state_v[1] - goal[1])
        #print('distance to goal :',dist_to_goal)
        if dist_to_goal <= 1:
            if(i ==1):
                print('reached :)')
                i = 0
            twist_msg = Twist()
            twist_msg.linear.x = 0
            twist_msg.angular.z = 0
            pub.publish(twist_msg)

        else:
            predicted_trajectory = dwa_control(state_v,Param, goal,laser,angles)
            #map(predicted_trajectory,laser,angles)
            rate.sleep()
    plt.show()

if __name__ == '__main__':
    main()


###################################
