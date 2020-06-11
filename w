import math
import matplotlib.pyplot as plt
import numpy as np

show_animation = True

def plot_robot(x,y,yaw):
    outline = np.array([[- 1.2 / 2, 1.2 / 2,(1.2 / 2), -1.2 / 2,-1.2 / 2],[o.5 / 2, o.5 / 2, - o.5 / 2, -o.5 / 2,o.5 / 2]])
    Rot1 = np.array([[math.cos(yaw), math.sin(yaw)],[-math.sin(yaw), math.cos(yaw)]])
    outline = (outline.T.dot(Rot1)).T
    outline[0, :] += x
    outline[1, :] += y
    plt.plot(np.array(outline[0, :]).flatten(),np.array(outline[1, :]).flatten(), "-k")

def plot_arrow(x, y, yaw, length=0.5, width=0.1):
    plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),head_length=width, head_width=width)
    plt.plot(x, y)

def temp_goal(x,y,goal):
    x_temp =
