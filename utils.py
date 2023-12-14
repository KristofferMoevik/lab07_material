import numpy as np
from numpy import linalg
from numpy.linalg import norm
import matplotlib.pyplot as plt

import math

def pi2pi(angle):
    """
    Normalize angles between [-pi, pi)

    Returns:
        y : normalized angle
    """
    angle = angle % (2 * np.pi)  # force in range [0, 2 pi)
    if angle > np.pi:  # move to [-pi, pi)
        angle -= 2 * np.pi
    return angle

class RobotStates:
    '''
    Class to store robot states (x, y, yaw, v, t)
    '''
    def __init__(self):
        self.x = []
        self.y = []
        self.yaw = []
        self.v = []
        self.t = []
        self.pind = []
        self.a = []

    def append(self, t, pind, state, a):
        '''
        Append a state to the list of states

        Parameters
        ----------
        t : float
            time
        pind : int
            index of the waypoint
        state : object
            state of the robot
        a : float
            acceleration
        '''
        self.x.append(state.x.flatten()[0])
        self.y.append(state.x.flatten()[1])
        self.yaw.append(state.x.flatten()[2])
        self.v.append(state.u.flatten()[0])
        self.a.append(a)
        self.t.append(t)
        self.pind.append(pind)

    def __len__(self):
        return len(self.x)

def interpolate_waypoints(waypoints, resolution=0.01):
    """
    Interpolate the waypoints to add more points along the path

    Args:
    waypoints: array of waypoints
    resolution: distance between two interpolated points

    Return:
    interpolated_waypoints: array of interpolated waypoints
    """
    interpolated_waypoints = []
    for i in range(len(waypoints) - 1):
        p1 = waypoints[i]
        p2 = waypoints[i + 1]
        dist = np.linalg.norm(p2 - p1)
        n_points = int(dist / resolution)
        x = np.linspace(p1[0], p2[0], n_points)
        y = np.linspace(p1[1], p2[1], n_points)
        interpolated_waypoints += list(zip(x, y))
    return np.array(interpolated_waypoints)

class turtlebot_C:
    # vehicle config
    RF = 0.178/2    # [m] distance from rear to vehicle front end of vehicle
    RB = 0.178/2    # [m] distance from rear to vehicle back end of vehicle
    W = 0.178       # [m] width of vehicle
    WD = 0.160      # [m] distance between left-right wheels
    TR = 0.033      # [m] Tyre radius
    TW = 0.018      # [m] Tyre width


def draw_turtlebot(x, y, yaw, C, color='black', ax=None):
    '''
    Draw a turtlebot with a given position and orientation
    
    Parameters
    ----------
    x : float
        x position of the turtlebot
    y : float
        y position of the turtlebot
    yaw : float
        yaw of the turtlebot
    C : object
        turtlebot constants
    color : str
        color of the turtlebot
    '''
    turtlebot = np.array([[  -C.RB,    -C.RB,     C.RF, 3/2 *C.RF,    C.RF, -C.RB],
                          [C.W / 2, -C.W / 2, -C.W / 2,       0.0, C.W / 2, C.W / 2]])

    wheel = np.array([[-C.TR, -C.TR, C.TR, C.TR, -C.TR],
                      [C.TW / 4, -C.TW / 4, -C.TW / 4, C.TW / 4, C.TW / 4]])

    rWheel = wheel.copy()
    lWheel = wheel.copy()

    Rot1 = np.array([[math.cos(yaw), -math.sin(yaw)],
                     [math.sin(yaw), math.cos(yaw)]])

    rWheel[1, :] -= C.WD / 2
    lWheel[1, :] += C.WD / 2

    rWheel = np.dot(Rot1, rWheel)
    lWheel = np.dot(Rot1, lWheel)
    turtlebot = np.dot(Rot1, turtlebot)

   
    rWheel += np.array([[x], [y]])
    lWheel += np.array([[x], [y]])
    turtlebot += np.array([[x], [y]])

    return turtlebot, rWheel, lWheel




