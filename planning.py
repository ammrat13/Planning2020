#!/usr/bin/env python3
"""
File:          planning.py
Author:        Binit Shah
Last Modified: Ammar on 3/6
"""

from math import sin, cos, pi, atan2
from collections import deque


# Utility functions
UTIL_SIGN = lambda x: x and (1, -1)[x < 0]

# Planning constants for the robot
R = .045
L = .1
D = .2427

V_SCALE = 2
OMEGA_MAX_ALLOWED = 5
GOAL_REACHED_MARGIN_SQUARED = 0.0508**2

# Enum constants
DIRECTION_FORWARD = 0
DIRECTION_REVERSE = 1

# A queue for all the waypoints
wp_queue = deque([(1,0,DIRECTION_FORWARD)])


def order_blocks(block_config):
    # TODO: Actually compute and return the ordering
    return block_config


def reached(cur, goal):
    # Error is simply x**2 + y**2
    # We don't particularly care about theta
    return GOAL_REACHED_MARGIN_SQUARED >= \
        (goal[0]-cur[0])**2 + (goal[1]-cur[1])**2


# Utility function to convert between domains
def xydot_to_w(v, curT, dire):

    # Component extraction
    xDotTarg, yDotTarg = v

    # Compute matrix coefficients
    xDotC0 = R/2 * cos(curT) + R*L/D * sin(curT)
    xDotC1 = R/2 * cos(curT) - R*L/D * sin(curT)
    yDotC0 = R/2 * sin(curT) - R*L/D * cos(curT)
    yDotC1 = R/2 * sin(curT) + R*L/D * cos(curT)

    # Invert the matrix
    matDetInv = 1 / (xDotC0*yDotC1 - xDotC1*yDotC0)
    wR = matDetInv * ( yDotC1 * xDotTarg - xDotC1 * yDotTarg)
    wL = matDetInv * (-yDotC0 * xDotTarg + xDotC0 * yDotTarg)

    # Normalize omegas
    wMax = max(abs(wR), abs(wL))
    wR *= OMEGA_MAX_ALLOWED / max(wMax, 1)
    wL *= OMEGA_MAX_ALLOWED / max(wMax, 1)

    return (wR, wL) if dire == DIRECTION_FORWARD else (-wL, -wR)

# Utility function just to turn us around
def match_pose_to_dir(v, dire):
    # If the direction doesn't need to be changed, don't
    if dire == DIRECTION_FORWARD:
        return v
    # Otherwise, move the control point and turn 180 degrees
    else:
        x, y, t = v
        return (x - 2*L*cos(t), y - 2*L*sin(t), (t + pi) % (2*pi))


# Does exactly what it says
# Depends on the next waypoint
def compute_wheel_velocities(cur):
    global wp_queue
    try:
        wp = wp_queue[0]
        if reached(cur, wp):
            wp_queue.popleft()
            return (0,0)
        else:
            d = (V_SCALE * (wp[0]-cur[0]), V_SCALE * (wp[1]-cur[1]))
            return xydot_to_w(d, cur[2], wp[2])
    except IndexError:
        return (0,0)
