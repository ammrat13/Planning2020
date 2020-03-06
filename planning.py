#!/usr/bin/env python3
"""
File:          planning.py
Author:        Binit Shah
Last Modified: Rishabh on 3/6
"""

from math import sin, cos, pi, atan2, sqrt

# Utility functions
UTIL_SIGN = lambda x: x and (1, -1)[x < 0]

# Planning constants for the robot
R = .045
L = .1
D = .2427

GOAL_MARGIN_X = .0508
CENTER_MARGIN_Y = .2038
GOAL_REACHED_MARGIN = 0.0508
GOAL_THETA_MARGIN = pi / 12

DIRECTION_FORWARD = 0
DIRECTION_REVERSE = 1


def order_blocks(block_config):
    return block_config


def reached(current_pose, goal_pos):
    distance = sqrt((goal_pos[0] - current_pose[0]) ** 2 + (goal_pos[1] - current_pose[1]) ** 2)
    theta_difference = (goal_pos[2] - current_pose[2]) % (2*pi)
    if theta_difference > pi:
        theta_difference = abs(theta_difference - 2*pi)
    if distance <= GOAL_REACHED_MARGIN and theta_difference <= GOAL_THETA_MARGIN:
        return True
    return False


# Utility function to convert between domains
def xydot_to_w(v, currentT, direction):

    xDot, yDot, tDot = v

    sign = (-1)**direction

    xDotC0 = R/2 * cos(currentT) - sign*R*L/D * sin(currentT)
    xDotC1 = R/2 * cos(currentT) + sign*R*L/D * sin(currentT)
    yDotC0 = R/2 * sin(currentT) + sign*R*L/D * cos(currentT)
    yDotC1 = R/2 * sin(currentT) - sign*R*L/D * cos(currentT)

    matDetInv = 1 / (xDotC0*yDotC1 - xDotC1*yDotC0)

    wR = matDetInv * ( yDotC1 * xDotTarg - xDotC1 * yDotTarg)
    wL = matDetInv * (-yDotC0 * xDotTarg + xDotC0 * yDotTarg)

    return (wR, wL)

# Utility function just to turn us around
def turn_around_pose(v):
    x, y, t = v
    return (x - 2*L*cos(t), y - 2*L*sin(t), (t + pi) % (2*pi))


# Does exactly what it says
# Depends on the current state of the robot for navigation
def compute_wheel_velocities(current_pose, goal_pos):

    straight_waypoint = None
    drive_direction = DIRECTION_FORWARD

    # Three conditions to computing `straight_waypoint`
    # If we are within a certain radius x-wise of our goal, drive directly
    #   toward it
    if abs(goal_pose[0] - current_pose[0]) <= GOAL_MARGIN_X:
        straight_waypoint = (goal_pose[0], goal_pose[1])

    # Othwerwise, if we are within a certain distance of the centerline, go to
    #   the goal projected onto the centerline
    elif abs(current_pose[1]) <= CENTER_MARGIN_Y:
        straight_waypoint = (goal_pose[0], 0)

    # Otherwise, reverse to the centerline
    else:
        straight_waypoint = (current_pose[0], -UTIL_SIGN(current_pose[1]) * 10)
        drive_direction = DIRECTION_REVERSE

    # Determine the control point
    currentL = L if drive_direction == 0 else -L
    control_point = (current_pose[0] + currentL*cos(current_pose[2]), current_pose[1] + currentL*sin(current_pose[2]))

    # Calculate xDot, yDot, tDot
    xDot = straight_waypoint[0] - control_point[0]
    yDot = straight_waypoint[1] - control_point[1]
    tDot = (atan2(yDot, xDot) - current_pose[2]) % (2*pi)
    v = (xDot, yDot, tDot)

    # Convert to wheel velocities
    wR, wL = xydot_to_w(v, current_pose[2], drive_direction)

    # Normalize wheel velocities and return
    wMax = max(wR, wL)
    wR /= max(wMax, 1)
    wL /= max(wMax, 1)
    return wR, wL
