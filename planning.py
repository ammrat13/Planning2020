#!/usr/bin/env python3
"""
File:          planning.py
Author:        Binit Shah
Last Modified: Ammar on 3/6
"""

from math import sin, cos, pi, atan2

# Utility functions
UTIL_SIGN = lambda x: x and (1, -1)[x < 0]

# Planning constants for the robot
R = .045
L = .05
D = .2427

V_SCALE = 10
OMEGA_MAX_ALLOWED = 5

CENTER_MARGIN_Y = .2038
GOAL_MARGIN_LOCK_X = .01
GOAL_MARGIN_UNLOCK_X = .0508
GOAL_REACHED_MARGIN_SQUARED = 0.0508**2

# Enum constants
DIRECTION_FORWARD = 0
DIRECTION_REVERSE = 1

# State for whether or not we have locked onto a bin
locked = False

def order_blocks(block_config):
    # TODO: Actually compute and return the ordering
    return block_config


def reached(current_pose, goal_pose):
    # Error is simply x**2 + y**2
    # We don't particularly care about theta
    return GOAL_REACHED_MARGIN_SQUARED >= \
        (goal_pose[0]-current_pose[0])**2 + (goal_pose[1]-current_pose[1])**2


# Utility function to convert between domains
def xydot_to_w(v, currentT, direction):

    # Component extraction
    xDotTarg, yDotTarg = v

    # Compute matrix coefficients
    xDotC0 = R/2 * cos(currentT) + R*L/D * sin(currentT)
    xDotC1 = R/2 * cos(currentT) - R*L/D * sin(currentT)
    yDotC0 = R/2 * sin(currentT) - R*L/D * cos(currentT)
    yDotC1 = R/2 * sin(currentT) + R*L/D * cos(currentT)

    # Invert the matrix
    matDetInv = 1 / (xDotC0*yDotC1 - xDotC1*yDotC0)
    wR = matDetInv * ( yDotC1 * xDotTarg - xDotC1 * yDotTarg)
    wL = matDetInv * (-yDotC0 * xDotTarg + xDotC0 * yDotTarg)

    # Normalize omegas
    wMax = max(abs(wR), abs(wL))
    wR *= OMEGA_MAX_ALLOWED / max(wMax, 1)
    wL *= OMEGA_MAX_ALLOWED / max(wMax, 1)

    return (wR, wL) if direction == DIRECTION_FORWARD else (-wL, -wR)

# Utility function just to turn us around
def match_pose_to_dir(v, direction):
    # If the direction doesn't need to be changed, don't
    if direction == DIRECTION_FORWARD:
        return v
    # Otherwise, move the control point and turn 180 degrees
    else:
        x, y, t = v
        return (x - 2*L*cos(t), y - 2*L*sin(t), (t + pi) % (2*pi))


# Does exactly what it says
# Depends on the current state of the robot for navigation
def compute_wheel_velocities(current_pose, goal_pose):
    global locked

    straight_waypoint = None
    drive_direction = DIRECTION_FORWARD

    # Keep track of whether we have locked onto a bin
    # Once we have locked on, we have to move farther to shake off the lock
    goal_margin = GOAL_MARGIN_UNLOCK_X if locked else GOAL_MARGIN_LOCK_X

    # Three conditions to computing `straight_waypoint`
    # If we are within a certain radius x-wise of our goal, drive directly
    #   toward it
    if abs(goal_pose[0] - current_pose[0]) <= goal_margin:
        straight_waypoint = (goal_pose[0], goal_pose[1])
        locked = True
    # Othwerwise, if we are within a certain distance of the centerline, go to
    #   the goal projected onto the centerline
    elif abs(current_pose[1]) <= CENTER_MARGIN_Y:
        straight_waypoint = (goal_pose[0], 0)
        locked = False
    # Otherwise, reverse to the centerline
    else:
        straight_waypoint = (current_pose[0], -UTIL_SIGN(current_pose[1]) * 10)
        drive_direction = DIRECTION_REVERSE
        locked = False

    # Reverse the control point if needed
    current_pose = match_pose_to_dir(current_pose, drive_direction)

    # Calculate target xDot, yDot
    xDotTarg = V_SCALE * (straight_waypoint[0] - current_pose[0])
    yDotTarg = V_SCALE * (straight_waypoint[1] - current_pose[1])

    # Convert to wheel velocities
    # Return because the method will normalize
    return xydot_to_w((xDotTarg, yDotTarg), current_pose[2], drive_direction)
