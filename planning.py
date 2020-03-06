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
L = .1
D = .2427

OMEGA_MAX_ALLOWED = 5

GOAL_MARGIN_X = .0508
CENTER_MARGIN_Y = .2038
GOAL_REACHED_MARGIN_SQUARED = 0.0508**2

# Enum constants
DIRECTION_FORWARD = 0
DIRECTION_REVERSE = 1


def order_blocks(block_config):
    # TODO: Actually compute and return the ordering
    return block_config


def reached(current_pose, goal_pos):
    # Error is simply x**2 + y**2
    # We don't particularly care about theta
    return GOAL_REACHED_MARGIN_SQUARED >= 
        (goal_pos[0]-current_pose[0])**2 + (goal_pos[1]-current_pose[1])**2


# Utility function to convert between domains
def xydot_to_w(v, currentT, direction):

    # Component extraction
    xDot, yDot = v

    # Note L is negative if we are reversing
    sign = (-1)**direction

    # Compute matrix coefficients
    xDotC0 = R/2 * cos(currentT) - sign*R*L/D * sin(currentT)
    xDotC1 = R/2 * cos(currentT) + sign*R*L/D * sin(currentT)
    yDotC0 = R/2 * sin(currentT) + sign*R*L/D * cos(currentT)
    yDotC1 = R/2 * sin(currentT) - sign*R*L/D * cos(currentT)

    # Invert the matrix
    matDetInv = 1 / (xDotC0*yDotC1 - xDotC1*yDotC0)
    wR = matDetInv * ( yDotC1 * xDotTarg - xDotC1 * yDotTarg)
    wL = matDetInv * (-yDotC0 * xDotTarg + xDotC0 * yDotTarg)

    # Normalize omegas
    wMax = max(abs(wR), abs(wL))
    wR *= OMEGA_MAX_ALLOWED / min(wMax, 1)
    wL *= OMEGA_MAX_ALLOWED / min(wMax, 1)

    return (wR, wL)

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

    # Reverse the control point if needed
    current_pose = match_pose_to_dir(current_pose, drive_direction)

    # Calculate target xDot, yDot
    # Only the direction matters since we will normalize wheel vels
    xDotTarg = straight_waypoint[0] - current_pose[0]
    yDotTarg = straight_waypoint[1] - current_pose[1]

    # Convert to wheel velocities
    # Return because the method will normalize
    return xydot_to_w((xDotTarg, yDotTarg), current_pose[2], drive_direction)
