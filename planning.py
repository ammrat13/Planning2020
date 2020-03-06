#!/usr/bin/env python3
"""
File:          planning.py
Author:        Binit Shah
Last Modified: Ammar on 3/5
"""

# Planning constants for the robot
R = .045
L = .1
D = .2427


def order_blocks(block_config):
	return block_config


def reached(current_pose, goal_pos):
	return False


# Utility function to convert between domains
def xydot_to_w(v, currentT):

    xDot, yDot, tDot = v

    xDotC0 = R/2 * cos(currentT) - R*L/D * sin(currentT)
    xDotC1 = R/2 * cos(currentT) + R*L/D * sin(currentT)
    yDotC0 = R/2 * sin(currentT) + R*L/D * cos(currentT)
    yDotC1 = R/2 * sin(currentT) - R*L/D * cos(currentT)

    matDetInv = 1 / (xDotC0*yDotC1 - xDotC1*yDotC0)

    wR = matDetInv * ( yDotC1 * xDotTarg - xDotC1 * yDotTarg)
    wL = matDetInv * (-yDotC0 * xDotTarg + xDotC0 * yDotTarg)

    return (wR, wL)


# Does exactly what it says
# Depends on the current state of the robot for navigation
def compute_wheel_velocities(current_pose, goal_pos):
    pass
