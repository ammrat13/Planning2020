#!/usr/bin/env python3
"""
File:          planning.py
Author:        Binit Shah
Last Modified: Ammar on 3/5
"""

def order_blocks(block_config):
	return block_config

def reached(current_pose, goal_pos):
	return False

def compute_wheel_velocities(current_pose, goal_pos, dt):
    R = .045
    L = .1
    D = .2427

    BEZIERX = [-.93,2,-1,0]
    BEZIERY = [0,0,1,-2]

    currentTheta = 0.0
    currentU = 0.0

    while True:
        jstates = p.getJointStates(self.robot, self.motor_links)
        wl = jstates[0][1]
        wr = jstates[1][1]

        xDotC0 = R/2 * cos(currentTheta) - R*L/D * sin(currentTheta)
        xDotC1 = R/2 * cos(currentTheta) + R*L/D * sin(currentTheta)
        yDotC0 = R/2 * sin(currentTheta) + R*L/D * cos(currentTheta)
        yDotC1 = R/2 * sin(currentTheta) - R*L/D * cos(currentTheta)

        xDot = xDotC0*wr + xDotC1*wl
        yDot = yDotC0*wr + yDotC1*wl
        thetaDot = (R/D)*wr + (-R/D)*wl

        currentTheta += thetaDot * dt

        dl = sqrt(xDot**2 + yDot**2) * dt
        xDotTarg = 3*BEZIERX[3] * currentU**2 + 2*BEZIERX[2] * currentU + BEZIERX[1]
        yDotTarg = 3*BEZIERY[3] * currentU**2 + 2*BEZIERY[2] * currentU + BEZIERY[1]
        currentU += dl / sqrt(xDotTarg**2 + yDotTarg**2)


        matDetInv = 1 / (xDotC0*yDotC1 - xDotC1*yDotC0)
        wrTarg = yDotC1*matDetInv*xDotTarg - xDotC1*matDetInv*yDotTarg
        wlTarg = -yDotC0*matDetInv*xDotTarg + xDotC0*matDetInv*yDotTarg

        wmax = max(abs(wrTarg), abs(wlTarg))
        wrTarg /= max(1, wmax)
        wlTarg /= max(1, wmax)

        yield (wlTarg, wrTarg)
