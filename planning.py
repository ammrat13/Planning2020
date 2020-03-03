#!/usr/bin/env python3
"""
File:          planning.py
Author:        Binit Shah
Last Modified: Binit on 2/21
"""

def order_blocks(block_config):
	return block_config

def reached(current_pose, goal_pos):
	return False

def compute_wheel_velocities(current_pose, goal_pos):
    R = .045
    L = .1
    D = .2427

    BEZIERX = [-.93,2,-1,0]
    BEZIERY = [0,0,1,-2]

    curve = Utilities.add_bezier_curve(BEZIERX, BEZIERY)

    currentTheta = 0.0
    currentU = 0.0

    lastCtrlPt = p.multiplyTransforms(
                    p.getBasePositionAndOrientation(self.robot)[0],
                    p.getBasePositionAndOrientation(self.robot)[1],
                    (0,0,L + 0.074676),
                    (0,0,0,1))[0]

    lid = p.addUserDebugLine(
                    [BEZIERX[0]+BEZIERX[1]*currentU+BEZIERX[2]*currentU**2+BEZIERX[3]*currentU**3,
                     BEZIERY[0]+BEZIERY[1]*currentU+BEZIERY[2]*currentU**2+BEZIERY[3]*currentU**3,
                     -10],
                    [BEZIERX[0]+BEZIERX[1]*currentU+BEZIERX[2]*currentU**2+BEZIERX[3]*currentU**3,
                     BEZIERY[0]+BEZIERY[1]*currentU+BEZIERY[2]*currentU**2+BEZIERY[3]*currentU**3,
                     10])
    ctrl = p.addUserDebugLine([lastCtrlPt[0], lastCtrlPt[1], -10], [lastCtrlPt[0], lastCtrlPt[1], 10])

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

        currentTheta += thetaDot / 240

        newCtrlPt = p.multiplyTransforms(
                        p.getBasePositionAndOrientation(self.robot)[0],
                        p.getBasePositionAndOrientation(self.robot)[1],
                        (0,0,L + 0.074676),
                        (0,0,0,1))[0]
        myxDot = 240 * (newCtrlPt[0] - lastCtrlPt[0])
        myyDot = 240 * (newCtrlPt[1] - lastCtrlPt[1])
        lastCtrlPt = newCtrlPt

        dl = sqrt(xDot**2 + yDot**2) / 240
        xDotTarg = 3*BEZIERX[3] * currentU**2 + 2*BEZIERX[2] * currentU + BEZIERX[1]
        yDotTarg = 3*BEZIERY[3] * currentU**2 + 2*BEZIERY[2] * currentU + BEZIERY[1]
        currentU += dl / sqrt(xDotTarg**2 + yDotTarg**2)


        matDetInv = 1 / (xDotC0*yDotC1 - xDotC1*yDotC0)
        wrTarg = yDotC1*matDetInv*xDotTarg - xDotC1*matDetInv*yDotTarg
        wlTarg = -yDotC0*matDetInv*xDotTarg + xDotC0*matDetInv*yDotTarg

        wmax = max(abs(wrTarg), abs(wlTarg))
        wrTarg /= max(1, wmax)
        wlTarg /= max(1, wmax)

        p.removeUserDebugItem(lid);
        lid = p.addUserDebugLine(
                    [BEZIERX[0]+BEZIERX[1]*currentU+BEZIERX[2]*currentU**2+BEZIERX[3]*currentU**3,
                     BEZIERY[0]+BEZIERY[1]*currentU+BEZIERY[2]*currentU**2+BEZIERY[3]*currentU**3,
                     -10],
                    [BEZIERX[0]+BEZIERX[1]*currentU+BEZIERX[2]*currentU**2+BEZIERX[3]*currentU**3,
                     BEZIERY[0]+BEZIERY[1]*currentU+BEZIERY[2]*currentU**2+BEZIERY[3]*currentU**3,
                     10])
        p.removeUserDebugItem(ctrl)
        ctrl = p.addUserDebugLine([lastCtrlPt[0], lastCtrlPt[1], -10], [lastCtrlPt[0], lastCtrlPt[1], 10])

        print(f"{xDot - myxDot} {yDot - myyDot}")

        yield (wlTarg, wrTarg)
