import xml.etree.ElementTree as ET
import os
from typing import Any

import numpy as np
import robosuite.utils.transform_utils as tfutil
import copy

#lol you can't make real constants in python
maxDistanceError = 1E-2
maxJointRotation = np.pi / 30  # 30hz sim, so that much is the maximum rotation per step
reduction = .1  # Smaller steps so it doesn't overshoot
errorGlobal = 999

#this and inverseKinematics debugged by chatgpt. Including the originals.
def predictCatchPosition(blockPosition, blockVelocity, blockOri, env=[]):
    initialJointAngles = env.robots[0]._joint_positions
    currPose = abomination2array(getGripperEEFPose(env))

    maxReach = 0.85/2

    minDistance = np.linalg.norm(blockPosition - currPose[0:3])
    closestPose = blockPosition

    for i in range(1000):
        pos = np.multiply(np.add(blockPosition, blockVelocity),  float(i+1) / 1000.0)
        '''doesn't work for cameravelocity either'''
        dist = np.linalg.norm(np.subtract(pos, currPose[0:3]))

        inside_pole = (np.sqrt(pos[0]**2 + pos[1]**2) <= .2 and pos[2] < 1)

        if dist < minDistance and not inside_pole:
            minDistance = dist
            closestPose = pos
    if minDistance < .3:
        return np.concatenate([closestPose, blockOri, [minDistance]])
    return [999,999,999,0,0,0,9999]

def inverseKinematics(DesiredPose_in_U=(np.zeros(6,)), env=[]):
    robotBasePose = (env.robots[0].base_pos, env.robots[0].base_ori)
    initialJointAngles = env.robots[0]._joint_positions

    # ============= Your code here =============
    J = getJacobian(env)
    dp = DesiredPose_in_U

    # =============NOTE TO SELF: ABOMINATION2ARRAY ALL THE POSES=======================
    currThetas = initialJointAngles
    currPose = abomination2array(getGripperEEFPose(env))
    error = np.subtract(dp, currPose)  # calcError(dp, currPose)

    maxDistanceError = .01
    maxAngleError = np.pi / 12.0
    reduction = .025  # Smaller steps so it doesn't overshoot

    if(np.linalg.norm(error[0:2]) > maxDistanceError or (angle(currPose[3:], dp[3:]) > maxAngleError)):
        J = getJacobian(env)  # Jacobian changes based on position
        #deltaTheta = np.linalg.lstsq(J, error, rcond=None)[0]
        # Compute position error
        pos_error = dp[:3] - currPose[:3]

        # Compute angle error using axis-angle representation
        angle_error = np.cross(currPose[3:], dp[3:])

        # Combine position and angle errors
        error = np.concatenate((pos_error, angle_error))

        # Compute pseudo-inverse of Jacobian
        jacobian_pseudo_inv = np.linalg.pinv(J)

        # Compute joint angle change using Moore-Penrose pseudo-inverse
        deltaTheta = np.dot(jacobian_pseudo_inv, error)

        deltaTheta = deltaTheta * reduction
        deltaTheta = np.clip(deltaTheta, -maxJointRotation, maxJointRotation)

        currThetas = currThetas + deltaTheta
        if np.mod(currThetas[2], 2.0*np.pi) < np.pi/180.0:
            currThetas[2] = np.pi/180.0 # singularity prevention
        elif np.mod(currThetas[2], 2.0 * np.pi) > 359.0 * np.pi / 180.0:
            currThetas[2] = 359 * np.pi / 180
        if np.mod(currThetas[3], 2.0*np.pi) < np.pi/180.0:
            currThetas[3] = np.pi/180.0
        elif np.mod(currThetas[2], 2.0 * np.pi) > 359.0 * np.pi / 180.0:
            currThetas[2] = 359.0 * np.pi / 180.0

        updateGripperEEFPose(env, currThetas)

        dist = np.linalg.norm(error[0:2])
        ang = angle(currPose[3:], dp[3:])

    return currThetas



def getError():
    return errorGlobal

def getMaxDistanceError():
    return maxDistanceError

def abomination2array(a):
    position = a[0]
    quat = a[1]
    angle = tfutil.quat2axisangle(quat)
    unitAngle = angle/(abs(angle[0]) + abs(angle[1]) + abs(angle[2]))
    saneArray = np.concatenate((position, unitAngle))
    return saneArray


#=========== Not a HW problem below ==========

def angle(v1, v2):
    angle_radians = np.arccos(np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2)))
    return angle_radians


def getGripperEEFPose(env): # This function works as a forward Kinematics
    gripper_EEF_pose = (env.robots[0].sim.data.get_body_xpos('gripper0_eef'), tfutil.convert_quat(env.robots[0].sim.data.get_body_xquat('gripper0_eef')))
    return gripper_EEF_pose # Outputs the position and quaternion (x,y,z,w) of the EEF pose in Universial Frame{0}.

def updateGripperEEFPose(env, setJointAngles):
    env.robots[0].set_robot_joint_positions(setJointAngles)

def getJacobian(env): # This function returns the jacobian of current configurations
    jacp = env.robots[0].sim.data.get_body_jacp('gripper0_eef').reshape((3, -1))[:,env.robots[0]._ref_joint_vel_indexes]
    jacr = env.robots[0].sim.data.get_body_jacr('gripper0_eef').reshape((3, -1))[:,env.robots[0]._ref_joint_vel_indexes]    
    jacobianMat_gripperEEF = np.concatenate((jacp, jacr),axis=0)
    return jacobianMat_gripperEEF #Outputs the Jacobian expressed in {0}

#V This function written by chatgpt
def generateBlockPos(robot_base_pos, reach):
    # Define the edge length of the box
    box_edge_length = 1.5

    # Calculate the minimum and maximum coordinates for each axis
    min_coord = robot_base_pos - box_edge_length / 2
    max_coord = robot_base_pos + box_edge_length / 2

    # Generate random coordinates within the specified range
    random_pos = np.random.uniform(min_coord, max_coord, size=3)

    # Ensure the generated position is farther than REACH from the center
    while ((np.linalg.norm(random_pos - robot_base_pos) <= reach) or random_pos[0] < robot_base_pos[0]
           or abs(random_pos[2] - robot_base_pos[2]) > reach/2 ):
        random_pos = np.random.uniform(min_coord, max_coord, size=3)

    return random_pos

