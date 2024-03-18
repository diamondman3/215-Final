"""
This script shows you how to select gripper for an environment.
This is controlled by gripper_type keyword argument.
"""
import matplotlib.pyplot as plt
import mujoco
import numpy as np
import time

import robosuite as suite
import robosuite.models.objects.primitive
from robosuite.models import MujocoWorldBase
from robosuite.models.arenas import TableArena
from robosuite.models.grippers import gripper_factory
from robosuite.models.objects import CylinderObject, MujocoObject
from robosuite.models.objects.primitive import cylinder
from robosuite.utils.mjcf_utils import new_joint
from robosuite import ALL_GRIPPERS
import robosuite.utils.transform_utils as tfutil
from robosuite.models.robots import UR5e
from robosuite.utils.placement_samplers import UniformRandomSampler

from final_rig import *

#TODO: I think the problem is that there are multiple coordinate systems at play here, for the robot and the block mostly.

if __name__ == "__main__":
    
    gripper = 'Robotiq85Gripper'

    # Notify user which gripper we're currently using
    print("Using gripper {}...".format(gripper))
    
    np.set_printoptions(precision=4, floatmode='fixed', suppress=True)

    # create environment with selected grippers
    # BIG IMPORTANT NOTE: THE ENVIRONMENT IS WHAT DEFINES WHAT OBJECTS ARE IN THE SIM
    # SPECIFICALLY, THE NAME.
    # EX LIFT MAKES A CUBE, PICKPLACE MAKES 4 FOOD ITEMS, ETC

    #I think what I want to do is set up a completely new Mujoco world.
    #https://robosuite.ai/docs/quickstart.html

    env = suite.make(
        "Lift",
        robots="UR5e", #Maximum reach of 850 mm from base
        gripper_types=gripper,
        has_renderer=True,  # make sure we can render to the screen
        has_offscreen_renderer=False,  # not needed since not using pixel obs
        use_camera_obs=False,  # do not use pixel observations
        control_freq=1,  # control should happen fast enough so that simulation looks smoother
        camera_names="frontview",
    )

    ROBOT_BASE_POS = env.robots[0].base_pos #Possibly need to rotate, too
    ROBOT_BASE_ORI = tfutil.quat2axisangle(env.robots[0].base_ori)
    REACH = .85

    env.table_offset=[0,0,.10] #Gets the table out of the way

    env.reset()
    env.sim.model.opt.gravity = 0 #No gravity, we're assuming this block is on a conveyor

    #Goal: Make this always start outside the robot's reach, then make it catch it at the closest point

    #posOriginal = transformCameraToU([.5, 0, 0], ROBOT_BASE_POS, ROBOT_BASE_ORI)
    posOriginal = generateBlockPos(ROBOT_BASE_POS, REACH)
    print(str(posOriginal))

    positivePosOriginal = [posOriginal[0]>ROBOT_BASE_POS[0], posOriginal[1]>ROBOT_BASE_POS[1], posOriginal[2]>ROBOT_BASE_POS[2]]
    iCantConvertBoolDirectlyToFloat = [0.0,0.0,0.0]
    for i in range(len(positivePosOriginal)):
        if positivePosOriginal[i] == True:
            iCantConvertBoolDirectlyToFloat[i] = 1
        else:
            iCantConvertBoolDirectlyToFloat[i] = -1

    env.sim.data.qpos[-7:-4] = posOriginal #OH MY FUCKING GOD, THAT WAS THE PROBLEM?!?!??!!

    # Tuple of position and orientation (quat) of the cube
    boxQuat = tfutil.convert_quat(env.sim.data.get_body_xquat('cube_main'))
    boxOri = tfutil.quat2axisangle(boxQuat)

    #This loop defines a velocity so that the block always comes within reach of the arm.
    #I was considering doing some insane geometry and trig, then I just realized this is easier.
    #Also checks that it actually does move.
    while True:
        speeds = np.multiply(np.multiply(-2, iCantConvertBoolDirectlyToFloat), [np.random.rand(), np.random.rand(), np.random.rand()])
        desiredPoseAndReach = predictCatchPosition(posOriginal, speeds, boxOri, env) - np.concatenate([env.table_offset, [0,0,0,0]])
        dist = desiredPoseAndReach[-1]
        temp = np.concatenate([ROBOT_BASE_POS, [0, 0, 0]])
        desiredPose = np.add(desiredPoseAndReach[:-1], temp)
        outOfReach = (dist > REACH-.1 or desiredPose[1] > ROBOT_BASE_POS[1] + .4)
        slow = (abs(np.linalg.norm(speeds)) < .1)

        if (not outOfReach and not slow):
            break

    via = np.add(desiredPose, [0, .1, 0, 0, 0, 0])

    distances = []
    angles = []
    iterations = 0

    blockToHandDistance = np.linalg.norm(np.subtract(getGripperEEFPose(env)[0], env.sim.data.qpos[-7:-4]))
    blockToBaseDistance = np.linalg.norm(np.subtract(ROBOT_BASE_POS, env.sim.data.qpos[-7:-4]))
    handToDPDistance = np.linalg.norm(np.subtract(getGripperEEFPose(env)[0], desiredPose[0:3]))
    handToViaDistance = np.linalg.norm(np.subtract(getGripperEEFPose(env)[0], via[0:3]))
    goToVia = True

    ''''''
    while(iterations < 300):
        desiredPose = [0, 0, ROBOT_BASE_POS[2], 0, -np.pi, 0]
        jointAngles = inverseKinematics(desiredPose, env)
        env.render()
        iterations +=1

        currPose = np.ndarray.tolist(abomination2array(getGripperEEFPose(env)))
        handToDPDistance = np.linalg.norm(np.subtract(currPose[0:3], desiredPose[0:3]))
        distances = np.concatenate([distances, [handToDPDistance]])
        fuckyou = angle(desiredPose[3:], currPose[3:])
        angles = np.concatenate([angles, [fuckyou]])

    '''
    while (blockToHandDistance > .05 and iterations < 1000):

        env.sim.data.qvel[-6:-3] = speeds #RELATIVE TO THE BLOCK
        env.sim.data.qvel[-3:] = 0 #no spinning out of control when hit
        env.sim.step() #makes the block move.

        blockToHandDistance = np.linalg.norm(np.subtract(getGripperEEFPose(env)[0], env.sim.data.qpos[-7:-4]))
        blockToBaseDistance = np.linalg.norm(np.subtract(ROBOT_BASE_POS, env.sim.data.qpos[-7:-4]))
        handToDPDistance = np.linalg.norm(np.subtract(getGripperEEFPose(env)[0], desiredPose[0:3]))
        handToViaDistance = np.linalg.norm(np.subtract(getGripperEEFPose(env)[0], via[0:3]))

        distances = np.concatenate([distances, [blockToHandDistance]])

        if blockToBaseDistance < REACH and goToVia:
            jointAngles = inverseKinematics(DesiredPose_in_U=via, env=env)
            if handToViaDistance < .1:
                goToVia = False
        elif blockToBaseDistance < REACH and not goToVia:
            jointAngles = inverseKinematics(DesiredPose_in_U=desiredPose, env=env)
        if handToDPDistance < .1:
            desiredPose[0:3] = env.sim.data.qpos[-7:-4]
        env.render()
        iterations+=1
    '''

    plt.plot(list(range(len(distances))), distances)
    plt.xlabel("Time")
    plt.ylabel("Distance (m)")
    plt.title("Distance between claw and target over time")
    plt.plot()
    plt.show()

    plt.plot(list(range(len(angles))), angles)
    plt.xlabel("Time")
    plt.ylabel("Angle(rads)")
    plt.title("Angle between claw and target over time")
    plt.plot()
    plt.show()

    '''
    if (abs(getError() - missError) <= .0001):
        print('===============================================')
        print('Block not caught.')
        print('===============================================')
    '''
    if False:
        pass
    else:
        print('===============================================')
        print('Block caught at ' + str(env.sim.data.qpos[-7:-4]))
        print('===============================================')

env.close()
