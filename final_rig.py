import xml.etree.ElementTree as ET
import os
import numpy as np
import robosuite.utils.transform_utils as tfutil
import copy

#TODO: GETTING ACCURATE POSITION WORKS. GETTING ACCURATE ANGLE IS SLOW AS FUCK


def inverseKinematics(DesiredPose_in_U = (np.zeros(3,), np.array([0., 0., 0., 1.])), env = []):
    # These two OPTIONAL helper functions will actually set the angles and get you the gripper endeffector pose and jacobian.    
    #  "getGripperEEFPose" is actually moving the robot in the simulation but it does not render it. This works as a forward kinematics function. If you want to see the new robot pose, add: env.render()
    # "getJacobian(env)" returns the Jacobian computed for the gripper end-effector which is different from what you get in HW3. 

    #getGripperEEFPose(env, setJointAngles)
    #getJacobian(env)


    # We will bring the robot back to original pose at the end of "inverseKinematics" function, because it is inteded to compute the joint angles, not execute the joint angles.
    # But it is not required for you to implement it.




    # Tuple of position and orientation (quat) of the base frame expressed in world frame
    robotBasePose = (env.robots[0].base_pos, env.robots[0].base_ori) 
    initialJointAngles= env.robots[0]._joint_positions
    jointAngles = initialJointAngles.copy()
    
    #============= Your code here =============
    J = getJacobian(env)
    JT = np.transpose(J)
    dp = abomination2array(DesiredPose_in_U)
    #I think this is where we're supposed to use Newton-Raphson
    #N-R is inverse kinematics, prof said

    #=============NOTE TO SELF: ABOMINATION2ARRAY ALL THE POSES=======================
    currThetas = initialJointAngles
    currPose = abomination2array(getGripperEEFPose(env, currThetas))
    error = dp-currPose #calcError(dp, currPose)
    
    #1mm, pi/12 rads
    #I could theoretically make it more accurate, but if I do it gets far, far slower
    #And I'm pretty sure you'd rather have a decent answer in 1 minute than a slightly more decent answer in 20.
    #(I tried with pi/16, that's how long it took)
    maxDistanceError = .001
    maxAngleError = np.pi/12.0
    reduction = .01 # Smaller steps so it doesn't overshoot
    
    while(np.linalg.norm(error[0:2])>maxDistanceError or (abs(error[3]) > maxAngleError) or abs(error[4]) > maxAngleError or abs(error[5]) > maxAngleError):
        J = getJacobian(env) # Jacobian changes based on position
        deltaTheta = np.linalg.lstsq(J, error, rcond=None)[0]
        #reduction = np.array([.01,.01,.01,.1,.1,.1]) 
        #Turns out, inconsistent reductions cause problems.
        
        deltaTheta = deltaTheta * reduction
        currThetas = currThetas + deltaTheta

        currPose = abomination2array(getGripperEEFPose(env, currThetas))
        error = dp - currPose
        print("errors: " + str(np.linalg.norm(error[0:2])) + " " + str(error[3:]))
 
    #==========================================
    getGripperEEFPose(env, initialJointAngles) # Brings the robot to the initial joint angle.
    env.render()
    return jointAngles

'''
def spin2win(env, dp, currPose, currThetas):
    print("Spin2win")
    error = dp-currPose
    maxAngleError = np.deg2rad(1)
    while(any(error[3:])>maxAngleError):
        if(error[3]>maxAngleError):
            currThetas[3]-=.001
        elif(error[3]<-maxAngleError):
            currThetas[3]+=.001
        if(error[4]>maxAngleError):
            currThetas[4]-=.001
        elif(error[4]<-maxAngleError):
            currThetas[4]+=.001
        if(error[5]>maxAngleError):
            currThetas[5]-=.001
        elif(error[5]<-maxAngleError):
            currThetas[5]+=.001
        currPose = abomination2array(getGripperEEFPose(env, currThetas))
        error = dp-currPose
    return currThetas
'''

def abomination2array(a):
    position = a[0]
    quat = a[1]
    angle = tfutil.quat2axisangle(quat)
    saneArray = np.concatenate((position, angle))
    return saneArray


#=========== Not a HW problem below ==========

def getGripperEEFPose(env, setJointAngles): # This function works as a forward Kinematics

    env.robots[0].set_robot_joint_positions(setJointAngles)
    gripper_EEF_pose = (env.robots[0].sim.data.get_body_xpos('gripper0_eef'), tfutil.convert_quat(env.robots[0].sim.data.get_body_xquat('gripper0_eef')))  
    env.render()   
    return gripper_EEF_pose # Outputs the position and quaternion (x,y,z,w) of the EEF pose in Universial Frame{0}.

def getJacobian(env): # This function returns the jacobian of current configurations
    jacp = env.robots[0].sim.data.get_body_jacp('gripper0_eef').reshape((3, -1))[:,env.robots[0]._ref_joint_vel_indexes]
    jacr = env.robots[0].sim.data.get_body_jacr('gripper0_eef').reshape((3, -1))[:,env.robots[0]._ref_joint_vel_indexes]    
    jacobianMat_gripperEEF = np.concatenate((jacp, jacr),axis=0)
    return jacobianMat_gripperEEF #Outputs the Jacobian expressed in {0}

