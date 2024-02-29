"""
This script shows you how to select gripper for an environment.
This is controlled by gripper_type keyword argument.
"""
import numpy as np
import time

import robosuite as suite
from robosuite import ALL_GRIPPERS
import robosuite.utils.transform_utils as tfutil

from final_rig import *

if __name__ == "__main__":
    
    gripper = 'Robotiq85Gripper'

    # Notify user which gripper we're currently using
    print("Using gripper {}...".format(gripper))
    
    np.set_printoptions(precision=4, floatmode='fixed', suppress=True)

    # create environment with selected grippers
    # BIG IMPORTANT NOTE: THE ENVIRONMENT IS WHAT DEFINES WHAT OBJECTS ARE IN THE SIM
    # SPECIFICALLY, THE NAME.
    # EX LIFT MAKES A CUBE, PICKPLACE MAKES 4 FOOD ITEMS, ETC
    env = suite.make(
        "Lift",
        robots="UR5e",
        gripper_types=gripper,
        has_renderer=True,  # make sure we can render to the screen
        has_offscreen_renderer=False,  # not needed since not using pixel obs
        use_camera_obs=False,  # do not use pixel observations
        control_freq=1,  # control should happen fast enough so that simulation looks smoother
        camera_names="frontview",
    )
    
    rod = suite.models.objects.primitive.CylinderObject("rod", size=[.01, 1])
    #env.sim.add_object(rod)
    
    while(True):
        # Reset the env
        env.reset()

        # Tuple of position and orientation (quat) of the cube
        boxQuat = tfutil.convert_quat(env.sim.data.get_body_xquat('cube_main'))
        boxQuat = tfutil.quat_multiply(boxQuat, tfutil.axisangle2quat([0.,np.pi,0.0])) # reorienting for gripper to approach downward.
        desiredPose = (env.sim.data.get_body_xpos('cube_main'), boxQuat ) 


        jointAngles = inverseKinematics(DesiredPose_in_U=desiredPose, env=env)
        Your_gripper_EEF_pose = getGripperEEFPose(env, jointAngles)
        
        
        print('===============================================')
        print('Desired Pose', desiredPose)        
        print('Your inverse kinematics result ', Your_gripper_EEF_pose)
        print('===============================================')    

        env.render()

        input('Hit Enter to test new pose')
    # env.close()
    


