import numpy as np
import robosuite.utils.transform_utils as tfutil

def random_rod(self):
    rodQuat = tfutil.random_quat()
    rodMat = tfutil.quat2mat(rodQuat)
    speed = np.random.random # 0 to 1. Will be moving along z axis.
    speed = speed - .5 # -.5 to .5
    speed = speed * 4 # -2 to 2
    return rodMat, speed

def step(self, rodMat, speed):
    speedVec = [0,0,speed, 1]
    rotatedSpeedVec = np.multiply(rodMat, speedVec)
    translationMat = #identity plus speedVec, will do later
    rodMat = np.multiply(rodMat, translationMat)
    return rodMat