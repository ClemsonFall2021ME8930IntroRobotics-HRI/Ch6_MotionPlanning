# Make sure to have the server side running in CoppeliaSim: 
# in a child script of a CoppeliaSim scene, add following command
# to be executed just once, at simulation start:
#
# simRemoteApi.start(19999)
#
# then start simulation, and run this program.
#
# IMPORTANT: for each successful call to simxStart, there
# should be a corresponding call to simxFinish at the end!
import cv2
import numpy as np
import sim
import time
print ('Program started')
sim.simxFinish(-1) # just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to CoppeliaSim
if clientID!=-1:
    e = sim.simxStartSimulation(clientID,sim.simx_opmode_blocking)
    print('start',e)
    try:
        res,camera0_handle = sim.simxGetObjectHandle(clientID,'top_view_camera',sim.simx_opmode_oneshot_wait)
        res_l,right_motor_handle = sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor',sim.simx_opmode_oneshot_wait)
        errorCode_rightM = sim.simxSetJointTargetVelocity(clientID, right_motor_handle,50, sim.simx_opmode_oneshot)
        err,resolution,image=sim.simxGetVisionSensorImage(clientID,camera0_handle,0,sim.simx_opmode_streaming)        
        if err == sim.simx_return_ok:
            img = np.array(image,dtype=np.uint8)
            img.resize([resolution[1],resolution[0],3])
    finally:
        sim.simxFinish(-1)
