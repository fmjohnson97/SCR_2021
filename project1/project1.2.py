import sys
sys.path.append('../sim_module')
import sim

import math
import time

print('Program started')
sim.simxFinish(-1)  # just in case, close all opened connections
clientID = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)  # Connect to CoppeliaSim
if clientID == -1:
    print('Could not connect remote API server!')
    sys.exit(-1)

print('Connected to remote API server')

### Utility Functions ###


def getHandleFromName(name):
    returnCode, handle = sim.simxGetObjectHandle(clientID, name, sim.simx_opmode_blocking)
    # print(name, handle)
    if not returnCode:
        return handle
    else:
        return returnCode


def getAbsolutePose(handle, mode='stream'):
    if mode == 'block':
        mode = sim.simx_opmode_blocking
    elif mode == 'buffer':
        mode = sim.simx_opmode_buffer
    else:
        mode = sim.simx_opmode_streaming

    res1, pos = sim.simxGetObjectPosition(clientID, handle, -1, mode)
    # print(res1, pos)
    res2, rot = sim.simxGetObjectOrientation(clientID, handle, -1, mode)
    # print(res2, rot)
    return pos, rot


def setAbsolutePose(handle, pos, rot):
    res1 = sim.simxSetObjectPosition(clientID, handle, -1, pos, sim.simx_opmode_oneshot)
    # print(res1)
    res2 = sim.simxSetObjectOrientation(clientID, handle, -1, rot, sim.simx_opmode_oneshot)
    # print(res2)
    return res1, res2


def computePath(handle):
    initPos, initRot = getAbsolutePose(handle, 'block')

    emptyBuff = bytearray()
    res, retInts, retFloats, retStrings, retBuffer = sim.simxCallScriptFunction(
        clientID,  # client
        'lumibot',  # scriptDescription
        sim.sim_scripttype_childscript,  # scriptHandleOrType
        'computePath',  # functionName
        [],  # ints
        [],  # floats
        [],  # strings
        emptyBuff,  # buffer
        sim.simx_opmode_blocking,
    )
    # print(res, retInts, retFloats, retStrings, retBuffer)

    path = []
    for i in range(0, len(retFloats) - 3, 3):
        pos = (retFloats[i], retFloats[i + 1], initPos[2])
        rot = (initRot[0], initRot[1], retFloats[i + 2])
        path.append((pos, rot))

    return path, retFloats


def visualizePath(raw_path):
    emptyBuff = bytearray()
    res, retInts, retFloats, retStrings, retBuffer = sim.simxCallScriptFunction(
        clientID,  # client
        'lumibot',  # scriptDescription
        sim.sim_scripttype_childscript,  # scriptHandleOrType
        'visualizePath',  # functionName
        [],  # ints
        raw_path,  # floats
        [],  # strings
        emptyBuff,  # buffer
        sim.simx_opmode_blocking,
    )
    # print(res, retInts, retFloats, retStrings, retBuffer)

    return res


def followPath(handle, path, left_motor_handle, right_motor_handle):
    v_des = 0.1  # total desired velocity of the robot
    d = 0.0886 / 2  #distance between the wheels
    r = 0.024738  #radius of the wheel
    epsilon = 0.04  # distance threshold

    for pos, rot in path[::10]:
        robot_pos, robotRot = getAbsolutePose(handle, 'block')
        distance = ((pos[0] - robot_pos[0])**2 + (pos[1] - robot_pos[1])**2)**.5
        while distance > epsilon:

            desired_angle = math.atan2(pos[1] - robot_pos[1], pos[0] - robot_pos[0])
            angle_diff = robotRot[2] - desired_angle - math.pi / 2
            w_des = 0.8 * angle_diff  # total desired angular velocity of the robot
            v_right = v_des - d * w_des
            v_left = v_des + d * w_des
            W_right = v_right / r  # angular velocity of the right wheel of the robot
            W_left = v_left / r  # angular velocity of the left wheel of the robot
            # print(angle_diff, W_left,W_right)
            # print(rot,'\t',robotRot)
            # print(pos, robot_pos)
            # print(distance, angle_diff, robotRot[2])
            res = sim.simxSetJointTargetVelocity(clientID, right_motor_handle, W_right, sim.simx_opmode_oneshot)
            res = sim.simxSetJointTargetVelocity(clientID, left_motor_handle, W_left, sim.simx_opmode_oneshot)
            time.sleep(0.025)
            robot_pos, robotRot = getAbsolutePose(handle, 'block')
            distance = ((pos[0] - robot_pos[0])**2 + (pos[1] - robot_pos[1])**2)**.5

    res = sim.simxSetJointTargetVelocity(clientID, right_motor_handle, 0, sim.simx_opmode_oneshot)
    res = sim.simxSetJointTargetVelocity(clientID, left_motor_handle, 0, sim.simx_opmode_oneshot)

    # VVV simple forced path following VVV
    # for pos, rot in path:
    #     setAbsolutePose(handle, pos, rot)
    #     time.sleep(0.05)


### Simulation  ###

# load the scene
res = sim.simxLoadScene(clientID, 'scene1.2.ttt', True, sim.simx_opmode_blocking)
res = sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot)

# get handles
robotHandle = getHandleFromName('lumibot')
start_dummy = getHandleFromName('Start')
end_dummy = getHandleFromName('End')

left_motor_handle = getHandleFromName('lumibot_leftMotor')
right_motor_handle = getHandleFromName('lumibot_rightMotor')

# Get the robot initial position and orientation
startPos, startRot = getAbsolutePose(start_dummy, 'block')
endPos, endRot = getAbsolutePose(end_dummy, 'block')
print(startPos, startRot)
print(endPos, endRot)

# compute the path
path, raw_path = computePath(robotHandle)
print(len(path))

# visualize and follow the path
visualizePath(raw_path)
followPath(robotHandle, path, left_motor_handle, right_motor_handle)

input("Press Enter to end simulation...\n")
# Stop simulation:
sim.simxStopSimulation(clientID, sim.simx_opmode_oneshot_wait)
sim.simxFinish(clientID)
