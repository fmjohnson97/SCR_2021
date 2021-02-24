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

try:
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

    def followPath(handle, path, followDist, left_motor_handle, right_motor_handle):
        d = 0.0886 / 2  #distance between the wheels
        r = 0.024738  #radius of the wheel
        epsilon = followDist # distance threshold

        for pos, rot in path:
            robot_pos, robotRot = getAbsolutePose(handle, 'block')
            print(math.dist(pos, robot_pos))
            while math.dist(pos[:-1], robot_pos[:-1]) > epsilon:

                angle_diff = robotRot[2] - math.atan2(pos[1] - robot_pos[1], pos[0] - robot_pos[0])
                angle_diff -= math.pi / 2  # robot rotation offset
                angle_diff = math.remainder(angle_diff, 2 * math.pi)  # bound between [-pi,pi]
                if abs(angle_diff) > math.pi / 16:
                    v_des = 0.02
                    w_des = 0.8
                else:
                    v_des = 0.08
                    w_des = 0.8
                w_des *= angle_diff  # total desired angular velocity of the robot
                v_right = v_des - d * w_des
                v_left = v_des + d * w_des
                W_right = v_right / r  # angular velocity of the right wheel of the robot
                W_left = v_left / r  # angular velocity of the left wheel of the robot
                res = sim.simxSetJointTargetVelocity(clientID, right_motor_handle, W_right, sim.simx_opmode_oneshot)
                res = sim.simxSetJointTargetVelocity(clientID, left_motor_handle, W_left, sim.simx_opmode_oneshot)
                # iterate time step
                time.sleep(0.025)
                robot_pos, robotRot = getAbsolutePose(handle, 'block')

        res = sim.simxSetJointTargetVelocity(clientID, right_motor_handle, 0, sim.simx_opmode_oneshot)
        res = sim.simxSetJointTargetVelocity(clientID, left_motor_handle, 0, sim.simx_opmode_oneshot)

    def approachObject(pos_rot, followDist,robotHandle, left_motor_handle, right_motor_handle):
        followPath(robotHandle, [pos_rot], followDist, left_motor_handle, right_motor_handle)

    ### Simulation  ###

    #set global variables
    HUMAN_FOLLOW_DIST=.4
    DOOR_FOLLOW_DIST=.1

    # load the scene
    res = sim.simxLoadScene(clientID, 'fivedoors.ttt', True, sim.simx_opmode_blocking)
    res = sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot)

    # get handles
    robotHandle = getHandleFromName('lumibot')
    left_motor_handle = getHandleFromName('lumibot_leftMotor')
    right_motor_handle = getHandleFromName('lumibot_rightMotor')
    doors=[getHandleFromName('_doorJoint')]
    pathCtrlPoints = []
    for i in range(1,5):
        doors.append(getHandleFromName('_doorJoint#'+str(i)))
        pathCtrlPoints.append(getHandleFromName('CtrlPt'+str(i)))
    pathCtrlPoints.append(getHandleFromName('CtrlPt5'))
    humanHandle = getHandleFromName('Bill_base')

    doorInd=0
    for i in range(1000):
        #get position of closest door
        doorPos, doorRot = getAbsolutePose(doors[doorInd], 'block')
        #get human's position
        humPos, humRot = getAbsolutePose(humanHandle, 'block')

        # import pdb; pdb.set_trace()
        # Code for lumibot go to the human's current position
        approachObject([humPos,humRot], HUMAN_FOLLOW_DIST, robotHandle, left_motor_handle, right_motor_handle)

        if math.dist(doorPos[:-1],humPos[:-1])>.5:
            #have the human move one step on its path
            emptyBuff = bytearray()
            res, retInts, retFloats, retStrings, retBuffer = sim.simxCallScriptFunction(
                clientID,  # client
                'Bill',  # scriptDescription
                sim.sim_scripttype_childscript,  # scriptHandleOrType
                'step',  # functionName
                [],  # ints
                [],  # floats
                [],  # strings
                emptyBuff,  # buffer
                sim.simx_opmode_blocking,
            )
        else:
            # Code for lumibot go to the human's current position
            approachObject([doorPos,doorRot], DOOR_FOLLOW_DIST, robotHandle, left_motor_handle, right_motor_handle)
          
            #Robot opens the door if closed:     
            r, pos= simxGetJointPosition(clientID, doors[doorInd])
            if (pos == 0):
                sim.simxSetJointPosition(clientID, doors[doorInd], -90 * math.pi / 180)
            
            doorInd+=1


    input("Press Enter to end simulation...\n")
except KeyboardInterrupt:
    pass
finally:
    # Stop simulation:
    sim.simxStopSimulation(clientID, sim.simx_opmode_oneshot_wait)
    sim.simxFinish(clientID)
