import sys

sys.path.append('../sim_module')
import sim

import math
import time
import random

print('Program started')
sim.simxFinish(-1)  # just in case, close all opened connections
clientID = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)  # Connect to CoppeliaSim
if clientID == -1:
    print('Could not connect remote API server!')
    sys.exit(-1)

print('Connected to remote API server')

ENDINGS=['no', 'nope','bye','stop','goodbye','never mind']
CONFIRMS=['yes','yeah','absolutely']
MISTAKE_THRESHOLD=.3
TRANSLATE_CASE={'bathroom':'Bathroom',
                'r2d2':'R2D2',
                'plant':'Plant',
                'blue chair':'Blue_Chair',
                'yellow chair':'Yellow_Chair',
                'table':'Table',
                'motorcycle':'Motorcycle'}

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


    def computePath(handle,location):
        # import pdb; pdb.set_trace()
        initPos, initRot = getAbsolutePose(handle, 'block')

        emptyBuff = bytearray()
        res, retInts, retFloats, retStrings, retBuffer = sim.simxCallScriptFunction(
            clientID,  # client
            'lumibot',  # scriptDescription
            sim.sim_scripttype_childscript,  # scriptHandleOrType
            'computePath',  # functionName
            [],  # ints
            [],  # floats
            [location],  # strings
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


    def followPath(robotHandle, billHandle, path, left_motor_handle, right_motor_handle):
        d = 0.0381  # / 2  # distance between the wheels
        r = 0.0975  # radius of the wheel
        epsilon = 0.08  # distance threshold
        path_length = 0

        for pos, rot in path[::]:
            robot_pos, robotRot = getAbsolutePose(robotHandle, 'block')
            while math.dist(pos, robot_pos) > epsilon:
                ## steer along path
                angle_diff = robotRot[2] - math.atan2(pos[1] - robot_pos[1], pos[0] - robot_pos[0])
                # angle_diff -= math.pi / 2  # robot rotation offset
                angle_diff = math.remainder(angle_diff, 2 * math.pi)  # bound between [-pi,pi]
                if abs(angle_diff) > math.pi / 16:
                    v_des = 0.02
                    w_des = 1.4
                else:
                    v_des = 0.16
                    w_des = 1.8
                w_des *= angle_diff  # toPioneer_p3dx_ultrasonicSensortal desired angular velocity of the robot
                v_right = v_des - d * w_des
                v_left = v_des + d * w_des
                W_right = v_right / r  # angular velocity of the right wheel of the robot
                W_left = v_left / r  # angular velocity of the left wheel of the robot

                ## avoid obstacles
                # sensed = readSensors()
                # if sensed[1] > 0.7 and sensed[2] > 0.7:
                #     W_left = 0
                #     W_right = 0

                ## actuate
                res = sim.simxSetJointTargetVelocity(clientID, right_motor_handle, W_right, sim.simx_opmode_oneshot)
                res = sim.simxSetJointTargetVelocity(clientID, left_motor_handle, W_left, sim.simx_opmode_oneshot)
                path_length += 1
                # iterate time step
                time.sleep(0.025)
                robot_pos, robotRot = getAbsolutePose(robotHandle, 'block')

            bill_pos, billRot = getAbsolutePose(billHandle, 'block')
            emptyBuff = bytearray()
            if math.dist(robot_pos,bill_pos)>.13:
                res, retInts, retFloats, retStrings, retBuffer = sim.simxCallScriptFunction(
                    clientID,  # client
                    'Bill',  # scriptDescription
                    sim.sim_scripttype_childscript,  # scriptHandleOrType
                    'move',  # functionName
                    [],  # ints
                    robot_pos,  # floats
                    [],  # strings
                    emptyBuff,  # buffer
                    sim.simx_opmode_blocking,
                )

        res = sim.simxSetJointTargetVelocity(clientID, right_motor_handle, 0, sim.simx_opmode_oneshot)
        res = sim.simxSetJointTargetVelocity(clientID, left_motor_handle, 0, sim.simx_opmode_oneshot)

        emptyBuff = bytearray()
        while math.dist(robot_pos, bill_pos) > .13:
            res, retInts, retFloats, retStrings, retBuffer = sim.simxCallScriptFunction(
                clientID,  # client
                'Bill',  # scriptDescription
                sim.sim_scripttype_childscript,  # scriptHandleOrType
                'move',  # functionName
                [],  # ints
                robot_pos,  # floats
                [],  # strings
                emptyBuff,  # buffer
                sim.simx_opmode_blocking,
            )
            bill_pos, billRot = getAbsolutePose(billHandle, 'block')


        return path_length


    def getLocFromHuman(dummy_list):
        # import pdb; pdb.set_trace()
        # greet the user and get the goal location
        response = input('Greetings! Where would you like to go?')
        # account for the human typing somewhere not in the env
        while response.lower() not in list(dummy_list.keys()):
            print("I'm sorry. I don't know where that is.")
            response = input('Please choose another location.')
            # end the interaction if the human wants to leave
            if response in ENDINGS:
                print('Goodbye!')
                return None

        # determine if we've heard them properly; if not, change the heard location to something else
        p_heard = random.random()
        if p_heard < MISTAKE_THRESHOLD:
            response = random.choice(list(dummy_list.keys()))

        # make sure you have the correct destination
        confirmation = input("You'd like to go to " + response + '?')
        if confirmation in ENDINGS:
            # if not, get the goal location again
            response = input('Looks like I misheard you. Can you tell me your destination again?')
            # account for the human typing somewhere not in the env
            while response.lower() not in list(dummy_list.keys()):
                print("I'm sorry. I don't know where that is.")
                response = input('Please choose another location.')
                # end the interaction if the human wants to leave
                if response in ENDINGS:
                    print('Goodbye!')
                    return None
        print("Great. Let's go to " + response)
        return response

    def getPathPreferences():
        # ask if they need accomodations
        response = input("Do you have any special mobility accomodations?")
        if response in ENDINGS:
            # if not then plan path
            print('Ok. Planning your path now.')
            return None
        # get the obstacles to avoid
        restrictions = input('Ok. What would you like to avoid?')
        if response in ENDINGS:
            # if the human changes their mind then just plan the path
            print('Ok. Planning your path now.')
            return None
        # list obstacles that the user wants to avoid
        print('Great. Planning a path now that avoids', restrictions)
        restrictions = restrictions.split(',')
        return restrictions

    ### Simulation  ###

    # load the scene
    res = sim.simxLoadScene(clientID, 'maze2.ttt', True, sim.simx_opmode_blocking)
    res = sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot)

    # get handles
    robotHandle = getHandleFromName('lumibot')
    billHandle = getHandleFromName('Bill')
    dummy_list = {'plant':getHandleFromName('Plant'),
                  'motorcycle':getHandleFromName('MotorCycle'),
                  'r2d2':getHandleFromName('R2D2'),
                  'bathroom':getHandleFromName('Bathroom'),
                  'blue chair':getHandleFromName('Blue_Chair'),
                  'yellow chair':getHandleFromName('Yellow_Chair'),
                  'table':getHandleFromName('Table')}
    left_motor_handle = getHandleFromName('lumibot_leftMotor')
    right_motor_handle = getHandleFromName('lumibot_rightMotor')


    # get location from user
    location = getLocFromHuman(dummy_list)
    # key=[d for d in dummy_list.keys() if d==loc.lower()]
    # location=dummy_list[key[0]]
    # get obstacles to avoid from user
    restrictions = getPathPreferences()

    # compute the path
    path, raw_path = computePath(robotHandle, TRANSLATE_CASE[location.lower()])

    # visualize and follow the path
    visualizePath(raw_path)

    # get leading preference from the user
    leader = input('Would you like me to take you there?')
    # make sure the user enters yes or no type responses
    while leader.lower() not in ENDINGS and leader.lower() not in CONFIRMS:
        print("I'm sorry. I don't understand your response.")
        leader = input('Would you like me to take you there?')

    if leader in ENDINGS:
        print('Ok. Goodbye!')
    else:
        print("Ok. Let's go!")
        path_length = followPath(robotHandle, billHandle, path, left_motor_handle, right_motor_handle)
        print('Path Length =', path_length)

    input("Press Enter to end simulation...\n")
except KeyboardInterrupt:
    pass
finally:
    # Stop simulation:
    sim.simxStopSimulation(clientID, sim.simx_opmode_oneshot_wait)
    sim.simxFinish(clientID)
