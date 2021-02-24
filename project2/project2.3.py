import sys
sys.path.append('../sim_module')
import sim

import math
import time
import numpy as np

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

    ### Simulation  ###

    # load the scene
    res = sim.simxLoadScene(clientID, 'maze.ttt', True, sim.simx_opmode_blocking)
    res = sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot)

    # get handles
    robotHandle = getHandleFromName('Quadricopter')
    robotTarget = getHandleFromName('Quadricopter_target')
    humanHandle = getHandleFromName('Bill')
    robot_waypoints = []
    for i in range(1, 7):
        pos, rot = getAbsolutePose(getHandleFromName('QT' + str(i)), 'block')
        robot_waypoints.append(pos)
    robot_waypoints.insert(-1, robot_waypoints[0])
    # print(robot_waypoints)

    # interpolate waypoints
    robot_path = []
    for p1, p2 in zip(robot_waypoints[:-1], robot_waypoints[1:]):
        num_points = int(50 * math.dist(p1, p2))
        robot_path += list(
            zip(
                np.linspace(p1[0], p2[0], num_points),
                np.linspace(p1[1], p2[1], num_points),
                np.linspace(p1[2], p2[2], num_points),
            )
        )

    # explore waypoints
    epsilon = 0.25  # distance threshold
    for pos in robot_path:
        robot_pos, robotRot = getAbsolutePose(robotHandle, 'block')
        setAbsolutePose(robotTarget, pos, robotRot)
        # print(math.dist(pos, robot_pos))
        while math.dist(pos, robot_pos) > epsilon:
            time.sleep(0.025)
            robot_pos, robotRot = getAbsolutePose(robotHandle, 'block')

    input("Press Enter to end simulation...\n")
except KeyboardInterrupt:
    pass
finally:
    # Stop simulation:
    sim.simxStopSimulation(clientID, sim.simx_opmode_oneshot_wait)
    sim.simxFinish(clientID)
