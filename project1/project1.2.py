import sys
sys.path.append('../sim_module')
import sim

import time

print('Program started')
sim.simxFinish(-1)  # just in case, close all opened connections
clientID = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)  # Connect to CoppeliaSim
if clientID == -1:
    print('Could not connect remote API server!')
    sys.exit(-1)


def getHandleFromName(name):
    returnCode, handle = sim.simxGetObjectHandle(clientID, name, sim.simx_opmode_blocking)
    print(name, handle)
    if returnCode:
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


print('Connected to remote API server')

# load the scene
res = sim.simxLoadScene(clientID, 'scene1.2.ttt', True, sim.simx_opmode_blocking)
res = sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot)

#get object information
# res, objs = sim.simxGetObjects(clientID, sim.sim_handle_all, sim.simx_opmode_blocking)
# res, handles, intData, floatData, names = sim.simxGetObjectGroupData(
#     clientID, sim.sim_appobj_object_type, 0, sim.simx_opmode_blocking
# )

#get object handles and names
# OBSTACLE_MASTERLIST = ['Cylinder', 'Cylinder0', 'Cylinder1', 'Cylinder2', 'Cuboid', 'Cuboid0', 'Cuboid1', 'Cuboid2']
# robot_names = [[i, n] for i, n in enumerate(names) if n == 'lumibot']
# obstacle_names = [[i, n] for i, n in enumerate(names) if n in OBSTACLE_MASTERLIST]
# start_dummy = [[i, n] for i, n in enumerate(names) if n == 'Start']
# robot_names = [[i, n] for i, n in enumerate(names) if 'dr20' == n]  #might need to do if dr20 IN n
# obstacle_names = [[i, n] for i, n in enumerate(names) if n in OBSTACLE_MASTERLIST]
# start_dummy = [[i, n] for i, n in enumerate(names) if n == 'Start']
# end_dummy = [[i, n] for i, n in enumerate(names) if n == 'End']
# print(robotHandle)
# print(obstacle_names)
# print(start_dummy)
# print(end_dummy)

# get handles
# robotHandle = getHandleFromName('lumibot')
start_dummy = getHandleFromName('Start')
end_dummy = getHandleFromName('End')

# Get the robot initial position and orientation
# time.sleep(5)
startPos, startRot = getAbsolutePose(start_dummy, 'block')
endPos, endRot = getAbsolutePose(end_dummy, 'block')

print(startPos, startRot)
print(endPos, endRot)

# Stop simulation:
input("Press Enter to stop Simulation")
sim.simxStopSimulation(clientID, sim.simx_opmode_oneshot_wait)
sim.simxFinish(clientID)
