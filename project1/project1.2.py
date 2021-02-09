import sys
sys.path.append('../sim_module')
import sim


OBSTACLE_MASTERLIST=['Cylinder','Cylinder0','Cylinder1','Cylinder2','Cuboid','Cuboid0','Cuboid1','Cuboid2']
print ('Program started')
sim.simxFinish(-1) # just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19997,True,True,5000,5) # Connect to CoppeliaSim
if clientID!=-1:
    print ('Connected to remote API server')
    # load the scene
    res=sim.simxLoadScene(clientID,'scene1.2.ttt',True,sim.simx_opmode_blocking)

    #get object information
    # res, objs = sim.simxGetObjects(clientID, sim.sim_handle_all, sim.simx_opmode_blocking)
    res, handles, intData, floatData, names=sim.simxGetObjectGroupData(clientID,sim.sim_appobj_object_type,0,sim.simx_opmode_blocking)

    #get object handles and names
    robot_names=[[i,n] for i,n in enumerate(names) if 'dr20'==n] #might need to do if dr20 IN n
    obstacle_names=[[i,n] for i,n in enumerate(names) if n in OBSTACLE_MASTERLIST]
    start_dummy=[[i,n] for i,n in enumerate(names) if n=='Start']
    end_dummy = [[i, n] for i, n in enumerate(names) if n == 'End']
    print(robot_names)
    print(obstacle_names)
    print(start_dummy)
    print(end_dummy)

    #set variables for path planning
    collisionChecking = 1  # whether collision checking is on or off
    minConfigsForPathPlanningPath = 400  # interpolation states for the OMPL path
    minConfigsForIkPath = 100  # interpolation states for the linear approach path
    maxConfigsForDesiredPose = 10  # we will try to find 10 different states corresponding to the goal pose and order them according to distance from initial state
    maxTrialsForConfigSearch = 300  # a parameter needed for finding appropriate goal states
    searchCount = 2  # how many times OMPL will run for a given task

# Stop simulation:
sim.simxStopSimulation(clientID,sim.simx_opmode_oneshot_wait)
sim.simxFinish(clientID)