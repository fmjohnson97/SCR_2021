import sim


sim.simxFinish(-1) # just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19997,True,True,5000,5) # Connect to CoppeliaSim
if clientID!=-1:
    print ('Connected to remote API server')
    # load the scene
    res=sim.simxLoadScene(clientID,'scene1.ttt',True,sim.simx_opmode_blocking)

# Stop simulation:
sim.simxStopSimulation(clientID,sim.simx_opmode_oneshot_wait)
sim.simxFinish(clientID)