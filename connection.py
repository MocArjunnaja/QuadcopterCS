import sim
import sys
def connect():
    sim.simxFinish(-1) # just in case, close all opened connections
    clientID=sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5) # Connect to CoppeliaSim
    if clientID != -1:
        print ('Connected to remote API server')
        return clientID
    else:
        print ('Failed to connect to remote API server')
        sys.exit(1)
        