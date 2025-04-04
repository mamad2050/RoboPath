from controller import Robot, Motor,DistanceSensor
import math

robot = Robot()

timestep = 64
MAX_SPEED =6.28

lw = robot.getDevice('LWM')
lw.setPosition(float('inf'))
lw.setVelocity(0)

rw = robot.getDevice('RWM')
rw.setPosition(float('inf'))
rw.setVelocity(0)


target = [-1.35,-1.34,0]
current = [0,0,0]
ds = []
dsNames = [
    'DSLeftFront',  
    'DSLeftSide', 
]
 
for i in range(2):
    ds.append(robot.getDevice(dsNames[i]))
    ds[i].enable(timestep)


while robot.step(timestep) != -1:
    
    leftSpeed  = MAX_SPEED
    rightSpeed = MAX_SPEED
    dsValues = []
    for i in range(len(ds)):
        dsValues.append(ds[i].getValue())
        
    for i in range(len(ds)):
        print(dsNames[i],' : ', dsValues[i])
    print('*********************************')
    
    left_wall = dsValues[1] <800 
    front_wall = dsValues[0] <800
    
    if front_wall:
        print('Turn right in place')
        leftSpeed  = MAX_SPEED
        rightSpeed = -MAX_SPEED
        
    else:
        if left_wall:
            print('Drive Forward')
            leftSpeed  = MAX_SPEED
            rightSpeed = MAX_SPEED           
        else:
            print('Turn left')
            leftSpeed  = MAX_SPEED/4
            rightSpeed = MAX_SPEED           
    
    # right_obstacle = dsValues[1] <850 or dsValues[0] <850 
    
    # if left_obstacle :
        # leftSpeed  = 6
        # rightSpeed = 1

    # elif right_obstacle:
        # leftSpeed  = 1
        # rightSpeed = 6
    # if dsValues[0]<750 and dsValues[1]<900:
        # leftSpeed  = 6
        # rightSpeed = 1    
    
 
    lw.setVelocity(leftSpeed)
    rw.setVelocity(rightSpeed)
  