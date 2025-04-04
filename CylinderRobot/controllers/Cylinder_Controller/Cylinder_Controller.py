from controller import Robot, Motor,DistanceSensor

robot = Robot()

timestep = 64


lw = robot.getDevice('LWM')
lw.setPosition(float('inf'))
lw.setVelocity(0)

rw = robot.getDevice('RWM')
rw.setPosition(float('inf'))
rw.setVelocity(0)


ds = []
dsNames = [
    'DSLeftFront', 'DSRightFront', 
    'DSLeftSide', 'DSRightSide',
]
 
for i in range(4):
    ds.append(robot.getDevice(dsNames[i]))
    ds[i].enable(timestep)
    
    
while robot.step(timestep) != -1:

    leftSpeed  = 5
    rightSpeed = 5

    dsValues = []
    for i in range(len(ds)):
        dsValues.append(ds[i].getValue())
        
    for i in range(len(ds)):
        print(dsNames[i],' : ', dsValues[i])
    print('*********************************')
    
    left_obstacle = dsValues[0] <850 or dsValues[2] <850 
    right_obstacle = dsValues[1] <850 or dsValues[3] <850 
    
    if left_obstacle :
        leftSpeed  = 6
        rightSpeed = 1

    elif right_obstacle:
        leftSpeed  = 1
        rightSpeed = 6
    if dsValues[0]<850 and dsValues[1]<900:
        leftSpeed  = 6
        rightSpeed = 1    
    
            
    lw.setVelocity(leftSpeed)
    rw.setVelocity(rightSpeed)

# Enter here exit cleanup code.
