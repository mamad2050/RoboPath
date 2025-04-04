from controller import Robot, Motor,DistanceSensor,GPS,Compass
import math
import random

robot = Robot()


timestep = 64
MAX_SPEED =6.28
speed = 6.28

lw = robot.getDevice('LWM')
lw.setPosition(float('inf'))
lw.setVelocity(0)

rw = robot.getDevice('RWM')
rw.setPosition(float('inf'))
rw.setVelocity(0)

ds = []
dsNames = [
    'DSLeftFront',
    'DSLeftSide',
    'DSRightFront',
    'DSRightSide',

]

for i in range(4):
    ds.append(robot.getDevice(dsNames[i]))
    ds[i].enable(timestep)

gps = robot.getDevice('gps')
gps.enable(timestep)


compass = robot.getDevice('compass')
compass.enable(timestep)

target = [1.13,0.94,0.04]
current =[0,0,0]

delta_x =  target[0]-current[0]
delta_y =  target[1]-current[1]
targetAngle =(math.acos( delta_x / math.sqrt(delta_x **2+delta_y**2))*180)/math.pi
distanceThreshold = 0.1

angleThreshold = math.pi / 4

forwardSpeed = 3
turningSpeed = 2

state = 0
goal_reached = False

start_angle=0
current_angle=0


wall =11

steps =0
Eject = 0

d=0
prev_r = float('inf')


ROBOT_STATE_TURN = 0
ROBOT_STATE_MOVE = 1
ROBOT_STATE_STOP = 2
ROBOT_STATE_WALL = 3
robot_state = ROBOT_STATE_TURN
hitpoint_x = 0
hitpoint_y = 0
init_flag = 1
rand=0
def dist(x1,y1,x2,y2):
    dx = x1 - x2
    dy = y1 - y2
    return math.sqrt(dx * dx + dy * dy)



while robot.step(timestep) != -1 and not goal_reached:
    current =gps.getValues()
    print(current)
    currentCompass=compass.getValues()
    a=math.degrees(math.atan2(target[0], target[1]))
    delta_x =  target[0]-current[0]
    delta_y =  target[1]-current[1]


    dx =delta_x
    dy = delta_y
    r = math.sqrt(dx * dx + dy * dy)
    dest_angle_cos = dx / r
    dest_angle_sin = dy / r

    angle_sin = compass.getValues()[0]
    angle_cos = compass.getValues()[1]

    if init_flag:
        init_flag = 0
        line_a = dy / dx
        line_b = current[1] - current[0] * line_a

    if state ==0:
        start_angle =(math.acos( delta_x / math.sqrt(delta_x **2+delta_y**2))*180)/math.pi
        state+=1
        d = math.sqrt(delta_x **2+delta_y**2)

    else:
        current_angle =(math.acos( delta_x / math.sqrt(delta_x **2+delta_y**2))*180)/math.pi
    print('\nstart possion ',round(start_angle))
    print('target possion ',round(targetAngle))
    print('current ',round((math.acos( delta_x / math.sqrt(delta_x **2+delta_y**2))*180)/math.pi))

    leftSpeed  = MAX_SPEED
    rightSpeed = MAX_SPEED
    dsValues = []
    for i in range(len(ds)):
        dsValues.append(ds[i].getValue())

    for i in range(len(ds)):
        print(dsNames[i],' : ', dsValues[i])
    print('*********************************')

    left_wall = dsValues[1] <800
    front_left_wall = dsValues[0] <800
    front_right_wall= dsValues[2] <800
    right_wall= dsValues[3] <800
    if robot_state == ROBOT_STATE_STOP:

        lw.setVelocity(0)
        rw.setVelocity(0)
        print("THIS IS GOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOLL ‌BROOOOO")
        break
        
    elif robot_state == ROBOT_STATE_MOVE:
        if r < 0.05:
            robot_state = ROBOT_STATE_STOP
        elif prev_r < r:
            robot_state = ROBOT_STATE_TURN
            
        elif front_right_wall or front_left_wall:
            hitpoint_x =current[0]
            hitpoint_y = current[1]
            robot_state = ROBOT_STATE_WALL
            rand =random.choice([0,1])
        else:
            lw.setVelocity(speed)
            rw.setVelocity(speed)
            

    elif robot_state == ROBOT_STATE_TURN:
        if abs(dest_angle_sin - angle_sin) <.17 and abs(dest_angle_cos - angle_cos)<.18:
            robot_state = ROBOT_STATE_MOVE
            lw.setVelocity(speed)
            rw.setVelocity(speed)

        else:
            lw.setVelocity(-speed * 0.35)
            rw.setVelocity(speed * 0.35)

    elif robot_state == ROBOT_STATE_WALL:
        if abs(current[1] - (current[0] * line_a + line_b)) < 0.02 and dist(current[0],current[1],hitpoint_x,hitpoint_y) > 0.02:
            robot_state = ROBOT_STATE_TURN
        right_speed = MAX_SPEED
        left_speed = MAX_SPEED
        
        if rand == 0:
            if front_left_wall:
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

        elif rand==1:
            if front_right_wall:
                print('Turn left in place')
                leftSpeed  = -MAX_SPEED
                rightSpeed = MAX_SPEED

            else:
                if right_wall:
                    print('Drive Forward')
                    leftSpeed  = MAX_SPEED
                    rightSpeed = MAX_SPEED

                else:
                    print('Turn Right')
                    leftSpeed  = MAX_SPEED
                    rightSpeed = MAX_SPEED/4
                    


        lw.setVelocity(leftSpeed)
        rw.setVelocity(rightSpeed)






