"""hinder_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import math
from controller import Motor
from controller import Lidar
import random
from controller import Supervisor

# create the Robot instance.
robot = Supervisor()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
MAX_SPEED=10.1523
CRUISING_SPEED=8.0
FAR_OBSTACLE_THRESHOLD=0.4
NEAR_OBSTACLE_THRESHOLD=0.6
FAST_DECREASE_FACTOR=0.90
SLOW_DECREASE_FACTOR=0.50
UNUSED_POINT=83
N_SECTOR=5
INFINITY=10000000
node=robot.getSelf()
rotation=node.getField("rotation")
tempPosition=node.getPosition()
count=0
a=int(random.uniform(0,4))
r=[]
if a==0:
    r=[1,0,0,-1.57]
elif a==1:
    r=[-0.577,0.577,0.578,2.09]
elif a==2:
    r=[0,0.707,0.707,3.14]
else:
    r=[-0.577,-0.577,-0.578,2.09]
rotation.setSFRotation(r)

def check_speed(speed):
    if speed>MAX_SPEED:
        speed=MAX_SPEED
    return speed


# get devices
urg04lx=robot.getDevice("Hokuyo URG-04LX-UG01")
left_wheel=robot.getDevice("wheel_left_joint")
right_wheel=robot.getDevice("wheel_right_joint")

# init urg04lx
urg04lx.enable(timestep)
urg04lx_width=urg04lx.getHorizontalResolution()

# print(urg04lx_width)

# defines sector range
sector_range=N_SECTOR*[0]
sector_size=(urg04lx_width-2*UNUSED_POINT-1)/N_SECTOR
for i in range(0,N_SECTOR):
    sector_range[i]=UNUSED_POINT+(i+1)*sector_size

# defines usefull points (above 5.6m / 2.0 = 2.8m, points not used)
max_range=urg04lx.getMaxRange()
range_threshold=max_range/2
urg04lx_values=0

# init motors
left_speed=0
right_speed=0
left_wheel.setPosition(INFINITY)
right_wheel.setPosition(INFINITY)
left_wheel.setVelocity(left_speed)
right_wheel.setVelocity(right_speed)

# init dynamic variables
left_obstacle=0
right_obstacle=0
front_obstacle=0
front_left_obstacle=0
front_right_obstacle=0


# conrtol loop
while robot.step(timestep) != -1:

    temp=node.getPosition()
    flag=False
    # print(temp)
    # print(tempPosition)
    for i in range(0,3):
        if ((temp[i]-tempPosition[i])>0.0005 and temp[i]>tempPosition[i]):
            flag=True
        if ((-temp[i]+tempPosition[i])>0.0005 and temp[i]<tempPosition[i]):
            flag=True
    if flag==False:
        count+=1
    else:
        count=0
        tempPosition=temp
    # print(count)
    if count>=20:
        # east=0.597 0.569 0.566 -2.07
        # north=-0.999 0.0238 0.0246 1.58
        # west=0.56 -0.587 -0.585 -2.13
        # sourth=0.0169 0.708 0.706 -3.11
        
        #  1     0     0    -1.57 1
        # -0.983 0.129 0.130 1.59 2
        # -0.935 0.251 0.251 1.64 3
        # -0.863 0.357 0.358 1.72 4
        # -0.774 0.447 0.447 1.82 5
        # -0.678 0.520 0.520 1.95 6
        # -0.577 0.577 0.578 2.09 7
        # -0.477 0.621 0.622 2.25 8
        # -0.378 0.654 0.655 2.42 9
        # -0.281 0.687 0.679 2.59 10
        # -0.186 0.694 0.695 2.77 11
        # -0.093 0.704 0.704 2.96 12
        
        #  0     0.707 0.707 3.14 1
        #  0.093 0.704 0.704-2.96 2
        # -0.186-0.694-0.695 2.77 3
        # -0.281-0.678-0.679 2.59 4
        # -0.378-0.654-0.655 2.42 5
        # -0.477-0.621-0.622 2.25 6
        # -0.577-0.577-0.5782.09 7
        # -0.678-0.520-0.520 1.95 8
        # -0.774-0.447-0.448 1.82 9
        # -0.863-0.357-0.358 1.72 10
        # -0.935-0.251-0.251 1.64 11
        # -0.983-0.129-0.130 1.59 12
        
        a=int(random.uniform(0,4))
        r=[]
        if a==0:
            r=[1,0,0,-1.57]
        elif a==1:
            r=[-0.577,0.577,0.578,2.09]
        elif a==2:
            r=[0,0.707,0.707,3.14]
        else:
            r=[-0.577,-0.577,-0.578,2.09]
        rotation.setSFRotation(r)
        count=0

    urg04lx_values=urg04lx.getRangeImage()
    
    for i in range(UNUSED_POINT, urg04lx_width - UNUSED_POINT - 1):
        if urg04lx_values[i] < range_threshold:
            if i < sector_range[0]:  # [83-182]
                left_obstacle += (1.0 - urg04lx_values[i] / max_range)
            if sector_range[0] <= i and i < sector_range[1]:  # [183-282]
                front_left_obstacle += (1.0 - urg04lx_values[i] / max_range)
            if sector_range[1] <= i and i < sector_range[2]:  # [283-382]
                front_obstacle += (1.0 - urg04lx_values[i] / max_range)
            if sector_range[2] <= i and i < sector_range[3]:  # [383-482]
                front_right_obstacle += (1.0 - urg04lx_values[i] / max_range)
            if sector_range[3] <= i and i < sector_range[4] + 1:  # [483-583]
                right_obstacle += (1.0 - urg04lx_values[i] / max_range)
    
    left_obstacle /= sector_size
    front_left_obstacle /= sector_size
    front_obstacle /= sector_size
    front_right_obstacle /= sector_size
    right_obstacle /= sector_size
    
    # print("left_obstacle: ",left_obstacle)
    # print("front_left_obstacle: ",front_left_obstacle)
    # print("front_obstacle: ",front_obstacle)
    # print("front_right_obstacle: ",front_right_obstacle)
    # print("right_obstacle: ",right_obstacle)
    
    # compute the speed according to the information on obstacles
    if front_obstacle > NEAR_OBSTACLE_THRESHOLD:
        speed_factor = (1.0 - FAST_DECREASE_FACTOR * front_obstacle) * MAX_SPEED / front_obstacle
        # more obstacles on the right, so make a left u-turn to avoid being stuck
        if right_obstacle > left_obstacle or front_right_obstacle > front_left_obstacle:
            right_speed = (check_speed(speed_factor * front_obstacle)+check_speed(speed_factor * front_obstacle))/2
            left_speed = right_speed/20
        else:
            left_speed = (check_speed(speed_factor * front_obstacle)+check_speed(speed_factor * front_obstacle))/2
            right_speed = left_speed/20
        robot.step(1)
    else:
        if left_obstacle > right_obstacle and left_obstacle > NEAR_OBSTACLE_THRESHOLD:
            speed_factor = (1.0 - FAST_DECREASE_FACTOR * left_obstacle) * MAX_SPEED / left_obstacle
            left_speed = check_speed(speed_factor * left_obstacle)
            if front_obstacle<FAR_OBSTACLE_THRESHOLD:
                right_speed = check_speed(speed_factor * right_obstacle*2)
                left_speed=check_speed(left_speed*2)
            else:
                right_speed = check_speed(speed_factor * right_obstacle)/5
        
        elif right_obstacle > left_obstacle and right_obstacle > NEAR_OBSTACLE_THRESHOLD:
            speed_factor = (1.0 - FAST_DECREASE_FACTOR * right_obstacle) * MAX_SPEED / right_obstacle
            right_speed = check_speed(speed_factor * right_obstacle)
            if front_obstacle<FAR_OBSTACLE_THRESHOLD:
                left_speed = check_speed(speed_factor * left_obstacle*2)
                right_speed=check_speed(right_speed*2)
            else:
                left_speed = check_speed(speed_factor * left_obstacle)/5 
            
        elif front_left_obstacle > front_right_obstacle and front_left_obstacle > FAR_OBSTACLE_THRESHOLD:
            speed_factor = (1.0 - SLOW_DECREASE_FACTOR * front_left_obstacle) * MAX_SPEED / front_left_obstacle
            left_speed = check_speed(speed_factor * front_left_obstacle)
            if front_left_obstacle > NEAR_OBSTACLE_THRESHOLD-0.1:
                right_speed = check_speed(speed_factor * front_right_obstacle)/10
            else:
                right_speed = check_speed(speed_factor * front_right_obstacle)
    
        elif front_right_obstacle > front_left_obstacle and front_right_obstacle > FAR_OBSTACLE_THRESHOLD:
            speed_factor = (1.0 - SLOW_DECREASE_FACTOR * front_right_obstacle) * MAX_SPEED / front_right_obstacle
            if front_right_obstacle > NEAR_OBSTACLE_THRESHOLD-0.1:
                left_speed = check_speed(speed_factor * front_left_obstacle)/10
            else:
                left_speed = check_speed(speed_factor * front_left_obstacle)
            right_speed = check_speed(speed_factor * front_right_obstacle)
        else:
            left_speed = CRUISING_SPEED
            right_speed = CRUISING_SPEED
    
    if left_speed==0:
        left_speed=right_speed/15
    if right_speed==0:
        right_speed=left_speed/15
    
    # set actuators
    left_wheel.setVelocity(left_speed)
    right_wheel.setVelocity(right_speed)

    # print("left speed: ",left_speed)
    # print("right speed: ",right_speed)
    # print(" ")

    #reset dynamic variables to zero
    left_obstacle = 0.0
    front_left_obstacle = 0.0
    front_obstacle = 0.0
    front_right_obstacle = 0.0
    right_obstacle = 0.0
    

# Enter here exit cleanup code.
