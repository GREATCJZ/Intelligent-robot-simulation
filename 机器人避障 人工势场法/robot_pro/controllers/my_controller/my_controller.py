import time
import random
import string
from controller import Robot
from controller import Motor
from controller import PositionSensor
from controller import Supervisor
import math
# create the Robot instance.
robot = Supervisor()

# const define
timestep = int(robot.getBasicTimeStep())
max_speed=6
mass=12
accfactor=3
decfactor=0.9
INFINITY=1000000
large_power=0.5
node=robot.getSelf()
rotation=node.getField("rotation")
tempPosition=node.getPosition()
count=0

def check_speed(speed):
    if speed>max_speed:
        speed=max_speed
    if speed<0:
        speed=0.5
    return speed
    
def forcecal(r,angle):
    r/=1000
    r=1-r
    a=(pow(1/r,2))/mass*accfactor
    a_l=[0,0]
    a_l[0]=a*math.cos(angle)
    a_l[1]=a*math.sin(angle)
    # print("a_l: ",a_l)
    return a_l

# 命名库
# backLeftMotor
# backRightMotor
# frontLeftMotor
# frontRightMotor
# backLeftSensor
# backRightSensor
# frontLeftSensor
# frontRightSensor


#device get
backLeftMotor = robot.getDevice("back left wheel")
backRightMotor = robot.getDevice("back right wheel")
frontLeftMotor = robot.getDevice("front left wheel")
frontRightMotor = robot.getDevice("front right wheel")

backLeftSensor = robot.getDevice("back left wheel sensor")
backRightSensor = robot.getDevice("back right wheel sensor")
frontLeftSensor = robot.getDevice("front left wheel sensor")
frontRightSensor = robot.getDevice("front right wheel sensor")
backLeftSensor.enable(timestep)
backRightSensor.enable(timestep)
frontLeftSensor.enable(timestep)
frontRightSensor.enable(timestep)

gps=robot.getDevice("gps")
gps.enable(timestep)

so_name="so"
so=[]

for i in range(0,16):
    so.append(robot.getDevice(so_name+str(i)))
    so[-1].enable(timestep)
    

#test program
backLeftMotor.setPosition(INFINITY)
frontLeftMotor.setPosition(INFINITY)
backRightMotor.setPosition(INFINITY)
frontRightMotor.setPosition(INFINITY)

# backLeftMotor.setVelocity(0.3*max_speed)
# frontLeftMotor.setVelocity(0.3*max_speed)
# backRightMotor.setVelocity(0.7*max_speed)
# frontRightMotor.setVelocity(0.7*max_speed)

speed_front_left=0
speed_back_left=0
speed_front_right=0
speed_back_right=0

# print("please enter a translation [x, z] which is bounded in [-7,7]")
# x=input("x: ")
# z=input("z: ")
x=-6
z=-3
nowx=0
now_z=0

# Main loop:
# - perform simulation steps until Webots is stopping the controller
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
        a=int(random.uniform(0,4))
        r=[]
        if a==0:
            r=[-1,0,0,1.57]
        elif a==1:
            r=[-0.577,0.577,0.578,2.09]
        elif a==2:
            r=[0,0.707,0.707,-3.14]
        else:
            r=[0.577,0.577,0.578,-2.09]
        rotation.setSFRotation(r)
        count=0
    
    #test
    # print(gps.getValues())
    # print(gps.getSpeed())
    # print(backLeftSensor.getValue())
    # print(frontLeftSensor.getValue())
    # print(backRightSensor.getValue())
    # print(frontRightSensor.getValue())

    if (speed_front_left>2 and speed_front_right>2):
        speed_front_left*=decfactor
        speed_back_left*=decfactor
        speed_front_right*=decfactor
        speed_back_right*=decfactor

    # calculate accelerate
    ace=[0,0]
    
    # print(so[0].getValue())
    
    temp=forcecal(so[7].getValue(),39.5/180*math.pi)
    ace[0]+=temp[0]
    ace[1]+=temp[1]
    temp=forcecal(so[6].getValue(),52.7/180*math.pi)
    ace[0]+=temp[0]
    ace[1]+=temp[1]
    temp=forcecal(so[5].getValue(),67/180*math.pi)
    ace[0]+=temp[0]
    ace[1]+=temp[1]
    temp=forcecal(so[4].getValue(),82.1/180*math.pi)
    ace[0]+=temp[0]
    ace[1]+=temp[1]
    temp=forcecal(so[3].getValue(),97.9/180*math.pi)
    ace[0]+=temp[0]
    ace[1]+=temp[1]
    temp=forcecal(so[2].getValue(),113/180*math.pi)
    ace[0]+=temp[0]
    ace[1]+=temp[1]
    temp=forcecal(so[1].getValue(),127.3/180*math.pi)
    ace[0]+=temp[0]
    ace[1]+=temp[1]
    temp=forcecal(so[0].getValue(),140.5/180*math.pi)
    ace[0]+=temp[0]
    ace[1]+=temp[1]
    temp=forcecal(so[15].getValue(),39.5/180*math.pi+math.pi)
    ace[0]+=temp[0]
    ace[1]+=temp[1]
    temp=forcecal(so[14].getValue(),52.7/180*math.pi+math.pi)
    ace[0]+=temp[0]
    ace[1]+=temp[1]
    temp=forcecal(so[13].getValue(),67/180*math.pi+math.pi)
    ace[0]+=temp[0]
    ace[1]+=temp[1]
    temp=forcecal(so[12].getValue(),82.1/180*math.pi+math.pi)
    ace[0]+=temp[0]
    ace[1]+=temp[1]
    temp=forcecal(so[11].getValue(),97.9/180*math.pi+math.pi)
    ace[0]+=temp[0]
    ace[1]+=temp[1]
    temp=forcecal(so[10].getValue(),113/180*math.pi+math.pi)
    ace[0]+=temp[0]
    ace[1]+=temp[1]
    temp=forcecal(so[9].getValue(),127.3/180*math.pi+math.pi)
    ace[0]+=temp[0]
    ace[1]+=temp[1]
    temp=forcecal(so[8].getValue(),140.5/180*math.pi+math.pi)
    ace[0]+=temp[0]
    ace[1]+=temp[1]
    
    ace[0]/=6
    ace[1]/=16
    ace[0]=-ace[0]
    ace[1]=-ace[1]
    
    temp=gps.getValues()
    nowx=temp[0]
    nowz=temp[2]
    d_x=x-nowx
    d_z=z-nowz
    
    temp_rotation=rotation.getSFRotation()
    angle=0
    if temp_rotation[0]>0:
        angle=temp_rotation[0]*math.pi
        if (temp_rotation[0]>0 and temp_rotation[1]>0) or (temp_rotation[0]<0 and temp_rotation[1]<0):
            angle-=math.pi/2
        else:
            angle=3/2*math.pi-angle
    else:
        angle=-temp_rotation[0]*math.pi
        if (temp_rotation[0]>0 and temp_rotation[1]>0) or (temp_rotation[0]<0 and temp_rotation[1]<0):
            angle-=math.pi/2
        else:
            angle=3/2*math.pi-angle
    
    # print("d_x: ",d_x)
    # print("d_z: ",d_z)
    # print("d_x/d_z: ",math.atan(d_x/d_z))
    if d_x>0 and d_z>0:
        dis_angle=math.atan(d_x/d_z)
    elif d_x<0 and d_z>0:
        dis_angle=math.atan(d_x/d_z)+2*math.pi
    elif d_x>0 and d_z<0:
        dis_angle=math.atan(d_x/d_z)+math.pi
    else:
        dis_angle=math.atan(d_x/d_z)+math.pi
    d_angle=dis_angle-angle+math.pi/2
    d_l=math.sqrt(pow(d_x,2)+pow(d_z,2))
    a=d_l/mass*random.uniform(0.5,4)
    d_ax=a*math.cos(d_angle)*1.5
    d_ay=a*math.sin(d_angle)
    # print("angle: ",angle)
    # print("dis_angle: ",dis_angle)
    # print("d_angle: ",d_angle)
    
    if d_ay<0:
        d_ay=0
        d_ax/=2
    if d_l<0.5:
        print("arrive it!")
        backLeftMotor.setVelocity(0)
        frontLeftMotor.setVelocity(0)
        backRightMotor.setVelocity(0)
        frontRightMotor.setVelocity(0)
        continue
    if d_l<1:
        d_ax*=5
        d_ay*=5

    # print("d_ax: ",d_ax)
    # print("d_ay: ",d_ay)
    
    # -0.775 0.447 0.447 1.82
    # -0.999 -0.0399 -0.027 1.57
    
    # for i in range(0,16):
        # print(i,": ",so[i].getValue())
    # print(ace)
    if speed_front_left<2 and speed_front_right<2 and ace[1]<0:
        pass
    else:
        speed_front_left=(ace[1]+speed_front_left+d_ay)
        speed_back_left=(ace[1]+speed_back_left+d_ay)
        speed_front_right=(ace[1]+speed_front_right+d_ay)
        speed_back_right=(ace[1]+speed_back_right+d_ay)
    
    if ace[0]>0:
        speed_front_left=(ace[0]+speed_front_left)
        speed_back_left=(ace[0]+speed_back_left)
        speed_front_right=(-ace[0]+speed_front_right)
        speed_back_right=(-ace[0]+speed_back_right)
    else :
        speed_front_right=(-ace[0]+speed_front_right)
        speed_back_right=(-ace[0]+speed_back_right)
        speed_front_left=(ace[0]+speed_front_left)
        speed_back_left=(ace[0]+speed_back_left)
        
    if d_ax>0:
        speed_front_left=(d_ax+speed_front_left)
        speed_back_left=(d_ax+speed_back_left)
        speed_front_right=(-d_ax+speed_front_right)
        speed_back_right=(-d_ax+speed_back_right)
        
    else :
        speed_front_right=(-d_ax+speed_front_right)
        speed_back_right=(-d_ax+speed_back_right)
        speed_front_left=(d_ax+speed_front_left)
        speed_back_left=(d_ax+speed_back_left)
    
    speed_front_left=check_speed(speed_front_left)
    speed_back_left=check_speed(speed_back_left)
    speed_front_right=check_speed(speed_front_right)
    speed_back_right=check_speed(speed_back_right)
    
    backLeftMotor.setVelocity(speed_back_left)
    frontLeftMotor.setVelocity(speed_front_left)
    backRightMotor.setVelocity(speed_back_right)
    frontRightMotor.setVelocity(speed_front_right)
    
    # print("backLeftMotor: ",backLeftMotor.getVelocity())
    # print("frontLeftMotor: ",frontLeftMotor.getVelocity())
    # print("backRightMotor: ",backRightMotor.getVelocity())
    # print("frontRightMotor: ",frontRightMotor.getVelocity())


# Enter here exit cleanup code.
