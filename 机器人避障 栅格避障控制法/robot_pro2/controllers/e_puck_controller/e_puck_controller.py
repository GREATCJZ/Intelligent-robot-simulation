"""e_puck_controller controller."""
import math
from controller import Supervisor
from controller import Robot
from controller import Lidar
from controller import Motor
import numpy as np
import random
import time

map_l=5
NULL=-1
node_edge=61
ret_edge=map_l/(node_edge-1)

class node_():
    def __init__(self,x,y):
        self.tag=1
        self.parent=NULL
        self.visit=False
        self.trans=[x,y]

# create the Robot instance.
robot = Supervisor()

# define global var
timestep = int(robot.getBasicTimeStep())
INFINITY=1000000
map=[]
for i in range(0,node_edge):
    temp=[]
    for j in range(0,node_edge):
        temp.append(node_(i,j))
        if i==0 or j==0 or i==node_edge-1 or j==node_edge-1:
            temp[-1].tag=0
    map.append(temp.copy())

# for i in range(0,21):
    # temp=[]
    # for j in range(0,21):
        # temp.append(map[i][j].tag)
    # print(temp)
# time.sleep(2)

node=robot.getSelf()
rotation=node.getField("rotation")
translation=node.getField("translation")
max_speed=6
min_speed=2
left_speed=0
right_speed=0
decfactor=0.9
accfactor=1

# goal
goal_x=0
goal_z=1.1
grid_x=int((-goal_x+2.5)/ret_edge+0.5)
grid_z=int((goal_z+2.5)/ret_edge+0.5)
map[grid_z][grid_x].tag=2

# interface function
def calDis(l):
    return l/10000

def calAng():
    global rotation
    temp=rotation.getSFRotation()
    angle=0
    if (temp[1]>0 and temp[3]>0) or (temp[1]<0 and temp[3]<0):
        if temp[3]>0:
             angle=temp[3]-math.pi/2
        else:
            angle=-temp[3]-math.pi/2
    else:
        if temp[3]>0:
            angle=-temp[3]+3/2*math.pi
        else:
            angle=temp[3]+3/2*math.pi
    return angle

def calPos(x,z):
    x=int(x/ret_edge+0.5)
    z=int(z/ret_edge+0.5)
    temp=[x,z]
    return temp

def calHPos(l,t):
    angle=calAng()
    if t=='l':
        angle+=math.pi/2
    elif t=='f':
        angle+=0
    elif t=='r':
        angle-=math.pi/2
    elif t=='lf':
        angle+=math.pi/4
    elif t=='rf':
        angle-=math.pi/4
    else:
        pass
    l=calDis(l)
    if t=='f':
        l+=0.04
    d_x=l*math.cos(angle)
    d_z=l*math.sin(angle)
    now_pos=translation.getSFVec3f()
    now_x=-now_pos[0]+2.5
    now_z=now_pos[2]+2.5
    H_x=now_x+d_x
    H_z=now_z+d_z
    H_pos=calPos(H_x,H_z)
    # print(t,' ',H_pos)
    return H_pos

def mapH(pos):
    global map
    if pos[0]<0 or pos[0]>node_edge-1:
        print("fail! pos[0]: ",pos[0])
    if pos[1]<0 or pos[1]>node_edge-1:
        print("fail! pos[1]: ",pos[1])
    map[pos[1]][pos[0]].tag=0
    return
    
def check_speed(speed):
    global max_speed
    global min_speed
    if speed>max_speed:
        speed=max_speed
    if speed<min_speed:
        speed=min_speed
    return speed


def setV(n):
    global left_speed
    global right_speed
    global translation
    global accfactor
    global decfactor
    now=translation.getSFVec3f()
    pos=[-now[0]+2.5,now[2]+2.5]
    goal_pos=[n[0].trans[1]*ret_edge,n[0].trans[0]*ret_edge]
    v_x=goal_pos[0]-pos[0]
    v_y=goal_pos[1]-pos[1]
    v_l=math.sqrt(pow(v_x,2)+pow(v_y,2))
    v_l*=accfactor
    v_angle=0
    # print("v_x: ",v_x)
    # print("v_y: ",v_y)
    # print("math.atan(v_y/v_x): ",math.atan(v_y/v_x))
    if v_x>0 and v_y>0:
        v_angle=math.atan(v_y/v_x)
    elif v_x<0 and v_y>0:
        v_angle=math.atan(v_y/v_x)+math.pi
    elif v_x>0 and v_y<0:
        v_angle=math.atan(v_y/v_x)+2*math.pi
    elif v_x<0 and v_y<0:
        v_angle=math.atan(v_y/v_x)+math.pi
    else:
        pass
    angle=calAng()
    dis_angle=v_angle-angle+math.pi/2
    v_x=v_l*math.cos(dis_angle)*0.5
    v_y=v_l*math.sin(dis_angle)
    if left_speed<4 and right_speed<4 and v_y<0:
        pass
    else:
        left_speed+=v_y
        right_speed+=v_y
    if v_x>0:
        left_speed+=v_x
        right_speed-=v_x
    else:
        left_speed+=v_x
        right_speed-=v_x
    if left_speed>4 and right_speed>4:
        left_speed*=decfactor
        right_speed*=decfactor
    left_speed=check_speed(left_speed)
    right_speed=check_speed(right_speed)
    
    
    # print("v_x: ",v_x)
    # print("v_y: ",v_y)
    # print("pos: ",pos)
    # print("goal: ",goal_pos)
    # print("left speed: ",left_speed)
    # print("right_speed: ",right_speed)
    speed=[left_speed,right_speed]
    # print("v_angle: ", v_angle)
    # print("angle: ",angle)
    # print("dis_angle: ",dis_angle)
    return speed
    
    

dir=[[-1,0],[1,0],[0,1],[0,-1]]
test_temp=[]
 
def BFS():
    global map
    queue=[]
    path=[]
    visit=[]
    now=translation.getSFVec3f()
    grid_now=calPos(-now[0]+2.5,now[2]+2.5)
    # print("grid now: ",grid_now)
    origin=map[grid_now[1]][grid_now[0]]
    origin.visit==True
    # print("origin: ",origin.trans)
    visit.append(origin)
    queue.append(origin)
    while len(queue)>0:
        vertex=queue.pop(0)
        if vertex.tag==2:
            # print("find")
            while vertex.trans[0]!=origin.trans[0] or vertex.trans[1]!=origin.trans[1]:
                path.append(vertex)
                vertex=vertex.parent
            path.reverse()
            
            # for i in range(0,len(path)):
                # print(path[i].trans)
                
            while len(visit)>0:
                visit[0].visit=False
                visit.pop(0)
            return path
        for i in range(0,4):
            if (vertex.trans[0]+dir[i][0])>node_edge-1 or (vertex.trans[0]+dir[i][0])<0 or (vertex.trans[1]+dir[i][1])>node_edge-1 or (vertex.trans[1]+dir[i][1])<0:
                continue
            temp=map[vertex.trans[0]+dir[i][0]][vertex.trans[1]+dir[i][1]]
            if (temp.visit==False and temp.tag!=0):
                visit.append(temp)
                temp.parent=vertex
                temp.visit=True
                queue.append(temp)
                     
    
        
    while len(visit)>0:
        visit[0].visit=False
        visit.pop()
    return NULL      
            
# get device
so_f=robot.getDevice("distance sensor front")
so_f.enable(timestep)
so_l=robot.getDevice("distance sensor left")
so_l.enable(timestep)
so_r=robot.getDevice("distance sensor right")
so_r.enable(timestep)
so_lf=robot.getDevice("distance sensor leftfront")
so_lf.enable(timestep)
so_rf=robot.getDevice("distance sensor rightfront")
so_rf.enable(timestep)

# lidar_l=robot.getDevice("lidar_l")
# lidar_f=robot.getDevice("lidar_f")
# lidar_r=robot.getDevice("lidar_r")
# lidar_l.enable(timestep)
# lidar_f.enable(timestep)
# lidar_r.enable(timestep)
# lidar_l_width=lidar_l.getHorizontalResolution()
# lidar_f_width=lidar_f.getHorizontalResolution()
# lidar_r_width=lidar_r.getHorizontalResolution()

leftwheel=robot.getDevice("left wheel motor")
rightwheel=robot.getDevice("right wheel motor")
leftwheel.setPosition(INFINITY)
rightwheel.setPosition(INFINITY)


                
# test
# for i in range(0,21):
    # temp=[]
    # for j in range(0,21):
        # temp.append(map[i][j].tag)
    # print(temp)
# time.sleep(2)


# prepare
now_path=BFS()
count=0


# Main loop:
while robot.step(timestep) != -1:
    
    now=translation.getSFVec3f()
    d_l=math.sqrt(pow(now[0]-goal_x,2)+pow(now[2]-goal_z,2))
    if d_l<0.2:
        print("arrive it!")
        leftwheel.setVelocity(0)
        rightwheel.setVelocity(0)
        continue
    
    # test lidar
    # lidar_l_image=lidar_l.getRangeImage()
    # print("lidar_l_image: ",lidar_l_image[int(len(lidar_l_image)/2)])
    # lidar_f_image=lidar_f.getRangeImage()
    # print("lidar_f_image: ",lidar_f_image[int(len(lidar_f_image)/2)])
    # lidar_r_image=lidar_r.getRangeImage()
    # print("lidar_r_image: ",lidar_r_image[int(len(lidar_r_image)/2)])
    # print("l_pos: ",calHPos(lidar_l_image[int(len(lidar_l_image)/2)],'l'))
    # print("f_pos: ",calHPos(lidar_f_image[int(len(lidar_f_image)/2)],'f'))
    # print("r_pos: ",calHPos(lidar_r_image[int(len(lidar_r_image)/2)],'r'))

    count+=1
    temp1=[-1,-1]
    temp2=[-1,-1]
    temp3=[-1,-1]
    temp4=[-1,-1]
    temp4=[-1,-1]
    l=so_f.getValue()
    # print(l)
    if l<20000:
        temp1=calHPos(l,'f')
        mapH(temp1)
    l=so_l.getValue()
    # print(l)
    if l<20000:
        temp2=calHPos(l,'l')
        mapH(temp2)
    l=so_r.getValue()
    # print(l)
    if l<20000:
        temp3=calHPos(l,'r')
        mapH(temp3)
    l=so_lf.getValue()
    # print(l)
    if l<20000:
        temp4=calHPos(l,'lf')
        mapH(temp4)
    l=so_rf.getValue()
    # print(l)
    if l<20000:
        temp5=calHPos(l,'rf')
        mapH(temp5)
    flag=False
    for i in range(0,len(now_path)):
        if (now_path[i].trans[1]==temp1[0] and now_path[i].trans[0]==temp1[1]) or (now_path[i].trans[1]==temp2[0] and now_path[i].trans[0]==temp2[1]) or (now_path[i].trans[1]==temp3[0] and now_path[i].trans[0]==temp3[1]) or (now_path[i].trans[1]==temp4[0] and now_path[i].trans[0]==temp4[1]) or (now_path[i].trans[1]==temp5[0] and now_path[i].trans[0]==temp5[1]):
            flag=True
            break
    if flag or count>500:
        now_path=BFS()
        count=0
    if now_path==NULL:
        map=[]
        for i in range(0,node_edge):
            temp=[]
            for j in range(0,node_edge):
                temp.append(node_(i,j))
                if i==0 or j==0 or i==node_edge-1 or j==node_edge-1:
                    temp[-1].tag=0
            map.append(temp.copy())
        print("update the map\n")

    now=translation.getSFVec3f()
    pos=[-now[0]+2.5,now[2]+2.5]
    goal_pos=[now_path[0].trans[1]*ret_edge,now_path[0].trans[0]*ret_edge]
    v_x=goal_pos[0]-pos[0]
    v_y=goal_pos[1]-pos[1]
    v_l=math.sqrt(pow(v_x,2)+pow(v_y,2))
    # print("v_l: ",v_l)
    if v_l<0.1:
        now_path.pop(0)
    v=setV(now_path)
    leftwheel.setVelocity(v[0])
    rightwheel.setVelocity(v[1])
    
    # print("dis front", so_f.getValue())
    # print("dis left", so_l.getValue())
    # print("dis right", so_r.getValue())
    
    # for i in range(0,len(now_path)):
        # print(now_path[i].trans)
    # print("goal: ",[grid_x,grid_z])
    
    # for i in range(node_edge-1,-1,-1):
        # temp=[]
        # for j in range(0,node_edge):
            # temp.append(map[i][j].tag)
        # print(temp)
    # print(" ")
    
# Enter here exit cleanup code.
