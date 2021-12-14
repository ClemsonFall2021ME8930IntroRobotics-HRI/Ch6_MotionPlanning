#!/usr/bin/env python
# coding: utf-8

# In[164]:


import sim
import time
import cv2
import numpy as np
from PIL import Image
from queue import PriorityQueue
import math
from numpy import sqrt
import itertools


# In[165]:


def resize(img,resolution):
    scale_percent = 500 # percent of original size
    width = int(resolution[1] * scale_percent / 100)
    height = int(resolution[0] * scale_percent / 100)
    dim = (width, height)
    resized = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)
    return resized
def x_w_i(w_x):
    y=((w_x+2.5)*1280)/5
    return round(y)
def y_w_i(w_y):
    x=((2.5-w_y)*1280)/5
    return round(x)
def x_i_w(i_x):
    y=(i_x*5/1280)-2.5
    return y
def y_i_w(i_y):
    x=(2.5)-(i_y*5/1280)
    return x
def draw_rec_red(x,image):
    for i in range(120):
         for j in range(120):
                if (0<x[0]-60+i<1280 and 0<x[1]-60+j<1280):
                    image[x[0]-60+i][x[1]-60+j]=(255,0,0)

def draw_rec_blue(x,image): 
    for i in range(120):
         for j in range(120):
                if (0<x[0]-60+i<1280 and 0<x[1]-60+j<1280):
                    image[x[0]-60+i][x[1]-60+j]=(0,0,255)
                    
                    
def draw_rec_green(x,image):
    for i in range(120):
         for j in range(120):
                if (0<x[0]-60+i<1280 and 0<x[1]-60+j<1280):
                    image[x[0]-60+i][x[1]-60+j]=(0,255,0)
                    
                    
def my_range(start, end, step):
    while start <= end:
        yield start
        start += step

def obstacles_grid(img):
    # getting the walls 
    mask_wall = cv2.inRange(img, np.array([228,228,253]),np.array([230,230,2256]))
    # getting the rims
    mask_rim = cv2.inRange(img, 0, 0)
    mask_total = cv2.bitwise_or(mask_wall,mask_rim,mask_rim)
    mask_total = cv2.bitwise_not(mask_total)
    return mask_total

def distu(x,y):
    dist = sqrt( (x[0] - y[0])**2 + (x[1] - y[1])**2 )
    return round(dist)


def get(array):
    array.sort()
    return array.pop(0)
    

def put(array,data):
    array.append(data)
    array.sort()
    return array

def expand(s):
    aa=[]
    for row in s:
        for a_a in row[1:]:
            aa.append(a_a)
    return aa
def man_dis(x,y):
    x1,y1=x
    x2,y2=y
    return abs(x1-x2)+abs(y1-y2)

def conv(s):
    g=round((s/128)-0.5)
    return g
def incov(s):
    g=(s+0.5)*128
    return g


def neighbours(x,img_obs):
    neigh=[]
    kk=[-128,0,128]
    for i in kk:
        for j in kk:
            if i==0 and j==0:
                continue
            else:
                if (0<(x[0]+i)) and ((x[0]+i)<=1280) and (0<(x[1]+j))and((x[1]+j)<=1280):
                            if img_obs[x[0]+i][x[1]+j]!=0:
                                neigh.append([x[0]+i,x[1]+j])
    return neigh
                            


# In[166]:



sim.simxFinish(-1)
scale_percent =600
clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)
thistime = time.time()
if clientID != -1:
    res_ro, robot = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx', sim.simx_opmode_oneshot_wait)
    res_l,right_motor_handle = sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor',sim.simx_opmode_oneshot_wait)
    res_r,left_motor_handle = sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor',sim.simx_opmode_oneshot_wait)
    res_floor,floor=sim.simxGetObjectHandle(clientID,'ResizableFloor_5_25',sim.simx_opmode_blocking)
    res,camera = sim.simxGetObjectHandle(clientID, 'ss', sim.simx_opmode_oneshot_wait)
    err, resolution, image = sim.simxGetVisionSensorImage(clientID, camera, 0, sim.simx_opmode_streaming)
    while (sim.simxGetConnectionId(clientID) != -1):
        res_pos,pos=sim.simxGetObjectPosition(clientID,robot,-1,sim.simx_opmode_oneshot_wait)
        res_or,orien=sim.simxGetObjectOrientation(clientID,robot,-1,sim.simx_opmode_oneshot_wait)
        print(pos)
        #print(orien)
        err, resolution, image = sim.simxGetVisionSensorImage(clientID, camera, 0, sim.simx_opmode_buffer)
        if err == sim.simx_return_ok:
            img = np.array(image, dtype=np.uint8)
            img.resize([resolution[0], resolution[1], 3]) 
            resized=resize(img,resolution)
            image = Image.fromarray(resized, 'RGB')
        sim.simxFinish(clientID)
img_obs = obstacles_grid(resized)
xs=x_w_i(pos[0])
ys=y_w_i(pos[1])
start=[xs,ys]
goal=[x_w_i(-2.25),y_w_i(2.25)]


# In[167]:


parent=np.array([[[0,0]]*10]*10)
def djikstra(start,goal,resized,img_obs):
    dist = np.array([[np.inf]*10]*10)
    a,b=conv(start[0]),conv(start[1])
    dist[a][b]=0
    closed=[]
    open_l=[]
    for i in range(len(parent)):
        for j in range(len(parent[i])):
            if img_obs[int((i+0.5)*128)][int((j+0.5)*128)]!=0:
                open_l.append([dist[i][j],[(i+0.5)*128,(j+0.5)*128]])
    while open_l:
        print(parent)
        node=get(open_l)
        node_1=node[1]
        node1=[int(node_1[0]),int(node_1[1])]
        draw_rec_red(node1,resized)
        print(node1)
        distance=dist[conv(node1[0])][conv(node1[1])]
        print(distance)
        closed.append(node1)
        nei=neighbours(node1,img_obs)
        print(nei)
        for ne in nei:
            draw_rec_red(ne,resized)
            print(ne)
            if ne not in closed:
                print(distu(node1,ne))
                d=distance+distu(node1,ne)
                if d <  dist[conv(ne[0])][conv(ne[1])]:
                    dist[conv(ne[0])][conv(ne[1])]=d
                    parent[conv(ne[0])][conv(ne[1])]=node1
                    for ro in open_l:
                        if ro[-1]==ne:
                            ro[0]=d
                    
                    
                
        
    
djikstra(start,goal,resized,img_obs)


# In[168]:


print(parent)
image_pp = Image.fromarray(resized)
image_pp



# In[171]:


ccc=[]
n=goal
#print(parent[conv(n[0])][conv(n[1])])
print(start)
draw_rec_green(n,resized)
#print(parent[conv(n[0])],parent[conv(n[1])])
while(True):
    print(n)
    a=parent[int(conv(n[0]))][int(conv(n[1]))]
    print(a)
    a1=[a[0],a[1]]
    if a1==[1216,1216]:
        break
    draw_rec_blue(a1,resized)
    n=a1
    print(n)
image_pp = Image.fromarray(resized)
image_pp


# In[ ]:




