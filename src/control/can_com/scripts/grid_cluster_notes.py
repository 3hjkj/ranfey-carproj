#!/usr/bin/env python
import message_filters
import rospy
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2

# from numba import jit
import numpy as np
from can_com.msg import autocontrolRadardata
from can_com.msg import autocontrol
from can_com.msg import getmap
from multiprocessing import Pool

# import pandas as pd
import multiprocessing
from multiprocessing import Lock
from multiprocessing import Manager, Queue
from collections import Counter
from multiprocessing import RawArray
from math import pi
from lidarcluster import *
import ros_numpy

# 使用RawArray来存储数据，用于多线程之间的数据传输，全局变量，
global Data

Data = RawArray("i", 270000)

global T

T = np.array(
    [
        [0.999988238126431, 0.00485, 0.00001, -0.010652],
        [-0.00485, 0.999988, -0.000000629942203510149, 0.237046],
        [-0.00001, 0.000000629942203510149, 1, 0.0000437469717837757],
        [0, 0, 0, 1],
    ]
)
# 全局变量，存储参数
global param
# 初始值全部置为0
param = np.mat([[0, 0, 0, 0, 0, 0, 0, 0]])
# 状态转移矩阵
H = np.array([[1.0, 0, 0, 0], [0, 1.0, 0, 0], [0, 0, 1.0, 0], [0, 0, 0, 1.0]])

# 量测噪声矩阵
lidar_R = np.array(
    [[0.0225, 0, 0, 0], [0, 0.0225, 0, 0], [0, 0, 100, 0], [0, 0, 0, 100]]
)
# 协方差矩阵
lidar_P = np.array([[1.0, 0, 0, 0], [0, 1.0, 0, 0], [0, 0, 100.0, 0], [0, 0, 0, 100.0]])

"""
def get_map(arg):
    left_lane_dis=arg.left_lane_dis
    right_lane_dis=arg.right_lane_dis
    left_road_dis=arg.left_road_dis
    right_road_dis=arg.right_road_dis
    return left_lane_dis,right_lane_dis,left_road_dis,right_road_dis


def get_lane(arg):
    nearleft=arg.lane_left_indv_a0
    nearright=arg.lane_right_indv_a0
    neileft=arg.lane_left_neig_a0
    neiright=arg.lane_right_neig_a0
    return abs(nearleft),abs(nearright),abs(neileft),abs(neiright)




#@jit(nopython=True,nogil=True)
def downsample_circle(xel, idex):
    points_update2 = np.zeros((len(idex),3))
    for i in range(len(idex)):
        if i+1 < len(idex): tmp = xel[idex[i]:idex[i+1]]
        else: tmp = xel[idex[i]:]
        if len(tmp) > 0: points_update2[i] = [tmp[:,0].mean(), tmp[:,1].mean(), tmp[:,2].mean()]
    return points_update2


#@jit(parallel=True)
def get_downsample(points_update, Cellsize=0.1):
    voxel=np.zeros((len(points_update),6))
    voxel[:,3:]=points_update[:, 0:3] 
    voxel[:,0:3]=np.ceil((voxel[:,3:])/Cellsize) 
    voxel = voxel[np.lexsort([voxel[:,2],voxel[:,1],voxel[:,0]])]
    vo, xel = voxel[:,0:3], voxel[:, 3:]        
    idex = np.where(np.insert(np.any(np.diff(vo, axis=0), axis=1), 0, True)==True)[0] 
    points_update2 = downsample_circle(xel, idex) 
    return points_update2[points_update2.any(axis=1)]


#@jit(nopython=True,nogil=True)
def grid_circle(voxel, idex): 
    points_update3 = np.zeros((len(voxel),6)) 
    for i in range(len(idex)):
        if i+1 < len(idex): tmp = voxel[idex[i]:idex[i+1]]
        else: tmp = voxel[idex[i]:]
        zmax, zmin = np.max(tmp[:,4]), np.min(tmp[:,4]) 
        if 0.3 <= zmax-zmin <= 3: 
            points_update3[idex[i]:idex[i]+len(tmp)] = np.hstack((tmp, zmax * np.ones((len(tmp),1))))
    return points_update3


def grid(points_update2, Cell=0.4):
    voxel = np.zeros((len(points_update2),5))
    voxel[:,2:]=points_update2 
    voxel[:,0]=np.ceil((voxel[:,2])/Cell) 
    voxel[:,1]=np.ceil((voxel[:,3])/Cell) 
    voxel = voxel[np.lexsort([voxel[:,1],voxel[:,0]])] 
    idex = np.where(np.insert(np.any(np.diff(voxel[:,[0,1]], axis=0), axis=1), 0, True)==True)[0]
    points_update3 = grid_circle(voxel, idex)
    return points_update3[points_update3.any(axis=1)]


#@jit(nopython=True,cache=True,nogil=True)
def get_object(item,ti):
    xmin,xmax=np.min(item[:,0]),np.max(item[:,0])
    ymin,ymax=np.min(item[:,1]),np.max(item[:,1])
    zmin,zmax=np.min(item[:,2]),np.max(item[:,2])
    length=xmax-xmin
    wide=ymax-ymin
    high=zmax-zmin
    x_1=(xmax+xmin)/2
    y=(ymax+ymin)/2
    return [ti,x_1,y,wide,length,high]

#@jit(nopython=True,cache=True,nogil=True)
def grid_1(usepoints):
    x,y=usepoints[0,0],usepoints[0,1]
    idx=np.where((usepoints[:,0]==x)&(usepoints[:,1]==y))[0]
    base=usepoints[idx,:]
    return idx,base,x,y
#@jit(nopython=True,cache=True,nogil=True)
def grid_2(usepoints,base,didx):
    infected_points=usepoints[didx,:]
    idx1=np.where(np.abs(infected_points[:,5]-base[0,5])<0.6)[0]
    return idx1
#@jit(nopython=True,cache=True,nogil=True)
def grid_3(usepoints,didx,idx1,base):
    border_points=usepoints[didx[idx1],:]
    base=np.concatenate((base,border_points),axis=0)
    return base,border_points
#@jit(nopython=True,cache=True,nogil=True)
def grid_4(usepoints,keep,i):
    x,y=keep[i,0],keep[i,1]
    didx=np.where((usepoints[:,0]>=x-1)&(usepoints[:,0]<=x+1)&(usepoints[:,1]>=y-1)&(usepoints[:,1]<=y+1))[0]
    return x,y,didx
#@jit(nopython=True,cache=True,nogil=True)
def grid_5(usepoints,didx,base,x,y):
    infected_points=usepoints[didx,:]
    a=base[np.where((base[:,0]==x)&(base[:,1]==y))[0],5][0]
    idx1=np.where(np.abs(infected_points[:,5]-a)<0.6)[0]
    return idx1
#@jit(nopython=True,cache=True,nogil=True)
def grid_6(usepoints,base,didx,idx1):
    border_points=usepoints[didx[idx1],:]
    base=np.concatenate((base,border_points),axis=0)
    return base,border_points
#@jit(nopython=True,cache=True,nogil=True)
def grid_7(usepoints,x,y):
    didx=np.where((usepoints[:,0]>=x-1)&(usepoints[:,0]<=x+1)&(usepoints[:,1]>=y-1)&(usepoints[:,1]<=y+1))[0]
    return didx

def grid_8(c):
    x=c[:,0]+c[:,1]*1j
    idx=np.unique(x,return_index=True)[1]
    return c[idx]

def grid_cluster(usepoints,ti):
    result=np.zeros((1,6))
    while len(usepoints)>1:
        idx,base,x,y=grid_1(usepoints)
        usepoints=np.delete(usepoints,idx, 0)
        didx=grid_7(usepoints,x,y)
        if len(didx)>0:
            idx1=grid_2(usepoints,base,didx)
            if len(idx1)>0:
                base,border_points=grid_3(usepoints,didx,idx1,base)
                usepoints=np.delete(usepoints,didx[idx1],0)
                keep = grid_8(border_points[:,:2])
            else:
                keep=[]
        else:
            keep=[]
        while len(keep)>0:
            ke=np.zeros((1,2))
            for i in range(len(keep)):
                x,y,didx=grid_4(usepoints,keep,i)
                if len(didx)>0:
                    idx1=grid_5(usepoints,didx,base,x,y)
                    if len(idx1)>0:
                        base,border_points=grid_6(usepoints,base,didx,idx1)
                        usepoints=np.delete(usepoints,didx[idx1],0)
                        e=grid_8(border_points[:,:2])
                        ke=np.concatenate((ke,e),axis=0)
            keep=np.delete(ke,0,0)
        if np.min(base[:,4])<0.2:
            obj=np.mat(get_object(base[:,2:5],ti))
            result=np.concatenate((result,obj),axis=0)
    result=np.delete(result,0,0)
    return result

def get_result(result):
    left,right=1.4,1.4
    a=np.zeros((len(result),1)) 
    r=np.zeros((1,7))
    result=np.column_stack((a,result))
    result[:,0]=np.sqrt(np.square(result[:,2])+np.square(result[:,3]))
    leftfront=result[np.where((result[:,2]>0)&(result[:,3]>left))[0],:]
    if len(leftfront)>0:
        leftfront=leftfront[np.argmin(leftfront[:,0]),:]
        r=np.concatenate((r,leftfront),axis=0)
    front=result[np.where((result[:,2]>0)&(result[:,3]<=left)&(result[:,3]>=-right))[0],:]
    if len(front)>0:
        front=front[np.argmin(front[:,0]),:]
        r=np.concatenate((r,front),axis=0)
    rightfront=result[np.where((result[:,2]>0)&(result[:,3]<=-right))[0],:]
    if len(rightfront)>0:
        rightfront=rightfront[np.argmin(rightfront[:,0]),:]
        r=np.concatenate((r,rightfront),axis=0)
    leftback=result[np.where((result[:,2]<=5)&(result[:,3]>=left))[0],:]
    if len(leftback)>0:
        leftback=leftback[np.argmin(leftback[:,0]),:]
        r=np.concatenate((r,leftback),axis=0)
    back=result[np.where((result[:,2]<=5)&(result[:,3]<=left)&(result[:,3]>=-right))[0],:]
    if len(back)>0:
        back=back[np.argmin(back[:,0]),:]
        r=np.concatenate((r,back),axis=0)
    rightback=result[np.where((result[:,2]<=5)&(result[:,3]<=-right))[0],:]
    if len(rightback)>0:
        rightback=rightback[np.argmin(rightback[:,0]),:]
        r=np.concatenate((r,rightback),axis=0)
    
    left_1=result[np.where((result[:,2]<0)&(result[:,2]>-3.24)&(result[:,3]>left))[0],:]
    if len(left_1)>0:
        left_1=left_1[np.argmin(left_1[:,0]),:]
        r=np.concatenate((r,left_1),axis=0)
    
    right_1=result[np.where((result[:,2]<0)&(result[:,2]>-3.24)&(result[:,3]<=-right))[0],:]
    if len(right_1)>0:
        right_1=right_1[np.argmin(right_1[:,0]),:]
        r=np.concatenate((r,right_1),axis=0)
    
    r=r[1:,1:]
    return r



#@jit(nopython=True,cache=True,nogil=True)
def updateQ(Q,dt):
    dt2 = dt * dt
    dt3 = dt * dt2
    dt4 = dt * dt3
    x_1, y = 3,3
    Q[0,0]= dt4 * x_1 /4
    Q[0,2]= dt3 * x_1 /2
    Q[1,1]= dt4 * y /4
    Q[1,3]= dt3 * y /2
    Q[2,0]= dt3 * x_1 /2 
    Q[2,2]= dt2 * x_1
    Q[3,1]= dt3 * y /2
    Q[3,3]= dt2 * y
    return Q
#@jit(nopython=True,cache=True,nogil=True)
def updateF(F,dt):
    F[0, 2], F[1, 3]  = dt, dt
    return F

#@jit(nopython=True,cache=True,nogil=True)
def predict(x_1,F,P,Q):  
    x_1= np.dot(F,x_1)
    a=np.dot(F,P)
    P = np.dot(a,F.T)+Q
    return x_1,P

#jit(nopython=True,cache=True,nogil=True)
def get_S(H,R,P):
    #PHt = P * H.T
    #S = H * PHt + R 
    PHt=np.dot(P,H.T)
    S=np.dot(H,PHt)+R
    S=np.linalg.inv(S)
    return S,PHt

#@jit(nopython=True,cache=True,nogil=True)
def get_P(x_1,z,H,K,P,G):
    x_1=x_1+np.dot(K,(z-np.dot(H,x_1)))
    P=np.dot((G-np.dot(K,H)),P)
    return x_1,P
#@jit(nopython=True,cache=True,nogil=True)
def update_optimal(x_1,z,H,R,P,G):
    S,PHt=get_S(H,R,P)
    K = np.dot(PHt,S)
    x_1,P=get_P(x_1,z,H,K,P,G)
    return x_1,P

#@jit(nopython=True,cache=True,nogil=True)
def get_L2(x1,y1,x2,y2):
    dx=x1-x2
    dy=y1-y2
    d=np.sqrt(dx**2+dy**2)
    return d

#@jit(nopython=True,cache=True,nogil=True)
def cosine_similarity(vector1, vector2):
    dot_product = 0.0
    normA = 0.0
    normB = 0.0
    for a, b in zip(vector1, vector2):
        dot_product += a * b
        normA += a ** 2
        normB += b ** 2
    if normA == 0.0 or normB == 0.0:
        return 0
    else:
        return (dot_product / ((normA**0.5)*(normB**0.5)))

def get_initialize(m,counterid,lidar_P,ti):
    x_1=[]
    id_value,p_value=[],[]
    for i in range(len(m)):
        m1=m[i,:].T
        m2=[ti,counterid[0],np.float64(m1[1]),np.float64(m1[2]),0,0,np.float64(m1[3]),np.float64(m1[4]),np.float64(m1[5]),0,1]
        p_value.append(lidar_P)
        id_value.append(counterid[0])
        del(counterid[0])
        x_1.append(m2)
    x_1=np.mat(np.float64(x_1))
    return x_1,counterid,id_value,p_value

def matched(a,b):
    index1=np.argmin(a[:,0])
    index2=np.argmax(a[:,1])
    if index1==index2:
        idx=b[0][index1]
        cos=a[index1,0]
    else:
        if a[index1,2]<a[index2,2]:
            idx=b[0][index1]
            cos=a[index1,0]
        else:
            idx=b[0][index2]
            cos=a[index2,0]
    return idx,cos

#@jit(nopython=True,nogil=True)
def get_para(x_1,d):
    para=np.zeros((len(d),4))
    for i in range(len(d)):
        para[i,0]=get_L2(x_1[2,0],x_1[3,0],d[i,1],d[i,2])
        para[i,1]=cosine_similarity([x_1[2,0],x_1[3,0]],[d[i,1],d[i,2]])
        para[i,2]=abs(x_1[3,0]-d[i,2])
        para[i,3]=abs(x_1[2,0]-d[i,1])
    return para

def get_match(x_1,d):
    #x_1=np.mat(x_1).T
    para=get_para(x_1,d)
    #thr=abs(x_1[2#,0])*0.1
    #thr=max(thr,2.5)
    b=np.where((abs(para[:,3])<3)&(abs(para[:,2])<1.5))
    a=para[b]
    if len(a)==0:
        idx=np.nan
        cos=np.nan
    else:
        idx,cos=matched(a,b)
    return idx,cos

def get_matched(m,r):
    me1=[]
    useid=[]
    for i in range(len(m)):
        m1=m[i,:].T
        idx,cos=get_match(m1,r)
        m1=np.row_stack((m1,[idx]))
        m1=np.row_stack((m1,[cos])).T
        me1.append(m1)
        if np.isnan(idx)==False:
            useid.append(idx)
    me1=np.mat(np.float64(me1))
    b = dict(Counter(useid))
    b1=[key for key,value in b.items()if value > 1]
    if len(b1)!=0:
        for i in range(len(b1)):
            key=b1[i]
            num=np.where(me1[:,-2]==key)[0]
            d=me1[num,]
            index1=np.argmin(d[:,-1])
            for j in range(len(d)):
                if j !=index1:
                    me1[num[j],-2]=np.nan
    me1=me1[:,:-1]
    return me1,useid

def get_value(x1):
    if x1[0,10]<50:
        value=2
    else:
        if x1[0,10]>150:
            value=5
        else:
            value=3
    return value

def update_lidar(r2,counterid,id_value,p_value,P1):
    r3=[]
    for i in range(len(r2)):
        m1=r2[i,:]
        m2=[m1[0,0],counterid[0],np.float64(m1[0,1]),np.float64(m1[0,2]),0,0,np.float64(m1[0,3]),np.float64(m1[0,4]),np.float64(m1[0,5]),0,1]
        
        id_value.append(counterid[0])
        p_value.append(P1)
        del(counterid[0])
        r3.append(m2)
    r3=np.mat(np.float64(r3))
    return r3,counterid,id_value,p_value

def get_connect(r,x_1):
    try:
        l=np.concatenate((r,x_1),axis=0)
    except:
        l=x_1
    return l

def get_single_lidar(match,r,useid,id_value,p_value,counterid,F,Q,lidar_R,G,H,lidar_P):
    r1=np.delete(r,useid,axis=0)
    try:
        frame=r[0,0]
    except:
        print('NO OBJECT!')
    x_1=[]
    for i in range(len(match)):
        if np.isnan(match[i][0,-1])==True:
            x1=np.float64(match[i][0,:-1])
            idx=np.where(id_value==x1[0,1])[0][0]
            P=p_value[idx]
            x2=x1[0,2:6].T
            x2,P=predict(x2,F,P,Q)
            x1[0,0],x1[0,2],x1[0,3],x1[0,4],x1[0,5]=frame,x2[0,0],x2[1,0],x2[2,0],x2[3,0]
            
            x1[0,9]=x1[0,9]+1
            if (x1[0,10]<255):
                x1[0,10]=x1[0,10]+1  
            value=get_value(x1)
            if x1[0,9]<value:
                x_1.append(x1)
                p_value[idx]=P
            else:
                del id_value[idx]
                del p_value[idx] 
                counterid.append(int(x1[0,1]))
        else:
            x1=match[i][0,:]
            idx=np.where(id_value==x1[0,1])[0][0]
            P=p_value[idx]
            x2=x1[0,2:6].T
            x2,P=predict(x2,F,P,Q)
            zm=r[int(x1[0,-1]),:]
            z=zm[0,1:3].T
            z=np.row_stack((z,[0]))
            z=np.row_stack((z,[0]))
            x2,P=update_optimal(x2,z,H,lidar_R,lidar_P,G)
            p_value[idx]=P
            x1[0,4]=((x2[0,0]-x1[0,2])/F[0,2])
            x1[0,5]=((x2[1,0]-x1[0,3])/F[0,2])
            x1[0,0],x1[0,2],x1[0,3]=frame,x2[0,0],x2[1,0]
            x1[0,6],x1[0,7],x1[0,8]=zm[0,3],zm[0,4],zm[0,5]
            
            x1=x1[0,:-1]
            x1[0,9]=0
            if (x1[0,10]<255):
                x1[0,10]=x1[0,10]+1 
            x_1.append(x1)
    x_1=np.mat(np.float64(x_1))
    if len(r1)==0:
        l=x_1
    else:
        r2,counterid,id_value,p_value=update_lidar(r1,counterid,id_value,p_value,lidar_P)
        #l=np.concatenate((r2,x),axis=0)
        l=get_connect(r2,x_1)
    return l,id_value,p_value,counterid

def unique(c):
    #c=np.array(c)
    x=c[:,0]+c[:,1]*1j
    idx=np.unique(x,return_index=True)[1]
    return c[idx]

"""

"""

def get_x(x,delta,trans_mat,head_angle,vehicle_x,vehicle_y):
    left,right=1.4,1.4
    x=x[:,2:7]
    a=np.zeros((len(x),1)) 
    r=np.zeros((1,6))
    t=np.zeros((1,6))
    result=np.column_stack((a,x))
    result[:,0]=np.sqrt(np.square(result[:,1])+np.square(result[:,2]))
    leftfront=result[np.where((result[:,1]>0)&(result[:,2]>left))[0],:]
    if len(leftfront)>0:
        leftfront=leftfront[np.argmin(leftfront[:,0]),:]
        r=np.concatenate((r,leftfront),axis=0)  
    else:
        r=np.concatenate((r,t),axis=0)
    front=result[np.where((result[:,1]>=0)&(result[:,2]<=left)&(result[:,2]>=-right))[0],:]
    if len(front)>0:
        front=front[np.argmin(front[:,0]),:]
        r=np.concatenate((r,front),axis=0)
    else:
        r=np.concatenate((r,t),axis=0)
    rightfront=result[np.where((result[:,1]>=0)&(result[:,2]<=-right))[0],:]
    if len(rightfront)>0:
        rightfront=rightfront[np.argmin(rightfront[:,0]),:]
        r=np.concatenate((r,rightfront),axis=0)
    else:
        r=np.concatenate((r,t),axis=0)
    leftback=result[np.where((result[:,1]<=0)&(result[:,2]>=left))[0],:]
    if len(leftback)>0:
        leftback=leftback[np.argmin(leftback[:,0]),:]
        r=np.concatenate((r,leftback),axis=0)
    else:
        r=np.concatenate((r,t),axis=0)
    back=result[np.where((result[:,1]<=0)&(result[:,2]<=left)&(result[:,2]>=-right))[0],:]
    if len(back)>0:
        back=back[np.argmin(back[:,0]),:]
        r=np.concatenate((r,back),axis=0)
    else:
        r=np.concatenate((r,t),axis=0)
    rightback=result[np.where((result[:,1]<=0)&(result[:,2]<=-right))[0],:]
    if len(rightback)>0:
        rightback=rightback[np.argmin(rightback[:,0]),:]
        r=np.concatenate((r,rightback),axis=0)
    else:
        r=np.concatenate((r,t),axis=0)
    
    
    
    left_1=result[np.where((result[:,1]<0)&(result[:,1]>-5)&(result[:,2]>left))[0],:]
    if len(left_1)>0:
        left_1=left_1[np.argmin(left_1[:,0]),:]
        r=np.concatenate((r,left_1),axis=0)
    else:
        r=np.concatenate((r,t),axis=0)
    right_1=result[np.where((result[:,1]<0)&(result[:,1]>-5)&(result[:,2]<=-right))[0],:]
    if len(right_1)>0:
        right_1=right_1[np.argmin(right_1[:,0]),:]
        r=np.concatenate((r,right_1),axis=0)
    else:
        r=np.concatenate((r,t),axis=0)
    r=r[1:,1:]
    
    
    b=[]
    for i in range(len(trans_mat)):
      if (trans_mat[i,:]==0).all()==False:
         b.append(i)
    trans=trans_mat[b,:]
    if len(trans)>0:
       trans_left=np.array(trans[:,:2])
       trans_right=np.array(trans[:,2:])
       trans_left=unique(trans_left)
       trans_right=unique(trans_right)
       coun_trans=len(trans_left)
    else:
       coun_trans=0
    trans_result=result[np.where((result[:,1]>0))[0],:]
    curve_delta=abs(delta)
    if (coun_trans>3)&(len(trans_result)>0)&(curve_delta>5):
        head_angle=head_angle*pi/180
        vehicle_x=-vehicle_x
        trans_left[:,0]=-trans_left[:,0]-vehicle_x
        trans_left[:,1]=trans_left[:,1]-vehicle_y
        trans_right[:,0]=-trans_right[:,0]-vehicle_x
        trans_right[:,1]=trans_right[:,1]-vehicle_y
        trans_left=np.array(trans_left.T)
        trans_right=np.array(trans_right.T)
        dicv=np.array([[np.cos(head_angle),np.sin(head_angle)],
                       [-np.sin(head_angle),np.cos(head_angle)]])
        trans_left=np.dot(dicv,trans_left).T
        trans_right=np.dot(dicv,trans_right).T
        p_left=np.poly1d(np.polyfit(trans_left[:,1],trans_left[:,0],2))#x~y
        p_right=np.poly1d(np.polyfit(trans_right[:,1],trans_right[:,0],2))#x~y
        
        tran=np.array(np.zeros((len(trans_result),4)))
        tran[:,:2]=trans_result[:,[1,2]]#1,y;2,x
        tran[:,2]=p_left(tran[:,0])
        tran[:,3]=p_right(tran[:,0])
        count=[]
        for i in range(len(tran)):
           if (tran[i,1]>=tran[i,3])&(tran[i,1]<=tran[i,2]):
              count.append(i)
        res=trans_result[count,:]
        if len(res)>0:
          rt1=res[np.argmin(res[:,0]),1:]
        else:
          rt1=[]
    else:
        gamma=delta*pi/180
        dicv=np.array([[np.cos(gamma),np.sin(gamma)],
                       [-np.sin(gamma),np.cos(gamma)]])
        p=result[:,1:3]
        pw=p.T
        tp=np.dot(dicv,pw).T
        result[:,0]=np.sqrt(np.square(tp[:,0])+np.square(tp[:,1]))
        res=result[np.where((tp[:,0]>0)&(tp[:,1]<=1)&(tp[:,1]>=-1))[0],:]
        if len(res)>0:
           rt1=res[np.argmin(res[:,0]),1:]
        else:
           rt1=[]
    return r,rt1

def get_me(arg):
    param=np.mat([[0,0,0,0,0,0,0,0]])
    param[0,0]=arg.heading_delta_angle
    param[0,1]=arg.v_obj_pointx2
    param[0,2]=arg.v_obj_pointy2
    param[0,3]=arg.v_obj_pointx1
    param[0,4]=arg.v_obj_pointy1
    param[0,5]=arg.vehicle_heading
    param[0,6]=arg.vehicle_x
    param[0,7]=arg.vehicle_y
    return param#delta,right_x,right_y,left_x,left_y,head_angle,vehicle_x,vehicle_y

#@jit(nopython=True,nogil=True)
def transfrom(points):
    
    x_min=-20
    x_max=50
    y_min=-5
    y_max=5
    z_max=1
    
    car_min_y=-0.9
    car_max_y=0.9
    
    car_min_x=-2
    car_max_x=2.55
    #points[:,0]=-points[:,0]
    #points[:,1]=-points[:,1]
    points=points[np.where((points[:,0]>x_min)&(points[:,0]<x_max)&(points[:,1]>y_min)&
                                       (points[:,1]<y_max)&(points[:,2]<z_max))[0],:]
    points=points[np.where(
            ~((points[:,0]>=car_min_x)&(points[:,0]<= car_max_x)&
              (points[:,1]>=car_min_y)&(points[:,1]<= car_max_y)))[0],:]
    points[:,0]=points[:,0]-2.55
    return points
"""


# 从can_com_pub 里面接收数据，然后赋值
def get_me(arg):
    param = np.mat([[0, 0, 0, 0, 0, 0, 0, 0]])
    param[0, 0] = arg.heading_delta_angle
    param[0, 1] = arg.v_obj_pointx2
    param[0, 2] = arg.v_obj_pointy2
    param[0, 3] = arg.v_obj_pointx1
    param[0, 4] = arg.v_obj_pointy1
    param[0, 5] = arg.vehicle_heading
    param[0, 6] = arg.vehicle_x
    param[0, 7] = arg.vehicle_y
    return param  # delta,right_x,right_y,left_x,left_y,head_angle,vehicle_x,vehicle_y


def timer_callback(event):
    global x
    global counterid
    global id_value
    global p_value
    global t
    global F
    global Q
    global G
    global result
    global frame
    global param
    global trans_mat
    # print(Data[-2])
    data = np.ctypeslib.as_array(Data)
    # frame是每一帧的计数，存储上一帧的值，如果和这一帧的帧数不相同，则进行后面的操作
    if frame != data[-2]:
        # 获取当前时间
        ti = rospy.get_time()
        # 从全局变量中获取参数
        para = param
        delta, right_x, right_y, left_x, left_y, head_angle, vehicle_x, vehicle_y = (
            para[0, 0],
            para[0, 1],
            para[0, 2],
            para[0, 3],
            para[0, 4],
            para[0, 5],
            para[0, 6],
            para[0, 7],
        )
        now_point = np.mat([[left_x, left_y, right_x, right_y]])
        # 把预瞄点左右两侧的点与trans_mat的点合并
        trans_mat = np.concatenate((now_point, trans_mat), axis=0)
        # 取前20个点
        trans_mat = trans_mat[:20, :]
        # 从data中得到长度
        length = data[-1]
        # 从data中得到帧数
        frame = data[-2]
        # 从data中得到点云，并且展开成n×3的形式，类型转为int16
        # print(frame,count,length)
        points = np.int16(data[:length].reshape(length / 3, 3))
        # 此时点云数据是乘以100后的，单位为厘米，所以此时降采样的参数为10
        points = get_downsample(points, Cellsize=10)
        # 点云栅格化操作
        usepoints = grid(points, Cell=50)
        # 如果栅格化之后的点大于0
        if len(usepoints) > 0:
            # 进行区域生长算法聚类
            result = np.asarray(grid_cluster(usepoints, ti))
            # 筛选八个区域的最近目标物，此时的单位为m
            result = get_result(result)
        else:
            # 否则result为空
            result = []
        if len(x) == 0:
            # 如果融合层为空，并且聚类出来的目标物不为空
            if len(result) > 0:
                # 初始化目标物信息
                x, counterid, id_value, p_value = get_initialize(
                    result, counterid, lidar_P, ti
                )
            else:
                x = []
            # 把当前帧时间存储下来
            t = ti
        else:
            # 否则计算前后帧的时间差
            dt = ti - t
            # 存储当前帧时间
            t = ti
            # 更新状态转移方程
            F_m = updateF(F, dt)
            # 更新过程噪声
            Q = updateQ(Q, dt)
            # 如果聚类出来的量测值不为空
            if len(result) > 0:
                # 把上一帧的融合值与这一帧的量测值进行数据关联
                match, useid = get_matched(x, result)
                # 目标物跟踪
                x, id_value, p_value, counterid = get_single_lidar(
                    match,
                    result,
                    useid,
                    id_value,
                    p_value,
                    counterid,
                    F_m,
                    Q,
                    lidar_R,
                    G,
                    H,
                    lidar_P,
                )
            else:
                # 如果当前帧没有量测值
                # 回收所有id和协方差阵
                # 将融合值置为空
                for i in range(len(x)):
                    counterid.append(x[i, 1])
                x = []
                id_value = []
                p_value = []
        # 结果发布到 ‘fusion’话题
        pub = rospy.Publisher("fusion", autocontrol, queue_size=1)
        # 打印这一帧的目标数
        print(len(x))
        # 如果目标物不为空
        if len(x) > 0:
            # xx=x[np.where(x[:,10]>0)[0],:]
            # if len(x)>0:
            # 获取rt1和周围八个目标物信息
            top_x, rt1 = get_x(x, delta, trans_mat, head_angle, vehicle_x, vehicle_y)
            top_x = np.array(top_x)

            # else:
            # rt1=[]
            # top_x=np.zeros((8,5))
        else:
            # 否则置为空
            rt1 = []
            top_x = np.zeros((8, 5))
        # 把对应的进行赋值，这里要注意，由于can_com_pub里面的接收rt1的消息类型为Front_L_，
        # 所以这里将rt1目标物，和Front目标物的赋值互换，然后发布出去
        arg = autocontrol()
        arg.leftfront_l_long_obj = top_x[0, 0]
        arg.leftfront_l_lat_obj = top_x[0, 1]
        arg.leftfront_v_long_obj = top_x[0, 2]
        arg.leftfront_v_lat_obj = top_x[0, 3]
        arg.leftfront_width = top_x[0, 4]

        arg.rt1_l_long_obj = top_x[1, 0]
        arg.rt1_l_lat_obj = top_x[1, 1]
        arg.rt1_v_long_obj = top_x[1, 2]
        arg.rt1_v_lat_obj = top_x[1, 3]
        arg.rt1_width = top_x[1, 4]

        arg.rightfront_l_long_obj = top_x[2, 0]
        arg.rightfront_l_lat_obj = top_x[2, 1]
        arg.rightfront_v_long_obj = top_x[2, 2]
        arg.rightfront_v_lat_obj = top_x[2, 3]
        arg.rightfront_width = top_x[2, 4]

        arg.leftback_l_long_obj = top_x[3, 0]
        arg.leftback_l_lat_obj = top_x[3, 1]
        arg.leftback_v_long_obj = top_x[3, 2]
        arg.leftback_v_lat_obj = top_x[3, 3]
        arg.leftback_width = top_x[3, 4]

        arg.back_l_long_obj = top_x[4, 0]
        arg.back_l_lat_obj = top_x[4, 1]
        arg.back_v_long_obj = top_x[4, 2]
        arg.back_v_lat_obj = top_x[4, 3]
        arg.back_width = top_x[4, 4]

        arg.rightback_l_long_obj = top_x[5, 0]
        arg.rightback_l_lat_obj = top_x[5, 1]
        arg.rightback_v_long_obj = top_x[5, 2]
        arg.rightback_v_lat_obj = top_x[5, 3]
        arg.rightback_width = top_x[5, 4]

        arg.left_l_long_obj = top_x[6, 0]
        arg.left_l_lat_obj = top_x[6, 1]
        arg.left_v_long_obj = top_x[6, 2]
        arg.left_v_lat_obj = top_x[6, 3]
        arg.left_width = top_x[6, 4]

        arg.right_l_long_obj = top_x[7, 0]
        arg.right_l_lat_obj = top_x[7, 1]
        arg.right_v_long_obj = top_x[7, 2]
        arg.right_v_lat_obj = top_x[7, 3]
        arg.right_width = top_x[7, 4]
        # print(top_x[7,0],top_x[7,1])
        if len(rt1) > 0:
            arg.front_l_long_obj = rt1[0, 0]
            arg.front_l_lat_obj = rt1[0, 1]
            arg.front_v_long_obj = rt1[0, 2]
            arg.front_v_lat_obj = rt1[0, 3]
            arg.front_width = rt1[0, 4]

            print(rt1[0, 0], rt1[0, 1])
        pub.publish(arg)
        # try:
        # print(dt)
        # except:
        # print(ti)
        t2 = rospy.get_time()

        print(t2 - ti)


def callback(test, me):
    global Data
    global T
    global param
    # 接收点云数据
    pc = ros_numpy.numpify(test)
    points_trans = pcl_points_rf(pc)
    # 筛选点云的范围
    points_trans = get_cloud(points_trans)
    # 把数据同一乘100，转成int32存储到data里面
    points_trans = points_trans * 100
    points_trans = points_trans.astype(np.int32)
    # 计算总共有多少个数据
    length = len(points_trans) * 3
    # 进程锁，对Data进行操作，其他进程无法访问
    lock = Lock()
    lock.acquire()
    # 从msg_RadarData接收
    param = get_me(me)
    # 把点云数据resize成一行存储到Data中，要注意长度要小于data的长度
    memoryview(Data)[:length] = points_trans.reshape(-1)
    # 倒数第一个值记录点云resize后的长度
    memoryview(Data)[-1] = np.array([length], np.int32)
    # 倒数第二个值计数
    memoryview(Data)[-2] = np.array([Data[-2] + 1], np.int32)
    # print(length,Data[-2])
    lock.release()


def main():
    global x
    x = []
    global counterid
    counterid = [i for i in range(100)]
    global id_value
    id_value = []
    global p_value
    p_value = []
    global t
    t = 0
    global Q
    Q = np.array(np.zeros([4, 4]))
    global F
    F = np.array(np.eye(4))
    global G
    G = np.array(np.eye(4))
    global result
    result = []
    global frame
    frame = 0

    global trans_mat
    trans_mat = np.zeros([20, 4])
    global param
    rospy.init_node("pcl_listener", anonymous=True)
    # message_filters同时获取两个话题里面的数据，并做时间同步，调用callback回调函数
    test_sub = message_filters.Subscriber(
        "/rfans_driver/rfans_points", PointCloud2, queue_size=1, buff_size=5240000
    )

    me = message_filters.Subscriber("msg_RadarData", autocontrolRadardata, queue_size=1)

    subs = (test_sub, me)
    ts = message_filters.ApproximateTimeSynchronizer(
        subs, 1, 0.05, allow_headerless=True
    )
    ts.registerCallback(callback)
    # 每0.05秒，进行一次timer_callback回调函数
    rospy.Timer(rospy.Duration(0.05), timer_callback)
    rospy.spin()


if __name__ == "__main__":

    main()
