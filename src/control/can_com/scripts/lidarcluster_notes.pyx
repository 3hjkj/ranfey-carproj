import numpy as np
cimport numpy as np
import cython
cimport cython
from cython.parallel import prange
DTYPE = np.intc
ctypedef int DTYPE_t
ctypedef np.float64_t DTYPE_s
#DTYPE_t[:,:]  np.float64
#np.ndarray[DTYPE_t, ndim=2]
from collections import Counter
from math import pi
cdef extern from "math.h" nogil:#c语言里面的函数
    float cosf(float)
    float sinf(float)
    float sqrtf(float)
    float powf(float x,float y)
    float ceilf(float)
    float floorf(float)

#r-fans 使用ros_numpy 读取点云数据，这个函数适用于北科天绘的雷达
@cython.boundscheck(False) 
@cython.wraparound(False) 
def pcl_points_rf(pc):
    cdef np.ndarray[np.float64_t, ndim=2] points=np.zeros((pc.shape[0],3),dtype=np.float64)
    points[:,0]=pc['x']
    points[:,1]=pc['y']
    points[:,2]=pc['z']
    return points   #x,y,z


#robosense  使用ros_numpy 读取点云数据，这个函数适用于速腾的雷达
@cython.boundscheck(False) 
@cython.wraparound(False) 
def pcl_points_rs(pc):
    cdef np.ndarray[np.float64_t, ndim=2] points=np.zeros((pc.shape[0]*pc.shape[1],3),dtype=np.float64)
    points[:,0]=pc['x'].reshape(-1)
    points[:,1]=pc['y'].reshape(-1)
    points[:,2]=pc['z'].reshape(-1)
    return points  #x,y,z

@cython.boundscheck(False) 
@cython.wraparound(False) 
def get_motion(np.float64_t[:,::1] points,np.float64_t Ve,np.float64_t Vn,np.float64_t delta,np.float64_t angle,np.float64_t tp):
    cdef int i,j,k,n,l=points.shape[0]
    cdef np.float64_t[:,::1] axy=np.zeros((l,2),dtype=np.float64)
    cdef np.float64_t[:,::1] axy1=np.zeros((l,2),dtype=np.float64)
    cdef np.float64_t[:,::1] point=np.zeros((l,2),dtype=np.float64)
    cdef np.float64_t[::1] num=np.zeros(l,np.float64),seita=np.zeros(l, np.float64)
    point=points[:,:2]
    for n in range(l):
       num[n]=n*tp/l
    for k in range(l):
       axy[k,0]=Ve*num[k]
       axy[k,1]=Vn*num[k]
       seita[k]=delta*num[k]
    
    for i in range(l):
       axy1[i,0]=axy[i,0]*cosf(angle)+axy[i,1]*sinf(angle)
       axy1[i,1]=axy[i,1]*cosf(angle)-axy[i,0]*sinf(angle)
    for j in range(l):
       points[j,0]=point[j,0]*cosf(seita[j])+point[j,0]*sinf(seita[j])+axy1[j,0]
       points[j,1]=point[j,1]*cosf(seita[j])-point[j,0]*sinf(seita[j])+axy1[j,1]
    return np.asarray(points)


@cython.boundscheck(False) 
@cython.wraparound(False) 
def get_cloud(np.ndarray[np.float64_t, ndim=2] points_cloud):
    cdef np.float64_t car_min_x=-2  #雷达距车辆后边的距离
    cdef np.float64_t car_max_x=2.55  #雷达距车辆前边的距离
    cdef np.float64_t car_min_y=-0.9  #雷达距车辆右边的距离
    cdef np.float64_t car_max_y=0.9    #雷达距车辆左边的距离
    cdef np.float64_t x_min=-20  #纵向最后范围
    cdef np.float64_t x_max=60   #纵向最前范围
    cdef np.float64_t y_max=6   #横向最左范围
    cdef np.float64_t y_min=-6   #横向最右范围
    cdef np.float64_t z_max=1    #高度最大范围
    #筛选探测范围
    points_cloud=points_cloud[np.where((points_cloud[:,0]>x_min)&(points_cloud[:,0]<x_max)&(points_cloud[:,1]>y_min)&
                                       (points_cloud[:,1]<y_max)&(points_cloud[:,2]<z_max))[0],:]
    #去除打在车辆上的点云
    points_cloud=points_cloud[np.where(
            ~((points_cloud[:,0]>=car_min_x)&(points_cloud[:,0]<= car_max_x)&
              (points_cloud[:,1]>=car_min_y)&(points_cloud[:,1]<= car_max_y)))[0],:]
    #把坐标统一在车前保险杆中点
    points_cloud[:,0]=points_cloud[:,0]-2.55
    return points_cloud

@cython.boundscheck(False) 
@cython.wraparound(False) 
cdef downsample_circle(DTYPE_t[:,::1] xel, np.int64_t[::1] idex):#依附于降采样下的循环操作
    cdef int i,j,n,ln=0,lm=xel.shape[0]
    cdef DTYPE_t[:,::1] tmp
    cdef DTYPE_t[:,::1] points_update2 = np.zeros((lm,3),dtype=DTYPE)
    cdef DTYPE_t[::1] tsum = np.zeros(3, DTYPE)
    for i in range(idex.shape[0]):
        if i+1 < idex.shape[0]: tmp = xel[idex[i]:idex[i+1]]
        else: tmp = xel[idex[i]:]
        if tmp.shape[0] > 1: #只有一个点的就删除
           for j in range(3):
              for n in range(tmp.shape[0]):
                 tsum[j]+=tmp[n,j]  #x,y,z取平均
           for j in range(3):
              points_update2[i,j]=tsum[j] / tmp.shape[0]
              tsum[j]=0
        ln = ln+1
    return points_update2[:ln]


@cython.boundscheck(False) 
@cython.wraparound(False) 
def get_downsample(points_update, Cellsize=10):# 降采样，减少数据量
    cdef np.ndarray[DTYPE_t, ndim=2] voxel=np.zeros((points_update.shape[0],6),dtype=DTYPE) #dx,dy,dz,x,y,z
    cdef np.ndarray[DTYPE_t, ndim=2] vo=np.zeros((points_update.shape[0],3),dtype=DTYPE)
    cdef np.ndarray[DTYPE_t, ndim=2] xel=np.zeros((points_update.shape[0],3),dtype=DTYPE)
    cdef np.int64_t[::1] idex
    cdef np.ndarray[DTYPE_t, ndim=2] points_update2
    voxel[:,3:]=points_update[:, 0:3]
    voxel[:,0:3]=np.ceil((voxel[:,3:])/Cellsize) 
    voxel = voxel[np.lexsort([voxel[:,2],voxel[:,1],voxel[:,0]])] # 依次按dx,dy,dz进行排序 
    vo, xel = np.ascontiguousarray(voxel[:,0:3]), np.ascontiguousarray(voxel[:, 3:])
    idex =np.where(np.insert(np.any(np.diff(vo, axis=0), axis=1), 0, True)==True)[0]  # 按降采样尺度分组，每组第1个point的索引
    
    # 循环提取每一组的平均x,y,z得到降采样的结果
    points_update2 =np.asarray(downsample_circle(xel, idex))
    return points_update2[points_update2.any(axis=1)]


@cython.boundscheck(False) 
@cython.wraparound(False) 
cdef grid_circle(DTYPE_t[:,::1] voxel, np.int64_t[::1] idex): # 依附于栅格化下的循环操作
    cdef DTYPE_t[:,::1] points_update3 = np.zeros((voxel.shape[0],6),dtype=DTYPE) # 'sx','sy','x','y','z','zmax'
    
    cdef DTYPE_t[:,::1] tmp
    cdef int i,j,m,n,ln=0
    cdef DTYPE_t zmax
    cdef DTYPE_t zmin
    for i in range(idex.shape[0]):
        
        if i+1 < idex.shape[0]: tmp = voxel[idex[i]:idex[i+1]] # sx, sy, x,y,z
        else: tmp = voxel[idex[i]:]
        zmax,zmin=-1000,1000 # 赋予zmax, zmin初始值
        for j in range(tmp.shape[0]):
           zmax=max(zmax,tmp[j,4])
           zmin=min(zmin,tmp[j,4])
        #栅格内z轴的最大最小值之差，小于0.4维地面点，剔除，高于3米的建筑物点剔除
        if 40 <= zmax-zmin <= 300: 
            # 则把tmp赋值给points_update
            for m in range(tmp.shape[0]):
               for n in range(5): points_update3[ln+m,n] = tmp[m,n]
               points_update3[ln+m,5] = zmax
            ln += tmp.shape[0] 
            #points_update3[idex[i]:idex[i]+len(tmp)]=np.hstack((tmp, zmax * np.ones((len(tmp),1))))
    return points_update3[:ln]


@cython.boundscheck(False) 
@cython.wraparound(False) 
def grid(points_update2, Cell=40):# 栅格化
    cdef np.ndarray[DTYPE_t, ndim=2] voxel = np.zeros((points_update2.shape[0],5),dtype=DTYPE) # gx, gy, x,y,z
    cdef np.int64_t[::1] idex
    cdef np.ndarray[DTYPE_t, ndim=2] points_update3
    voxel[:,2:]=points_update2 
    voxel[:,0]=np.ceil((voxel[:,2])/Cell)  
    voxel[:,1]=np.ceil((voxel[:,3])/Cell) 
    voxel = voxel[np.lexsort([voxel[:,1],voxel[:,0]])] # 将voxel按先x后y递增排序
    # 按栅格化尺度分组，每组第1个point的索引
    idex = np.where(np.insert(np.any(np.diff(voxel[:,:2],axis=0),axis=1),0,True)==True)[0] 
    voxel=np.ascontiguousarray(voxel)
    # 'sx','sy','x','y','z'
    points_update3 = np.asarray(grid_circle(voxel, idex))
    return points_update3[points_update3.any(axis=1)]

@cython.boundscheck(False) 
@cython.wraparound(False) 
cdef cluster_circle2(object points_slice, DTYPE_t[:,::1] grid_slice,DTYPE_t[:,::1] target_points,DTYPE_t[::1] idex,DTYPE_t[::1] idex2):
    cdef int i, j, lt2, lr2, li2 = idex.shape[0]
    cdef DTYPE_t[::1] idex2_2, idex_round2
    cdef DTYPE_t[::1] seed_grid2
    cdef DTYPE_t[:,::1] tmp2
    for i in range(idex2.shape[0]):
        seed_grid2 = grid_slice[idex2[i]] # 得到次一级种子栅格
        lt2 = 0
        for j in range(target_points.shape[0]):
           if target_points[j,4] == 0: break
           lt2 += 1
        tmp2 = points_slice[idex2[i]]  # 该邻域栅格所包含的点云
        # 把次级种子栅格的点云加入目标
        target_points[lt2:lt2+tmp2.shape[0]] = tmp2 
        # 用于存放被选中的邻近栅格的的索引, lr2是idex_round2的索引
        idex_round2, idex2_2, lr2 = np.zeros(8, np.int32), np.zeros(8, np.int32), 0 
        
        for j in range(idex.shape[0]):
            # 如果属于邻近栅格，且zmax相差小于100
           if ((abs(grid_slice[idex[j],0]-seed_grid2[0]) <= 1) & (abs(grid_slice[idex[j],1]-seed_grid2[1]) <= 1) &(abs(grid_slice[idex[j],2]-seed_grid2[2]) <= 70)):
               # 将idex2_2中的索引作为种子栅格，进行下一轮栅格筛选
              idex_round2[lr2], idex2_2[lr2], lr2 = j, idex[j], lr2+1
        # 从idex中 删除 idex_round2的内容
        idex = np.delete(idex, idex_round2[:lr2])
        #idex = np.delete(np.asarray(idex), np.asarray(idex_round2[:lr2]))
        idex, target_points = cluster_circle2(points_slice, grid_slice, target_points, idex, idex2_2[:lr2])
    return idex, target_points



@cython.boundscheck(False) 
@cython.wraparound(False) 
def cluster_circle(object points_slice, DTYPE_t[:,::1] grid_slice,DTYPE_t[::1] idex,np.float64_t ti):
    # 对栅格化点云的父级聚类
    cdef int row=0
    cdef int i,lr,li=idex.shape[0]
    # target_output：time,x,y,wide,length,hight,z
    cdef np.float64_t[:,::1] target_output= np.zeros((li,7)) 
    cdef DTYPE_t[:,::1] tmp,target_points
    cdef DTYPE_t[::1] idex2,idex_round
    cdef DTYPE_t[::1] seed_grid
    cdef DTYPE_t xmin, ymin, zmin,xmax, ymax, zmax,xc,yc,zc
    while idex.shape[0] > 0:# 只要栅格数量>0
        # 以10为尺度进行降采样，以50为尺度进行栅格化... 一个栅格内最多存在50个点云
        target_points = np.zeros((li*50,6),dtype=DTYPE) # 初始化'sx','sy','x','y','z','zmax'
        
        #print(grid_slice[idx, 0:3])
        seed_grid = grid_slice[idex[0]] # 找到一个种子栅格... sx, sy, zmax
        tmp = points_slice[idex[0]]  # 将points_slice中的第idex[0]个片 
        target_points[:tmp.shape[0]] = tmp # 栅格内点云赋给target_points,目标中点云数量加tmp.shape[0]
        #idex = np.delete(np.asarray(idex), np.array([0]))
        # 索引中删除当前种子, 栅格数量减1
        idex = np.delete(idex, np.array([0]))
        # print (idex)
        # 用于存放被选中的邻近栅格的的索引, lr是idex_round的索引
        idex_round, idex2, lr = np.zeros(8, np.int32), np.zeros(8, np.int32), 0
        for i in range(idex.shape[0]):# 对idex中的每一个索引
            # 如果属于邻近栅格 且zmax相差小于70
           if ((abs(grid_slice[idex[i],0]-seed_grid[0]) <= 1) & (abs(grid_slice[idex[i],1]-seed_grid[1]) <= 1) &(abs(grid_slice[idex[i],2]-seed_grid[2]) <=70)): 
               # 可以作为邻域种子栅格的索引, 由邻域形成的种子索引
               idex_round[lr], idex2[lr], lr = i, idex[i], lr+1
        #idex_round = np.where(np.logical_and(np.abs(grid_slice[idex,0]-seed_grid[0])<=1, 
                                             #np.abs(grid_slice[idex,1]-seed_grid[1])<=1) & 
                             #(np.abs(grid_slice[idex,2] - seed_grid[2]) <=0.6))[0] 
        #print ('idex_round:', idex_round)
        # 从原索引中删除已经形成种子的索引
        idex = np.delete(idex, idex_round[:lr])
        #idex = np.delete(np.ctypeslib.as_array(idex), np.ctypeslib.as_array(idex_round[:lr]))
        idex2 = idex2[:lr]
        
        idex, target_points= cluster_circle2(points_slice, grid_slice, target_points, idex, idex2)
        # target_points: 'sx','sy','x','y','z','zmax' ...认为是同一目标的点云
        #计算xmin, ymin, zmin，xmax, ymax, zmax
        xmin, ymin, zmin = target_points[0,2], target_points[0,3], target_points[0,4]
        xmax, ymax, zmax = target_points[0,2], target_points[0,3], target_points[0,4]
        
        for i in range(1, target_points.shape[0]):
            if target_points[i,4] == 0: break
            xmin, xmax = min(xmin, target_points[i,2]), max(xmax, target_points[i,2])
            ymin, ymax = min(ymin, target_points[i,3]), max(ymax, target_points[i,3])
            zmin, zmax = min(zmin, target_points[i,4]), max(zmax, target_points[i,4])
        #计算重心，xc,yc,zc,并输出
        #if zmin<20: 
        xc, yc,zc = (xmin+xmax)/2, (ymin+ymax)/2,(zmin+zmax)/2
        target_output[row,0], target_output[row,1],target_output[row,2],target_output[row,3],target_output[row,4],target_output[row,5],target_output[row,6],row = ti,np.float64(xc)/100, np.float64(yc)/100,np.float64((ymax-ymin))/100,np.float64((xmax-xmin))/100,np.float64((zmax-zmin))/100,np.float64(zc)/100,row+1 
    return target_output[:row,:]

@cython.boundscheck(False) 
@cython.wraparound(False) 
def grid_cluster(DTYPE_t[:,::1] usepoints,np.float64_t ti):
    cdef int l=usepoints.shape[0]
    cdef DTYPE_t[:,::1] grid_point=np.zeros((l,3),dtype=DTYPE)
    cdef DTYPE_t[:,::1] grid_slice
    cdef np.float64_t[:,::1] targets_tmp
    #cdef DTYPE_t[::1] idex
    grid_point[:,0]=usepoints[:,0]
    grid_point[:,1]=usepoints[:,1]
    grid_point[:,2]=usepoints[:,5]
    grid_slice, idex = np.unique(grid_point, axis=0, return_index=True)
    # 将points_update3按栅格切片
    points_slice = np.array(np.vsplit(usepoints, idex[1:])) 
    # 一个idex为一个种子的索引
    idex = (np.arange(points_slice.shape[0])).astype(np.int32)
    #targets_tmp：time,x,y,wide,length,hight,z
    targets_tmp = cluster_circle(points_slice, grid_slice, idex,ti) 
    return targets_tmp


'''

@cython.boundscheck(False) 
@cython.wraparound(False)
def cluster_result(np.ndarray[DTYPE_t, ndim=2] points,np.float64_t ti):
    cdef np.ndarray[DTYPE_t, ndim=2] usepoints
    points=get_downsample(points,Cellsize=10)
    #print(len(points))
    usepoints=grid(points,Cell=40)
    if len(usepoints)>0:
       result=np.asarray(grid_cluster(usepoints,ti))
       #result=get_result(result)  
    else:
       result=[]
    return result

@cython.boundscheck(False) 
@cython.wraparound(False) 
def get_object(np.ndarray[DTYPE_t, ndim=2] item,DTYPE_t ti):
    cdef DTYPE_t xmin,xmax,ymin,ymax,zmin,zmax,length,wide,high,x_1,y
    xmin,xmax=np.min(item[:,0]),np.max(item[:,0])
    ymin,ymax=np.min(item[:,1]),np.max(item[:,1])
    zmin,zmax=np.min(item[:,2]),np.max(item[:,2])
    length=xmax-xmin
    wide=ymax-ymin
    high=zmax-zmin
    x_1=(xmax+xmin)/2
    y=(ymax+ymin)/2
    return [ti,x_1,y,wide,length,high]

@cython.boundscheck(False) 
@cython.wraparound(False) 
def grid_cluster(np.ndarray[DTYPE_t, ndim=2] usepoints,DTYPE_t ti):
    cdef np.ndarray[DTYPE_t, ndim=2] result=np.zeros((1,6),dtype=DTYPE)
    cdef DTYPE_t x,y,a
    
    cdef np.ndarray[DTYPE_t, ndim=2] base
    cdef np.ndarray[DTYPE_t, ndim=2] border_points
    cdef np.ndarray[DTYPE_t, ndim=2] c
    cdef np.ndarray[DTYPE_t, ndim=2] e
    cdef np.ndarray[DTYPE_t, ndim=2] ke
    cdef np.ndarray[DTYPE_t, ndim=2] keep
    cdef np.ndarray[DTYPE_t, ndim=2] obj
    while len(usepoints)>1:
        x,y=usepoints[0,0],usepoints[0,1]
        idx=np.where((usepoints[:,0]==x)&(usepoints[:,1]==y))[0]
        base=usepoints[idx,:]
        usepoints=np.delete(usepoints,idx, 0)
        didx=np.where((usepoints[:,0]>=x-1)&(usepoints[:,0]<=x+1)&(usepoints[:,1]>=y-1)&(usepoints[:,1]<=y+1))[0]
        if len(didx)>0:
            infected_points=usepoints[didx,:]
            idx1=np.where(np.abs(infected_points[:,5]-base[0,5])<0.6)[0]
            if len(idx1)>0:
                border_points=usepoints[didx[idx1],:]
                base=np.concatenate((base,border_points),axis=0)
                usepoints=np.delete(usepoints,didx[idx1],0)
                c=border_points[:,:2]
                idx2=np.unique(c[:,0]+c[:,1]*1j,return_index=True)[1]
                keep = c[idx2]
            else:
                keep=np.zeros((0,0),dtype=DTYPE)
        else:
            keep=np.zeros((0,0),dtype=DTYPE)
        while len(keep)>0:
            ke=np.zeros((1,2),dtype=DTYPE)
            for i in range(len(keep)):
                x,y=keep[i,0],keep[i,1]
                didx=np.where((usepoints[:,0]>=x-1)&(usepoints[:,0]<=x+1)&(usepoints[:,1]>=y-1)&(usepoints[:,1]<=y+1))[0]
                
                if len(didx)>0:
                    infected_points=usepoints[didx,:]
                    a=base[np.where((base[:,0]==x)&(base[:,1]==y))[0],5][0]
                    idx1=np.where(np.abs(infected_points[:,5]-a)<0.6)[0]
                    if len(idx1)>0:
                        border_points=usepoints[didx[idx1],:]
                        base=np.concatenate((base,border_points),axis=0)
                        usepoints=np.delete(usepoints,didx[idx1],0)
                        c=border_points[:,:2]
                        idx2=np.unique(c[:,0]+c[:,1]*1j,return_index=True)[1]
                        e=c[idx2]
                        ke=np.concatenate((ke,e),axis=0)
            keep=np.delete(ke,0,0)
        if np.min(base[:,4])<0.2:
            obj=np.mat(get_object(base[:,2:5],ti))
            result=np.concatenate((result,obj),axis=0)
    result=np.delete(result,0,0)
    return result
'''


def transfrom(gen_left,gen_right):
    length=3.3
    width=0.3
    back_height=0.24
    length_offset=0.54
    
    '''
    m=gen_right.shape[1]
    dst = np.ones((m+1,gen_right.shape[0]))
    dst[:m, :] = np.copy(gen_right.T)
    dst= np.dot(T,dst)
    gen_right=dst[:m, :].T
    '''
    
    gen_right[:,0]=-gen_right[:,0]-length_offset
    gen_right[:,1]=-gen_right[:,1]-width/2
    
    gen_left[:,0]=-gen_left[:,0]-length_offset  #
    gen_left[:,1]=-gen_left[:,1]+ width/2
    points=np.concatenate((gen_left,gen_right),axis=0)
    
    #gen_back=gen_back[:,[1,0,2]]
    #gen_back=gen_back[np.where(gen_back[:,0]>0.1)[0],:]
    #gen_back[:,0]=-gen_back[:,0]-length
    #gen_back[:,2]=gen_back[:,2]-back_height
    #points=np.concatenate((points,gen_back),axis=0)
    x_min=-25
    x_max=80
    y_min=-5
    y_max=5
    z_max=2.5
    car_min_y=-width/2-0.2
    car_max_y=width/2+0.2
    car_min_x=-length-0.2
    car_max_x=0.2
    points=points[np.where((points[:,0]>x_min)&(points[:,0]<x_max)&(points[:,1]>y_min)&
                                       (points[:,1]<y_max)&(points[:,2]<z_max))[0],:]
    points=points[np.where(
            ~((points[:,0]>=car_min_x)&(points[:,0]<= car_max_x)&
              (points[:,1]>=car_min_y)&(points[:,1]<= car_max_y)))[0],:]
    return points

#dt为前后帧的时间差，
@cython.boundscheck(False) 
@cython.wraparound(False) 
def updateF(np.ndarray[DTYPE_s, ndim=2] F,DTYPE_s dt):
    F[0, 2], F[1, 3]  = dt, dt
    return F
#根据上一帧的x,p，进行预测
@cython.boundscheck(False) 
@cython.wraparound(False) 
def predict(np.ndarray[DTYPE_s, ndim=2] x_1,np.ndarray[DTYPE_s, ndim=2] F,np.ndarray[DTYPE_s, ndim=2] P,np.ndarray[DTYPE_s, ndim=2] Q):  
    cdef np.ndarray[DTYPE_s, ndim=2] a
    x_1= np.dot(F,x_1)
    a=np.dot(F,P)
    P = np.dot(a,F.T)+Q
    return x_1,P

@cython.boundscheck(False) 
@cython.wraparound(False) 
def get_S(np.ndarray[DTYPE_s, ndim=2] H,np.ndarray[DTYPE_s, ndim=2] R,np.ndarray[DTYPE_s, ndim=2] P):
    #PHt = P * H.T
    #S = H * PHt + R 
    cdef np.ndarray[DTYPE_s, ndim=2] PHt
    cdef np.ndarray[DTYPE_s, ndim=2] S
    PHt=np.dot(P,H.T)
    S=np.dot(H,PHt)+R
    S=np.linalg.inv(S)
    return S,PHt

@cython.boundscheck(False) 
@cython.wraparound(False) 
def get_P(np.ndarray[DTYPE_s, ndim=2] x_1,np.ndarray[DTYPE_s, ndim=2] z,np.ndarray[DTYPE_s, ndim=2] H,np.ndarray[DTYPE_s, ndim=2] K,np.ndarray[DTYPE_s, ndim=2] P,np.ndarray[DTYPE_s, ndim=2] G):
    x_1=x_1+np.dot(K,(z-np.dot(H,x_1)))
    P=np.dot((G-np.dot(K,H)),P)
    return x_1,P
#使用量测值进行目标物状态更新
@cython.boundscheck(False) 
@cython.wraparound(False) 
def update_optimal(np.ndarray[DTYPE_s, ndim=2] x_1,np.ndarray[DTYPE_s, ndim=2] z,np.ndarray[DTYPE_s, ndim=2] H,np.ndarray[DTYPE_s, ndim=2] R,np.ndarray[DTYPE_s, ndim=2] P,np.ndarray[DTYPE_s, ndim=2] G):
    cdef np.ndarray[DTYPE_s, ndim=2] S,PHt,K
    S,PHt=get_S(H,R,P)
    K = np.dot(PHt,S)
    x_1,P=get_P(x_1,z,H,K,P,G)
    return x_1,P

#计算两个点的欧式距离
@cython.boundscheck(False) 
@cython.wraparound(False) 
def get_L2(DTYPE_s x1,DTYPE_s y1,DTYPE_s x2,DTYPE_s y2):
    cdef DTYPE_s dx,dy,d
    dx=x1-x2
    dy=y1-y2
    d=np.sqrt(dx**2+dy**2)
    return d

#计算两个点的余弦相似度
@cython.boundscheck(False) 
@cython.wraparound(False) 
def cosine_similarity(vector1, vector2):
    cdef DTYPE_s dot_product = 0.0
    cdef DTYPE_s normA = 0.0
    cdef DTYPE_s normB = 0.0
    for a, b in zip(vector1, vector2):
        dot_product += a * b
        normA += a ** 2
        normB += b ** 2
    if normA == 0.0 or normB == 0.0:
        return 0
    else:
        return (dot_product / ((normA**0.5)*(normB**0.5)))

@cython.boundscheck(False) 
@cython.wraparound(False) 
def get_para(x_1,d):
    cdef np.ndarray[DTYPE_s, ndim=2] para=np.zeros((len(d),4))
    for i in range(len(d)):
        para[i,0]=get_L2(x_1[2,0],x_1[3,0],d[i,1],d[i,2]) #l2距离
        para[i,1]=cosine_similarity([x_1[2,0],x_1[3,0]],[d[i,1],d[i,2]]) #余弦相似度
        para[i,2]=abs(x_1[3,0]-d[i,2])  #左右距离差
        para[i,3]=abs(x_1[2,0]-d[i,1])  #前后距离差
    return para
#更新过程噪声矩阵
@cython.boundscheck(False) 
@cython.wraparound(False) 
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

#挑选车身周围八个目标物
@cython.boundscheck(False) 
@cython.wraparound(False) 
def get_result(result):
    result=np.mat(result)
    #左右两侧的范围
    left,right=1.2,1.2
    #在result里面插入一列，为0
    a=np.zeros((len(result),1)) 
    r=np.zeros((1,8))
    result=np.column_stack((a,result))
    #result的第一列赋值为各个目标物距离原点的绝对距离
    result[:,0]=np.sqrt(np.square(result[:,2])+np.square(result[:,3]))
    #leftfront，x>0,y>left
    leftfront=result[np.where((result[:,2]>0)&(result[:,3]>left))[0],:]
    if len(leftfront)>0:
        #如果该区域有目标，取最小值与r合并
        leftfront=leftfront[np.argmin(leftfront[:,0]),:]
        r=np.concatenate((r,leftfront),axis=0)
    #front x>0,y<=left,y>=-right
    front=result[np.where((result[:,2]>0)&(result[:,3]<=left)&(result[:,3]>=-right))[0],:]
    if len(front)>0:
        #如果该区域有目标，取最小值与r合并
        front=front[np.argmin(front[:,0]),:]
        r=np.concatenate((r,front),axis=0)
    #rightfront  x>0,y<-right
    rightfront=result[np.where((result[:,2]>0)&(result[:,3]<=-right))[0],:]
    if len(rightfront)>0:
        #如果该区域有目标，取最小值与r合并
        rightfront=rightfront[np.argmin(rightfront[:,0]),:]
        r=np.concatenate((r,rightfront),axis=0)
    #leftback x<-3.24,y>left
    leftback=result[np.where((result[:,2]<=-3.24)&(result[:,3]>=left))[0],:]
    if len(leftback)>0:
        #如果该区域有目标，取最小值与r合并
        leftback=leftback[np.argmin(leftback[:,0]),:]
        r=np.concatenate((r,leftback),axis=0)
    #back x<-3.24,y<left y>=-right
    back=result[np.where((result[:,2]<=-3.24)&(result[:,3]<=left)&(result[:,3]>=-right))[0],:]
    if len(back)>0:
        #如果该区域有目标，取最小值与r合并
        back=back[np.argmin(back[:,0]),:]
        r=np.concatenate((r,back),axis=0)
    #rightback x<-3.24,y<=-right
    rightback=result[np.where((result[:,2]<=-3.24)&(result[:,3]<=-right))[0],:]
    if len(rightback)>0:
        #如果该区域有目标，取最小值与r合并
        rightback=rightback[np.argmin(rightback[:,0]),:]
        r=np.concatenate((r,rightback),axis=0)
    #left_1 x>-3.24 x<0,y>left
    left_1=result[np.where((result[:,2]<0)&(result[:,2]>-3.24)&(result[:,3]>left))[0],:]
    if len(left_1)>0:
        #如果该区域有目标，取最小值与r合并
        left_1=left_1[np.argmin(left_1[:,0]),:]
        r=np.concatenate((r,left_1),axis=0)
    #right_1 x>-3.24 x<0,y<=right
    right_1=result[np.where((result[:,2]<0)&(result[:,2]>-3.24)&(result[:,3]<=-right))[0],:]
    if len(right_1)>0:
        right_1=right_1[np.argmin(right_1[:,0]),:]
        r=np.concatenate((r,right_1),axis=0)
    
    r=r[1:,1:]
    return r

#第一帧目标，初始化参数，赋予id，
#counterid：list结构0-255
@cython.boundscheck(False) 
@cython.wraparound(False) 
def get_initialize(m,counterid,lidar_P,ti):
    x_1=[]
    id_value,p_value=[],[]
    #获取每一个目标物测量状态
    for i in range(len(m)):
        #time,x,y,wide,length,high，z
        m1=m[i,:].T
        #m2:time,id,x,y,vx,vy,wide,length,high,state,age
        m2=[ti,counterid[0],np.float64(m1[1]),np.float64(m1[2]),0,0,np.float64(m1[3]),np.float64(m1[4]),np.float64(m1[5]),0,1]
        #存下对应的协方差和id
        p_value.append(lidar_P)
        id_value.append(counterid[0])
        #从counterid中删除以及赋值的id
        del(counterid[0])
        #添加到目标物列表
        x_1.append(m2)
    x_1=np.mat(np.float64(x_1))
    return x_1,counterid,id_value,p_value

@cython.boundscheck(False) 
@cython.wraparound(False) 
def matched(a,b):
    index1=np.argmin(a[:,0]) #最小L2距离
    index2=np.argmax(a[:,1])  # 最大的余弦相似度
    #如果两个index指向同一位置
    if index1==index2:
        idx=b[0][index1] #获取最邻近匹配的索引
        cos=a[index1,0]  # 采用L2距离为指标
    else:
        #否则取横向距离差更小的索引
        if a[index1,2]<a[index2,2]:
            idx=b[0][index1]
            cos=a[index1,0]
        else:
            idx=b[0][index2]
            cos=a[index2,0]
    return idx,cos

@cython.boundscheck(False) 
@cython.wraparound(False) 
def get_matched(m,r):
    me1=[]
    useid=[]
    for i in range(len(m)):
        #上一帧目标物的测量状态，
        m1=m[i,:].T
        idx,cos=get_match(m1,r)# 单个已跟踪目标匹配
        #将每一个匹配的idx和cos添加到上一帧测量状态后面
        m1=np.row_stack((m1,[idx]))
        m1=np.row_stack((m1,[cos])).T
        me1.append(m1)
        #如果idx不是nan，则添加到uesid中，
        if np.isnan(idx)==False:
            useid.append(idx)
    me1=np.mat(np.float64(me1))
    #对useid去重，求重复的id和出现的频率
    b = dict(Counter(useid))
    #选择出现频率大于1的id
    b1=[key for key,value in b.items()if value > 1]
    #如果存在多个重复id
    if len(b1)!=0:
        #对于每一个重复id，找到重复匹配的目标所在的位置，选择l2距离最小的，
        #剩下的把idx置为nan
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
@cython.boundscheck(False) 
@cython.wraparound(False) 
def get_match(x_1,d): # 单个已跟踪目标与新目标匹配
    #x_1=np.mat(x_1).T
    #计算关联矩阵
    para=get_para(x_1,d)
    #thr=abs(x_1[2#,0])*0.1
    #thr=max(thr,2.5)
    #获取左右距离差小于1.5，前后距离差小于3的目标物索引
    b=np.where((abs(para[:,3])<3)&(abs(para[:,2])<1.5))
    #获取落入跟踪们里面的目标物列表
    a=para[b]
    #如果a一行都没有
    if len(a)==0:
        #idx和cos都为0
        idx=np.nan
        cos=np.nan
    else:
        #否则计算相应的索引和l2值
        idx,cos=matched(a,b)
    return idx,cos


#计算state值，value值代表可以连续存在多少帧预测值
@cython.boundscheck(False) 
@cython.wraparound(False) 
def get_value(x1):
    if x1[0,10]<50:
        value=2
    else:
        if x1[0,10]>150:
            value=5
        else:
            value=3
    return value
#前后帧数据关联结果，
@cython.boundscheck(False) 
@cython.wraparound(False) 
def update_lidar(r2,counterid,id_value,p_value,P1):
    r3=[]
    for i in range(len(r2)):
        #获取每一个目标物测量状态
        #time,x,y,wide,length,high,z
        m1=r2[i,:]
        #m2:time,id,x,y,vx,vy,wide,length,high,state,age
        m2=[m1[0,0],counterid[0],np.float64(m1[0,1]),np.float64(m1[0,2]),0,0,np.float64(m1[0,3]),np.float64(m1[0,4]),np.float64(m1[0,5]),0,1]
        #存下对应的协方差p和id
        id_value.append(counterid[0])
        p_value.append(P1)
        #从counterid中删除以及赋值的id
        del(counterid[0])
        #添加到目标物列表
        r3.append(m2)
    r3=np.mat(np.float64(r3))
    return r3,counterid,id_value,p_value
#合并两个数组
@cython.boundscheck(False) 
@cython.wraparound(False) 
def get_connect(r,x_1):
    try:
        l=np.concatenate((r,x_1),axis=0)
    except:
        l=x_1
    return l
@cython.boundscheck(False) 
@cython.wraparound(False) 
def get_single_lidar(match,r,useid,id_value,p_value,counterid,F,Q,lidar_R,G,H,lidar_P):
    #通过数据关联的结果，得到匹配上的量测值对应得index，把没有与上一帧匹配上得量测值筛选出来，为r1
    #r:time,x,y,wide,length,high,z
    r1=np.delete(r,useid,axis=0)
    try:
        frame=r[0,0]
    except:
        print('NO OBJECT!')
    x_1=[]
    #对于上一帧检测得目标物
    for i in range(len(match)):
        #如果在这一帧没有量测值关联，match[i][0,-1]为关联时存储的idx，为nan时
        if np.isnan(match[i][0,-1])==True:
            #获取这一目标物在上一帧的状态
            x1=np.float64(match[i][0,:-1])
            #获取id存储的位置
            idx=np.where(id_value==x1[0,1])[0][0]
            #获取协方差矩阵p
            P=p_value[idx]
            #从量测状态中取出x,y,vx,vy
            x2=x1[0,2:6].T
            #使用卡尔曼滤波的预测方程进行预测，得到新的预测值和预测协方差矩阵
            x2,P=predict(x2,F,P,Q)
            #用新的量测值状态替换原来的测量状态  
            #time,id,x,y,vx,vy,wide,length,high,state,age
            x1[0,0],x1[0,2],x1[0,3],x1[0,4],x1[0,5]=frame,x2[0,0],x2[1,0],x2[2,0],x2[3,0]
            #state值+1
            x1[0,9]=x1[0,9]+1
            #age值+1；直到255
            if (x1[0,10]<255):
                x1[0,10]=x1[0,10]+1
            #得到value值
            value=get_value(x1)
            #如果state值小于value
            if x1[0,9]<value:
                #将x1添加到目标列表，使用的时预测值
                x_1.append(x1)
                p_value[idx]=P
            else:
                #否则，不添加到目标列表，删除对应得id和协方差阵p，将id回收到counterid中
                del id_value[idx]
                del p_value[idx] 
                counterid.append(int(x1[0,1]))
        else:
            #如果在这一帧有量测值关联，match[i][0,-1]为关联时存储的idx，不为nan时
            #获取这一目标物在上一帧的状态
            x1=match[i][0,:]
            #获取id存储的位置
            idx=np.where(id_value==x1[0,1])[0][0]
            #获取协方差矩阵p
            P=p_value[idx]
            #从量测状态中取出x,y,vx,vy
            x2=x1[0,2:6].T
            #使用卡尔曼滤波的预测方程进行预测，得到新的预测值和预测协方差矩阵
            x2,P=predict(x2,F,P,Q)
            #从对量测值里面找到该目标对应的量测所在的行数
            zm=r[int(x1[0,-1]),:]
            #量测值中取出x,y
            z=zm[0,1:3].T
            #在对应vx,vy位置添加0，因为激光雷达目标检测没有检测速度
            z=np.row_stack((z,[0]))
            z=np.row_stack((z,[0]))
            #使用量测值进行状态更新，得到更新后的估计值和协方差阵
            x2,P=update_optimal(x2,z,H,lidar_R,lidar_P,G)
            #把协方差矩阵做一个替换
            p_value[idx]=P
            #使用现在估计的x,y和上一帧的x,y求速度，F[0,2]中存储的是时间dt
            x1[0,4]=((x2[0,0]-x1[0,2])/F[0,2])
            x1[0,5]=((x2[1,0]-x1[0,3])/F[0,2])
            #用新的量测值状态替换原来的测量状态
            x1[0,0],x1[0,2],x1[0,3]=frame,x2[0,0],x2[1,0]
            #把wide,length,height更新一下
            x1[0,6],x1[0,7],x1[0,8]=zm[0,3],zm[0,4],zm[0,5]
            #取出对应的数据：time,id,x,y,vx,vy,wide,length,high,state,age
            x1=x1[0,:-1]
            #state=0
            x1[0,9]=0
            ##age值+1；直到255
            if (x1[0,10]<255):
                x1[0,10]=x1[0,10]+1 
            #添加到目标物列表
            x_1.append(x1)
    x_1=np.mat(np.float64(x_1))
    #如果r1（没有与上一帧目标物匹配上的这一帧量测值）不为0；
    if len(r1)==0:
        #直接输出上一步的列表
        l=x_1
    else:
        #把这些没有匹配上的目标初始化
        r2,counterid,id_value,p_value=update_lidar(r1,counterid,id_value,p_value,lidar_P)
        #并且添加到目标物列表里面，输出
        #l=np.concatenate((r2,x),axis=0)
        l=get_connect(r2,x_1)
    return l,id_value,p_value,counterid
#去重函数
@cython.boundscheck(False) 
@cython.wraparound(False) 
def unique(c):
    #c=np.array(c)
    x=c[:,0]+c[:,1]*1j
    idx=np.unique(x,return_index=True)[1]
    return c[idx]

@cython.boundscheck(False) 
@cython.wraparound(False) 
def get_x(x,delta,trans_mat,head_angle,vehicle_x,vehicle_y):
    #筛选的左右量测的范围
    left,right=1.2,1.2
    #此时的x数组的列为x,y,vx,vy,wide,length
    x=x[:,2:7]
    #给x添加一列元素
    a=np.zeros((len(x),1)) 
    r=np.zeros((1,6))
    t=np.zeros((1,6))
    result=np.column_stack((a,x))
    #result的第一列赋值为各个目标物距离原点的绝对距离
    result[:,0]=np.sqrt(np.square(result[:,1])+np.square(result[:,2]))
    #把八个区域里面最近的目标挑选出来，组成八行的数组，如果对应区域没有目标物，则该行使用全0替代
    #行顺序依次为：leftfront，front，rightfront，leftback，back，rightback，left_1，right_1
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
    #把第一行和第一列去掉，取剩下的数据
    #x,y,vx,vy,wide,length
    r=r[1:,1:]
    #计算cipv目标物
    b=[]
    #筛选一下前方的点，把都为0的去掉
    #trans_mat：left_x1,left_y1  right_x1,right_y1
    for i in range(len(trans_mat)):
      if (trans_mat[i,:]==0).all()==False:
         b.append(i)
    trans=trans_mat[b,:]
    #trans行数大于0，说明此时有前方的点发来
    if len(trans)>0:
       #分别获取左右量测的坐标点，存在trans_left和trans_right中，
       #并且去重，防止车辆静止的时候发来重复的点
       #计算去重后点的数量coun_trans
       trans_left=np.array(trans[:,:2])
       trans_right=np.array(trans[:,2:])
       trans_left=unique(trans_left)
       trans_right=unique(trans_right)
       coun_trans=len(trans_left)
    else:
        #否则，点的数量为0
       coun_trans=0
    #只保留y大于0的点
    trans_result=result[np.where((result[:,1]>0))[0],:]
    #delta是目标行驶的航向角，求绝对值
    curve_delta=abs(delta)
    #三个条件，前方发来预瞄点两侧点的的个数大于3，且前方有目标物，且转向角大于5度时
    if (coun_trans>3)&(len(trans_result)>0)&(curve_delta>5):
        #车头偏航角，原来单位时弧度，转为度
        head_angle=head_angle*pi/180
        #预瞄点的坐标系是经纬度坐标转为大地坐标系之后的，需要转移到车体坐标系
        vehicle_x=-vehicle_x
        #平移
        trans_left[:,0]=-trans_left[:,0]-vehicle_x
        trans_left[:,1]=trans_left[:,1]-vehicle_y
        trans_right[:,0]=-trans_right[:,0]-vehicle_x
        trans_right[:,1]=trans_right[:,1]-vehicle_y
        trans_left=np.array(trans_left.T)
        trans_right=np.array(trans_right.T)
        #二维坐标旋转矩阵
        dicv=np.array([[np.cos(head_angle),np.sin(head_angle)],
                       [-np.sin(head_angle),np.cos(head_angle)]])
        #进行旋转变化
        trans_left=np.dot(dicv,trans_left).T
        trans_right=np.dot(dicv,trans_right).T
        #使用numpy里面的np.polyfit拟合一元二次方程，自变量为纵向坐标，因变量为横向坐标
        #对left和right的点都拟合一下
        p_left=np.poly1d(np.polyfit(trans_left[:,1],trans_left[:,0],2))#x~y
        p_right=np.poly1d(np.polyfit(trans_right[:,1],trans_right[:,0],2))#x~y
        #新建一个空的矩阵
        tran=np.array(np.zeros((len(trans_result),4)))
        #tran：纵向坐标，横向坐标，与左侧曲线交点的横向坐标，与右侧曲线交点的横向坐标
        #使用拟合的曲线方程计算的得到与左侧曲线交点的横向坐标，与右侧曲线交点的横向坐标
        tran[:,:2]=trans_result[:,[1,2]]#1,y;2,x
        tran[:,2]=p_left(tran[:,0])
        tran[:,3]=p_right(tran[:,0])
        #计算哪些目标物在两条曲线之间，判断目标物横向位置是否在与两条曲线的交点之间
        count=[]
        for i in range(len(tran)):
           if (tran[i,1]>=tran[i,3])&(tran[i,1]<=tran[i,2]):
              count.append(i)
        res=trans_result[count,:]
        #如果在，取纵向最小的为rt1
        if len(res)>0:
          rt1=res[np.argmin(res[:,0]),1:]
        else:
          rt1=[]
    else:
        #如果不满足三个条件，要找行驶方向上最近的点为cipv
        #计算下一刻行驶的航向角
        gamma=delta*pi/180
        #根据偏航角计算旋转矩阵
        dicv=np.array([[np.cos(gamma),np.sin(gamma)],
                       [-np.sin(gamma),np.cos(gamma)]])
        #取所有目标物中的x,y
        p=result[:,1:3]
        #把坐标转换到以航向为y正轴坐标系的坐标系下
        pw=p.T
        tp=np.dot(dicv,pw).T
        #result的第一列赋值为各个目标物距离原点的绝对距离，此时计算的距离，坐标系是转换后的
        result[:,0]=np.sqrt(np.square(tp[:,0])+np.square(tp[:,1]))
        #在新坐标系下筛选范围，纵向大于0，横向在1，-1之间
        res=result[np.where((tp[:,0]>0)&(tp[:,1]<=1.2)&(tp[:,1]>=-1.2))[0],:]
        if len(res)>0:
            #如果存在，取绝对距离最近的为rt1
           rt1=res[np.argmin(res[:,0]),1:]
        else:
            #否则输出空
           rt1=[]
    return r,rt1

