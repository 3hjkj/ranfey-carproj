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
cdef extern from "math.h" nogil:
    float cosf(float)
    float sinf(float)
    float sqrtf(float)
    float powf(float x,float y)
    float ceilf(float)
    float floorf(float)

#r-fans
@cython.boundscheck(False) 
@cython.wraparound(False) 
def pcl_points_rf(pc):
    cdef np.ndarray[np.float64_t, ndim=2] points=np.zeros((pc.shape[0],3),dtype=np.float64)
    points[:,0]=pc['x']
    points[:,1]=pc['y']
    points[:,2]=pc['z']
    return points


#robosense
@cython.boundscheck(False) 
@cython.wraparound(False) 
def pcl_points_rs(pc):
    cdef np.ndarray[np.float64_t, ndim=2] points=np.zeros((pc.shape[0]*pc.shape[1],3),dtype=np.float64)
    points[:,0]=pc['x'].reshape(-1)
    points[:,1]=pc['y'].reshape(-1)
    points[:,2]=pc['z'].reshape(-1)
    return points

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
    cdef np.float64_t car_min_x=-2
    cdef np.float64_t car_max_x=0
    cdef np.float64_t car_min_y=-0.9
    cdef np.float64_t car_max_y=0.9
    cdef np.float64_t x_min=-20
    cdef np.float64_t x_max=60
    cdef np.float64_t y_max=6
    cdef np.float64_t y_min=-6
    cdef np.float64_t z_max=1
    points_cloud=points_cloud[np.where((points_cloud[:,0]>x_min)&(points_cloud[:,0]<x_max)&(points_cloud[:,1]>y_min)&
                                       (points_cloud[:,1]<y_max)&(points_cloud[:,2]<z_max))[0],:]
    points_cloud=points_cloud[np.where(
            ~((points_cloud[:,0]>=car_min_x)&(points_cloud[:,0]<= car_max_x)&
              (points_cloud[:,1]>=car_min_y)&(points_cloud[:,1]<= car_max_y)))[0],:]
    
    points_cloud[:,0]=points_cloud[:,0]-2.55
    return points_cloud

@cython.boundscheck(False) 
@cython.wraparound(False) 
cdef downsample_circle(DTYPE_t[:,::1] xel, np.int64_t[::1] idex):
    cdef int i,j,n,ln=0,lm=xel.shape[0]
    cdef DTYPE_t[:,::1] tmp
    cdef DTYPE_t[:,::1] points_update2 = np.zeros((lm,3),dtype=DTYPE)
    cdef DTYPE_t[::1] tsum = np.zeros(3, DTYPE)
    for i in range(idex.shape[0]):
        if i+1 < idex.shape[0]: tmp = xel[idex[i]:idex[i+1]]
        else: tmp = xel[idex[i]:]
        if tmp.shape[0] > 1: 
           for j in range(3):
              for n in range(tmp.shape[0]):
                 tsum[j]+=tmp[n,j]
           for j in range(3):
              points_update2[i,j]=tsum[j] / tmp.shape[0]
              tsum[j]=0
        ln = ln+1
    return points_update2[:ln]


@cython.boundscheck(False) 
@cython.wraparound(False) 
def get_downsample(points_update, Cellsize=10):
    cdef np.ndarray[DTYPE_t, ndim=2] voxel=np.zeros((points_update.shape[0],6),dtype=DTYPE)
    cdef np.ndarray[DTYPE_t, ndim=2] vo=np.zeros((points_update.shape[0],3),dtype=DTYPE)
    cdef np.ndarray[DTYPE_t, ndim=2] xel=np.zeros((points_update.shape[0],3),dtype=DTYPE)
    cdef np.int64_t[::1] idex
    cdef np.ndarray[DTYPE_t, ndim=2] points_update2
    voxel[:,3:]=points_update[:, 0:3]
    voxel[:,0:3]=np.ceil((voxel[:,3:])/Cellsize) 
    voxel = voxel[np.lexsort([voxel[:,2],voxel[:,1],voxel[:,0]])]
    vo, xel = np.ascontiguousarray(voxel[:,0:3]), np.ascontiguousarray(voxel[:, 3:])
    idex =np.where(np.insert(np.any(np.diff(vo, axis=0), axis=1), 0, True)==True)[0]
    
    points_update2 =np.asarray(downsample_circle(xel, idex))
    return points_update2[points_update2.any(axis=1)]


@cython.boundscheck(False) 
@cython.wraparound(False) 
cdef grid_circle(DTYPE_t[:,::1] voxel, np.int64_t[::1] idex):
    cdef DTYPE_t[:,::1] points_update3 = np.zeros((voxel.shape[0],6),dtype=DTYPE) 
    cdef DTYPE_t[:,::1] tmp
    cdef int i,j,m,n,ln=0
    cdef DTYPE_t zmax
    cdef DTYPE_t zmin
    for i in range(idex.shape[0]):
        
        if i+1 < idex.shape[0]: tmp = voxel[idex[i]:idex[i+1]]
        else: tmp = voxel[idex[i]:]
        zmax,zmin=-1000,1000
        for j in range(tmp.shape[0]):
           zmax=max(zmax,tmp[j,4])
           zmin=min(zmin,tmp[j,4])
        
        if 40 <= zmax-zmin <= 300: 
            for m in range(tmp.shape[0]):
               for n in range(5): points_update3[ln+m,n] = tmp[m,n]
               points_update3[ln+m,5] = zmax
            ln += tmp.shape[0] 
            #points_update3[idex[i]:idex[i]+len(tmp)]=np.hstack((tmp, zmax * np.ones((len(tmp),1))))
    return points_update3[:ln]


@cython.boundscheck(False) 
@cython.wraparound(False) 
def grid(points_update2, Cell=40):
    cdef np.ndarray[DTYPE_t, ndim=2] voxel = np.zeros((points_update2.shape[0],5),dtype=DTYPE)
    cdef np.int64_t[::1] idex
    cdef np.ndarray[DTYPE_t, ndim=2] points_update3
    voxel[:,2:]=points_update2 
    voxel[:,0]=np.ceil((voxel[:,2])/Cell)  
    voxel[:,1]=np.ceil((voxel[:,3])/Cell) 
    voxel = voxel[np.lexsort([voxel[:,1],voxel[:,0]])] 
    idex = np.where(np.insert(np.any(np.diff(voxel[:,:2],axis=0),axis=1),0,True)==True)[0] 
    voxel=np.ascontiguousarray(voxel)
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
        seed_grid2 = grid_slice[idex2[i]]
        lt2 = 0
        for j in range(target_points.shape[0]):
           if target_points[j,4] == 0: break
           lt2 += 1
        tmp2 = points_slice[idex2[i]]
        target_points[lt2:lt2+tmp2.shape[0]] = tmp2 
        idex_round2, idex2_2, lr2 = np.zeros(8, np.int32), np.zeros(8, np.int32), 0 
        
        for j in range(idex.shape[0]):
           if ((abs(grid_slice[idex[j],0]-seed_grid2[0]) <= 1) & (abs(grid_slice[idex[j],1]-seed_grid2[1]) <= 1) &(abs(grid_slice[idex[j],2]-seed_grid2[2]) <= 70)):
              idex_round2[lr2], idex2_2[lr2], lr2 = j, idex[j], lr2+1
        idex = np.delete(idex, idex_round2[:lr2])
        #idex = np.delete(np.asarray(idex), np.asarray(idex_round2[:lr2]))
        idex, target_points = cluster_circle2(points_slice, grid_slice, target_points, idex, idex2_2[:lr2])
    return idex, target_points



@cython.boundscheck(False) 
@cython.wraparound(False) 
def cluster_circle(object points_slice, DTYPE_t[:,::1] grid_slice,DTYPE_t[::1] idex,np.float64_t ti):
     
    cdef int row=0
    cdef int i,lr,li=idex.shape[0]
    cdef np.float64_t[:,::1] target_output= np.zeros((li,7)) 
    cdef DTYPE_t[:,::1] tmp,target_points
    cdef DTYPE_t[::1] idex2,idex_round
    cdef DTYPE_t[::1] seed_grid
    cdef DTYPE_t xmin, ymin, zmin,xmax, ymax, zmax,xc,yc,zc
    while idex.shape[0] > 0:
        target_points = np.zeros((li*50,6),dtype=DTYPE)
        #print(grid_slice[idx, 0:3])
        seed_grid = grid_slice[idex[0]]
        tmp = points_slice[idex[0]]
        target_points[:tmp.shape[0]] = tmp
        #idex = np.delete(np.asarray(idex), np.array([0]))
        idex = np.delete(idex, np.array([0]))
        # print (idex)

        idex_round, idex2, lr = np.zeros(8, np.int32), np.zeros(8, np.int32), 0
        for i in range(idex.shape[0]):
           if ((abs(grid_slice[idex[i],0]-seed_grid[0]) <= 1) & (abs(grid_slice[idex[i],1]-seed_grid[1]) <= 1) &(abs(grid_slice[idex[i],2]-seed_grid[2]) <=70)): 
              idex_round[lr], idex2[lr], lr = i, idex[i], lr+1
        #idex_round = np.where(np.logical_and(np.abs(grid_slice[idex,0]-seed_grid[0])<=1, 
                                             #np.abs(grid_slice[idex,1]-seed_grid[1])<=1) & 
                             #(np.abs(grid_slice[idex,2] - seed_grid[2]) <=0.6))[0] 
        #print ('idex_round:', idex_round)
        idex = np.delete(idex, idex_round[:lr])
        #idex = np.delete(np.ctypeslib.as_array(idex), np.ctypeslib.as_array(idex_round[:lr]))
        idex2 = idex2[:lr]
        
        idex, target_points= cluster_circle2(points_slice, grid_slice, target_points, idex, idex2)
        xmin, ymin, zmin = target_points[0,2], target_points[0,3], target_points[0,4]
        xmax, ymax, zmax = target_points[0,2], target_points[0,3], target_points[0,4]
        
        for i in range(1, target_points.shape[0]):
            if target_points[i,4] == 0: break
            xmin, xmax = min(xmin, target_points[i,2]), max(xmax, target_points[i,2])
            ymin, ymax = min(ymin, target_points[i,3]), max(ymax, target_points[i,3])
            zmin, zmax = min(zmin, target_points[i,4]), max(zmax, target_points[i,4])
            
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
    
    points_slice = np.array(np.vsplit(usepoints, idex[1:])) 
    idex = (np.arange(points_slice.shape[0])).astype(np.int32)
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


@cython.boundscheck(False) 
@cython.wraparound(False) 
def updateF(np.ndarray[DTYPE_s, ndim=2] F,DTYPE_s dt):
    F[0, 2], F[1, 3]  = dt, dt
    return F

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

@cython.boundscheck(False) 
@cython.wraparound(False) 
def update_optimal(np.ndarray[DTYPE_s, ndim=2] x_1,np.ndarray[DTYPE_s, ndim=2] z,np.ndarray[DTYPE_s, ndim=2] H,np.ndarray[DTYPE_s, ndim=2] R,np.ndarray[DTYPE_s, ndim=2] P,np.ndarray[DTYPE_s, ndim=2] G):
    cdef np.ndarray[DTYPE_s, ndim=2] S,PHt,K
    S,PHt=get_S(H,R,P)
    K = np.dot(PHt,S)
    x_1,P=get_P(x_1,z,H,K,P,G)
    return x_1,P


@cython.boundscheck(False) 
@cython.wraparound(False) 
def get_L2(DTYPE_s x1,DTYPE_s y1,DTYPE_s x2,DTYPE_s y2):
    cdef DTYPE_s dx,dy,d
    dx=x1-x2
    dy=y1-y2
    d=np.sqrt(dx**2+dy**2)
    return d

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
        para[i,0]=get_L2(x_1[2,0],x_1[3,0],d[i,1],d[i,2])
        para[i,1]=cosine_similarity([x_1[2,0],x_1[3,0]],[d[i,1],d[i,2]])
        para[i,2]=abs(x_1[3,0]-d[i,2])
        para[i,3]=abs(x_1[2,0]-d[i,1])
    return para

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


@cython.boundscheck(False) 
@cython.wraparound(False) 
def get_result(result):
    result=np.mat(result)
    left,right=1.2,1.2
    a=np.zeros((len(result),1)) 
    r=np.zeros((1,8))
    result=np.column_stack((a,result))
    result[:,0]=np.sqrt(np.square(result[:,2])+np.square(result[:,3]))
    leftfront=result[np.where((result[:,2]>0)&(result[:,3]>left))[0],:]
    if len(leftfront)>0:
        leftfront=leftfront[np.argmin(leftfront[:,0]),:]
        r=np.concatenate((r,leftfront),axis=0)
    front=result[np.where((result[:,2]>0)&(result[:,3]<=left)&(result[:,3]>=-right))[0],:]
    if len(front)>0:
        #print(front)
        #print(r)
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

@cython.boundscheck(False) 
@cython.wraparound(False) 
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

@cython.boundscheck(False) 
@cython.wraparound(False) 
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

@cython.boundscheck(False) 
@cython.wraparound(False) 
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
@cython.boundscheck(False) 
@cython.wraparound(False) 
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

@cython.boundscheck(False) 
@cython.wraparound(False) 
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
    left,right=1.2,1.2
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
        res=result[np.where((tp[:,0]>0)&(tp[:,1]<=1.2)&(tp[:,1]>=-1.2))[0],:]
        if len(res)>0:
           rt1=res[np.argmin(res[:,0]),1:]
        else:
           rt1=[]
    return r,rt1

