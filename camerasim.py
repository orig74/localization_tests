# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4

import cv2
import numpy as np
import utils
from numpy import matrix as mat

def generate_3d_points(spread=0.06 , npoints=3):
    ax=np.linspace(-spread,spread,npoints)
    xx,yy=np.meshgrid(ax,ax)
    return np.vstack((xx.flatten(),yy.flatten(),np.zeros(len(ax)**2))).T

def manuver1():
    #x,y,z,rx,ry,rz
    vec=np.zeros(6)
    vec[2]=3
    #vec[3]=-90/180.0*np.pi
    for _ in range(100):
        vec[2]+=0.1
        yield vec

    for _ in range(50):
        vec[3]+=1.0/180*np.pi
        yield vec

def manuver2():
    #x,y,z,rx,ry,rz
    vec=np.zeros(6)
    vec[2]=0.3
    #vec[3]=-90/180.0*np.pi
    for _ in range(50):
        vec[2]+=0.01
        yield vec

    for rot_pitch,rot_roll in [(0,0),(200,20)]:
        sz=0.004
        for ind in range(400):
            if ind==0: 
                step_x=-sz
                step_y=0
            if ind==100:
                step_x=0
                step_y=sz
            if ind==200:
                step_x=sz
                step_y=0
            if ind==300:
                step_x=0
                step_y=-sz
            vec[0]-=step_x
            vec[1]+=step_y
            
            if rot_pitch>0:
                pitch_sign=1 if (ind%rot_pitch)<(rot_pitch//2) else -1
                vec[3]+=pitch_sign*0.3/180.0*np.pi
                vec[5]+=pitch_sign*0.1/180.0*np.pi
                vec[2]+=pitch_sign*0.001
            if rot_roll>0:
                roll_sign=1 if (ind%rot_roll)<(rot_roll//2) else -1
                vec[4]+=roll_sign*0.2/180.0*np.pi
            yield vec

def manuver3():
    #x,y,z,rx,ry,rz
    vec=np.zeros(6)
    vec[2]=3
    #vec[3]=-90/180.0*np.pi
    for _ in range(50):
        vec[2]+=0.1
        yield vec

    for rot_pitch,rot_roll in [(0,0)]: 
        for ind in range(400):
            if ind==0: 
                step_x=-0.01
                step_y=0
            if ind==100:
                step_x=0
                step_y=0.01
            if ind==200:
                step_x=0.01
                step_y=0
            if ind==300:
                step_x=0
                step_y=-0.01
            vec[0]-=step_x
            vec[1]+=step_y
            yield vec

    for rot_pitch,rot_roll in [(0,30)]:             
        for ind in range(400):
            if rot_pitch>0:
                pitch_sign=1 if (ind%rot_pitch)<(rot_pitch//2) else -1
                vec[3]+=pitch_sign*0.1/180.0*np.pi
            if rot_roll>0:
                roll_sign=1 if (ind%rot_roll)<(rot_roll//2) else -1
                vec[4]+=roll_sign*0.1/180.0*np.pi
            yield vec





class Capture(object):
    def __init__(self,camera_matrix,size,noise_model=None, spread=0.06 , npoints=3):
        self.K=mat(camera_matrix).reshape(3,3)
        self.size=size
        self.manuever=manuver2()
        self.last_points=None
        self.last_position=None
        self.spread=spread
        self.npoints=npoints

    def read(self):
        try:
            self.last_position=self.manuever.__next__()
        except StopIteration:
            return False,None
        x,y,z,rx,ry,rz=self.last_position
        img=np.zeros((self.size[0],self.size[1],3),dtype='uint8')
        C=np.array([x,y,z])
        #0=RC+T T=-R.T*C
        
        R=utils.eulerAnglesToRotationMatrix([rx,ry,rz]) 
        
        T=-mat(R)*mat(C).T
        if 0:
            pts=(self.K*((mat(R)*generate_3d_points().T).T+T.T).T).T
            pts/=pts[:,2]
            pts=pts[:,:2].T
        else:
            R_vec,_=cv2.Rodrigues(R)
            pts,jac=cv2.projectPoints(generate_3d_points(self.spread , self.npoints),R_vec,T.A1,self.K,np.zeros(5))
            pts=pts.reshape(-1,2)
 
        for ptm in pts:
            #pt=list(map(int,ptm.A1)) 
            pt=list(map(int,ptm))
            cv2.rectangle(img,(pt[0]-1,pt[1]-1),(pt[0]+1,pt[1]+1),(0,255,0),1)
        self.last_points=pts[:,:2]
        return True,img

    def track(self,*args,**kargs):
        return self.last_points
          
if __name__=='__main__':
    cap=Capture([160.0,0,160, 0,160.0,120.0,0,0,1],(240,320))
    while 1:
        ret,im=cap.read()
        if ret:
            cv2.imshow('img',im)
            k=cv2.waitKey(0)%256
            if k==27:
                break
        else:
            import pdb;pdb.set_trace()
                
