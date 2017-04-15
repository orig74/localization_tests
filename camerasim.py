# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4

import cv2
import numpy as np
import utils
from numpy import matrix as mat

def generate_3d_points():
    ax=np.linspace(-2,2,8)
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



class Capture(object):
    def __init__(self,camera_matrix,size,noise_model=None):
        self.K=mat(camera_matrix).reshape(3,3)
        self.size=size
        self.manuever=manuver1()
        self.last_points=None


    def read(self):
        x,y,z,rx,ry,rz=self.manuever.__next__()
        img=np.zeros((self.size[0],self.size[1],3),dtype='uint8')
        C=np.array([x,y,z])
        #0=RC+T T=-R.T*C
        
        R=utils.eulerAnglesToRotationMatrix([rx,ry,rz]) 
        T=-mat(R).T*mat(C).T
        pts=(self.K*((mat(R)*generate_3d_points().T).T+T.T).T).T
        pts/=pts[:,2]
        for ptm in pts:
            pt=list(map(int,ptm.A1)) 
            #print(pt)
            cv2.rectangle(img,(pt[0]-1,pt[1]-1),(pt[0]+1,pt[1]+1),(0,255,0),1)
        #print(pts)
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
                
