# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
from scipy.optimize import least_squares
import numpy as np
import cv2
import utils


#todo: no mater what the representation Rvrc shuld be rotation vector and shuld be converted in PnP function
#todo: finish transfer_first
def myPnP_axisAng(  pts3d, pts2d, K, distortion, 
                    estimation_vec, #0-2 Rotation matrix ,3-5 camera position (not Tvec) 
                    estimation_bounds):


    reps=np.ones(3)*np.radians(10)
    #zeps=0.3
    def cost(X):
        eRvec=X[:3]

        #estimating camera position and not T vec for creating alt bounds
        camera_pos=X[3:6]
        Rest,_=cv2.Rodrigues(eRvec)
        #solving 0=RC+T => T=-RC
        estimate_Tvec=-Rest @ camera_pos.T

        ppts2d,jac=cv2.projectPoints(pts3d,eRvec,estimate_Tvec,K,distortion)
        ppts2d=ppts2d.reshape(-1,2)
        ret=(ppts2d-pts2d).flatten()
        return ret.flatten()
    
    bounds=estimation_bounds

    X0=estimation_vec

    res=least_squares(cost,X0,'3-point',bounds=bounds,method='trf')
    return True,res.x


def myPnP_Euler(  pts3d, pts2d, K, distortion, 
                    estimation_vec, #0-2 euler angles ,3-5 camera position (not Tvec) 
                    prev_vec,
                    estimation_bounds):
    def cost(X):
        Rmat=utils.eulerAnglesToRotationMatrix(X[:3])

        #estimating camera position and not T vec for creating alt bounds
        camera_pos=X[3:6]
        #solving 0=RC+T => T=-RC
        estimate_Tvec=(-Rmat @ camera_pos.T).flatten()

        Rvec,_ = cv2.Rodrigues(Rmat)
        ppts2d,jac=cv2.projectPoints(pts3d,Rvec,estimate_Tvec,K,distortion)
        ppts2d=ppts2d.reshape(-1,2)
        ret=(ppts2d-pts2d).flatten()
        return ret.flatten()


    X0=estimation_vec

    res=least_squares(cost,X0,'3-point',bounds=estimation_bounds,method='trf')

   #print('X=',res.message)
    return True,res.x
