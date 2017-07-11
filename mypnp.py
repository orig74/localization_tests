# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
from scipy.optimize import least_squares
import numpy as np
import cv2
import utils


#todo: no mater what the representation Rvrc shuld be rotation vector and shuld be converted in PnP function
#todo: finish transfer_first
def myPnP(pts3d,pts2d,K,distortion,Rvec,Tvec,estimation=None, repres='axisang'):

    alt_est_mod='alt' in estimation

    estimated_rvec=Rvec if estimation is None or 'rvec' not in estimation else estimation['rvec'].flatten()
    

    if repres=='axisang':
        reps=np.ones(3)*np.radians(2)
        zeps=0.1
        def cost(X):
            eRvec=X[:3]

            #estimating camera position and not T vec for creating alt bounds 
            if alt_est_mod:
                camera_pos=X[3:6]
                Rest,_=cv2.Rodrigues(eRvec)
                #solving 0=RC+T => T=-RC
                estimate_Tvec=-Rest @ camera_pos.T
            else:
                estimate_Tvec=X[3:6]
            
            ppts2d,jac=cv2.projectPoints(pts3d,eRvec,estimate_Tvec,K,distortion)
            ppts2d=ppts2d.reshape(-1,2)
            ret=(ppts2d-pts2d).flatten()
            return ret.flatten()
    else:
        print('Error bad representation')
        return None
                
    bounds=([-np.inf]*6,[np.inf]*6)

    X0=np.hstack((estimated_rvec.flatten(),Tvec.flatten()))

    if estimation is not None:
        if 'alt' in estimation:
            bounds[0][5]=estimation['alt']-zeps
            bounds[1][5]=estimation['alt']+zeps
            X0[5]=np.clip(X0[5],bounds[0][5],bounds[1][5])
        if 'rvec' in estimation:
            bounds[0][:3]=estimated_rvec-reps
            bounds[1][:3]=estimated_rvec+reps
    

    res=least_squares(cost,X0,'3-point',bounds=bounds,method='trf')
    rot_ret=res.x[:3]

    #retuning Tvec
    if alt_est_mod:
        #solving 0=RC+T => T=-RC
        Rest,_=cv2.Rodrigues(rot_ret)
        estimate_Tvec=-Rest @ res.x[3:6].T
    else:
        estimate_Tvec=res.x[3:6]
    
   #print('X=',res.message)
    return True,rot_ret,estimate_Tvec
