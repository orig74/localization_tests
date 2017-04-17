# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
from scipy.optimize import least_squares
import numpy as np
from numpy import matrix as mat
import cv2

def myPnP(pts3d,pts2d,K,distortion,Rvec,Tvec,estimation=None):

    alt_est_mod='alt' in estimation
    def cost(X):
        eRvec=X[:3]

        #estimating camera position and not T vec for creating alt bounds 
        if alt_est_mod:
            camera_pos=X[3:6]
            Rest,_=cv2.Rodrigues(eRvec)
            #solving 0=RC+T => T=-RC
            estimate_Tvec=(-mat(Rest)*mat(camera_pos).T).A1
        else:
            estimate_Tvec=X[3:6]
        
        ppts2d,jac=cv2.projectPoints(pts3d,eRvec,estimate_Tvec,K,distortion)
        ppts2d=ppts2d.reshape(-1,2)
        ret=(ppts2d-pts2d).flatten()
        return ret.flatten()
  
            
    #### define defferent bounds
 
    #bounds=([-0.5,-0.5,-1,-3,-3,-3],[0.5,0.5,1,3,3,3])
    #rl=15.0/180.0*np.pi
    #bounds=([-rl,-rl,-1,-3,-3,0],[rl,rl,1,3,3,3])
    eps=1e-3
    bounds=([-np.inf]*6,[np.inf]*6)

    ### different methods for least sq.

    #res=least_squares(cost,np.hstack((Rvec.flatten(),Tvec.flatten())),
    #        '3-point',method='lm',xtol=1e-12,ftol=1e-12)#,bounds=bounds)
    #res=least_squares(cost,np.hstack((Rvec.flatten(),Tvec.flatten())),\
    #            '2-point',bounds=bounds,method='trf', ftol=1e-12)
    X0=np.hstack((Rvec.flatten(),Tvec.flatten()))
    
    if estimation is not None:
        if 'alt' in estimation:
            X0[5]=estimation['alt']
            bounds[0][5]=estimation['alt']-eps
            bounds[1][5]=estimation['alt']+eps
        if 'rvec' in estimation:
            X0[:3]=estimation['rvec']
            bounds[0][:3]=estimation['rvec']-eps
            bounds[1][:3]=estimation['rvec']+eps

    res=least_squares(cost,X0,'3-point',bounds=bounds,method='trf')
    #res=least_squares(cost,X0,'3-point',method='trf')
    #res=least_squares(cost,np.hstack((Rvec.flatten(),Tvec.flatten())),\
    #            '2-point',method='dogbox')

    #retuning Tvec
    if alt_est_mod:
        #solving 0=RC+T => T=-RC
        Rest,_=cv2.Rodrigues(res.x[:3])
        estimate_Tvec=(-mat(Rest)*mat(res.x[3:6]).T).A1
    else:
        estimate_Tvec=res.x[3:6]
    print('X=',res.message)
    return True,res.x[:3],estimate_Tvec
 
