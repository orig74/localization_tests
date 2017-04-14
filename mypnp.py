# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
from scipy.optimize import least_squares
import numpy as np
import cv2

def myPnP(pts3d,pts2d,K,distortion,Rvec,Tvec,estimation=None):
    def costf(X):
        eRvec=X[:3]
        eTvec=X[3:6]
        ppts2d,jac=cv2.projectPoints(pts3d,eRvec,eTvec,K,distortion)
        ppts2d=ppts2d.reshape(-1,2)
        ret=(ppts2d-pts2d).flatten()
        return ret.flatten()
  
    def costf_alt_est(X):
        eRvec=X[:3]
        eTvec=np.zeros(3)
        eTvec[:2]=X[3:5]
        eTvec[2]=estimation['alt']
        ppts2d,jac=cv2.projectPoints(pts3d,eRvec,eTvec,K,distortion)
        ppts2d=ppts2d.reshape(-1,2)
        ret=(ppts2d-pts2d).flatten()
        return ret.flatten()
 
            
    #### define defferent bounds
 
    #bounds=([-0.5,-0.5,-1,-3,-3,-3],[0.5,0.5,1,3,3,3])
    rl=15.0/180.0*np.pi
    bounds=([-rl,-rl,-1,-3,-3,0],[rl,rl,1,3,3,3])

    #bounds=([-np.inf,-np.inf,-np.inf,-3,-3,-3],[np.inf,np.inf,np.inf,3,3,3])

    ### different methods for least sq.

    #res=least_squares(cost,np.hstack((Rvec.flatten(),Tvec.flatten())),
    #        '3-point',method='lm',xtol=1e-12,ftol=1e-12)#,bounds=bounds)
    #res=least_squares(cost,np.hstack((Rvec.flatten(),Tvec.flatten())),\
    #            '2-point',bounds=bounds,method='trf', ftol=1e-12)
    if estimation is not None:
        if 'alt' in estimation:
            X0=np.hstack((Rvec.flatten(),Tvec.flatten()[:2]))
            cost=costf_alt_est
    else:
            X0=np.hstack((Rvec.flatten(),Tvec.flatten()))
            cost=costf

    res=least_squares(cost,X0,'3-point',method='trf')
    #res=least_squares(cost,np.hstack((Rvec.flatten(),Tvec.flatten())),\
    #            '2-point',method='dogbox')
    print('X=',res.message)
    return True,res.x[:3],res.x[3:6]
 
