# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
from scipy.optimize import least_squares
import numpy as np
import cv2

def myPnP(pts3d,pts2d,K,distortion,Rvec,Tvec):
    def cost(X):
        eRvec=X[:3]
        eTvec=X[3:6]
        ppts2d,jac=cv2.projectPoints(pts3d,eRvec,eTvec,K,distortion)
        ppts2d=ppts2d.reshape(-1,2)

        #apply radial distance cost
        if 1:
            if 0:
                rcost=np.sqrt(((ppts2d-ppts2d.mean(axis=0))**2).sum(axis=1))
                ret=(ppts2d-pts2d)*rcost.reshape(-1,1)
            else:
                #ret=np.sqrt(((ppts2d-pts2d)**2).sum(axis=1))
                ret=(ppts2d-pts2d).flatten()
                #import pdb;pdb.set_trace()
                #ddd
                #ret=np.mean(np.abs(ppts2d-pts2d),axis=0)#.sum(axis=0)
        else:
            ret=np.abs(ppts2d-pts2d)
            mm=pts2d.mean(axis=0)
            t1=pts2d[:,0]>mm[0]
            t2=pts2d[:,1]>mm[1]
            q1=ret[t1 & t2].sum(axis=0)
            q2=ret[t1 & ~t2].sum(axis=0)
            q3=ret[~t1 & t2].sum(axis=0)
            q4=ret[~t1 & ~t2].sum(axis=0)
            ret=np.hstack((q1,q2,q3,q4))
            #import pdb;pdb.set_trace()
            #dddd
            
        return ret.flatten()
    
    #bounds=([-0.5,-0.5,-1,-3,-3,-3],[0.5,0.5,1,3,3,3])
    rl=15.0/180.0*np.pi
    bounds=([-rl,-rl,-1,-3,-3,0],[rl,rl,1,3,3,3])

    #bounds=([-np.inf,-np.inf,-np.inf,-3,-3,-3],[np.inf,np.inf,np.inf,3,3,3])


    #res=least_squares(cost,np.hstack((Rvec.flatten(),Tvec.flatten())),
    #        '2-point',method='lm',xtol=1e-12,ftol=1e-12)#,bounds=bounds)
    #res=least_squares(cost,np.hstack((Rvec.flatten(),Tvec.flatten())),\
    #            '2-point',bounds=bounds,method='trf', ftol=1e-12)
    res=least_squares(cost,np.hstack((Rvec.flatten(),Tvec.flatten())),\
                '2-point',method='trf')
    #res=least_squares(cost,np.hstack((Rvec.flatten(),Tvec.flatten())),\
    #            '2-point',method='dogbox')
    #import pdb;pdb.set_trace()
    print('X=',res.message)
    return True,res.x[:3],res.x[3:6]
 

