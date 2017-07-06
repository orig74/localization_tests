# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
from scipy.optimize import least_squares
import numpy as np
import cv2
import utils


#todo: no mater what the representation Rvrc shuld be rotation vector and shuld be converted in PnP function
#todo: finish transfer_first
def myPnP(pts3d,pts2d,K,distortion,Rvec,Tvec,estimation=None, repres='axisang',
            transfer_first=False):

    alt_est_mod='alt' in estimation
    if transfer_first:
        rotm=cv2.Rodrigues(Rvec)[0]
        pts3d=((mat(rotm)*mat(pts3d).T+Tvec.reshape((3,1)) )).T.A

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

    elif repres=='eulerang':
        #definging epsilons
        reps=np.radians([0.5,0.1,0.1])
        #reps=np.radians([180,0.001,0.001])
        zeps=0.1
        def cost(X):
            rmat=utils.eulerAnglesToRotationMatrix(X[:3])

            #estimating camera position and not T vec for creating alt bounds 
            if alt_est_mod:
                camera_pos=X[3:6]
                #Rest,_=cv2.Rodrigues(eRvec)
                #solving 0=RC+T => T=-RC
                estimate_Tvec=(-mat(rmat)*mat(camera_pos).T).A1
            else:
                estimate_Tvec=X[3:6]
            
            eRvec,_ = cv2.Rodrigues(rmat)
            ppts2d,jac=cv2.projectPoints(pts3d,eRvec,estimate_Tvec,K,distortion)
            ppts2d=ppts2d.reshape(-1,2)
            ret=(ppts2d-pts2d).flatten()
            return ret.flatten()
    else:
        print('Error bad representation')
        return None
                
    #### define defferent bounds
 
    #bounds=([-0.5,-0.5,-1,-3,-3,-3],[0.5,0.5,1,3,3,3])
    #rl=15.0/180.0*np.pi
    #bounds=([-rl,-rl,-1,-3,-3,0],[rl,rl,1,3,3,3])
    bounds=([-np.inf]*6,[np.inf]*6)

    ### different methods for least sq.

    #res=least_squares(cost,np.hstack((Rvec.flatten(),Tvec.flatten())),
    #        '3-point',method='lm',xtol=1e-12,ftol=1e-12)#,bounds=bounds)
    #res=least_squares(cost,np.hstack((Rvec.flatten(),Tvec.flatten())),\
    #            '2-point',bounds=bounds,method='trf', ftol=1e-12)
    if transfer_first:
        X0 = np.zeros(6)
    else:
        X0=np.hstack((Rvec.flatten(),Tvec.flatten()))

    
    if estimation is not None:
        if 'alt' in estimation:
            bounds[0][5]=estimation['alt']-zeps
            bounds[1][5]=estimation['alt']+zeps
            X0[5]=np.clip(X0[5],bounds[0][5],bounds[1][5])
        if 'rvec' in estimation:
            estimated_rvec=estimation['rvec'].flatten()
            prev_rvec=Rvec.flatten()
            #wight=1
            #combined_rvec=prev_rvec*wight+estimated_rvec*(1-wight)
            X0[:3]=utils.rotationMatrixToEulerAngles(cv2.Rodrigues(prev_rvec)[0])
            estimated_angs=utils.rotationMatrixToEulerAngles(cv2.Rodrigues(estimated_rvec)[0])
            bounds[0][:3]=estimated_angs-reps
            bounds[1][:3]=estimated_angs+reps
            #if((X0[:3]>bounds[1][:3]).any() or (X0[:3]<bounds[0][:3]).any()):
            #    print('`x0` is infeasible. trying estimation as x0', estimated_angs,X0[:3])
            X0[:3]=estimated_angs
            #X0[:3]=np.clip(X0[:3],bounds[0][:3],bounds[1][:3])

    res=least_squares(cost,X0,'3-point',bounds=bounds,method='trf')
    #res=least_squares(cost,X0,'3-point',method='trf')
    #res=least_squares(cost,np.hstack((Rvec.flatten(),Tvec.flatten())),\
    #            '2-point',method='dogbox')
    rot_ret=res.x[:3]

    if repres=='eulerang': 
        #conver x[:3] back to axis angle
        rot_ret=cv2.Rodrigues(utils.eulerAnglesToRotationMatrix(rot_ret))[0]

    #retuning Tvec
    if alt_est_mod:
        #solving 0=RC+T => T=-RC
        Rest,_=cv2.Rodrigues(rot_ret)
        estimate_Tvec=-Rest @ res.x[3:6].T
    else:
        estimate_Tvec=res.x[3:6]
    

    if transfer_first:
        estimate_Tvec += Tvec.flatten()
        rotm=cv2.Rodrigues(Rvec)[0]
        pts3d=((mat(rotm)*mat(pts3d).T+Tvec.reshape((3,1)) )).T.A

   #print('X=',res.message)
    return True,rot_ret,estimate_Tvec




def ___myPnP(pts3d,pts2d,K,distortion,Rvec,Tvec,estimation=None, repres='axisang',
            transfer_first=False):

    alt_est_mod='alt' in estimation
    if transfer_first:
        rotm=cv2.Rodrigues(Rvec)[0]
        pts3d=((mat(rotm)*mat(pts3d).T+Tvec.reshape((3,1)) )).T.A

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

    elif repres=='eulerang':
        #definging epsilons
        reps=np.radians([0.5,0.1,0.1])
        #reps=np.radians([180,0.001,0.001])
        zeps=0.1
        def cost(X):
            rmat=utils.eulerAnglesToRotationMatrix(X[:3])

            #estimating camera position and not T vec for creating alt bounds 
            if alt_est_mod:
                camera_pos=X[3:6]
                #Rest,_=cv2.Rodrigues(eRvec)
                #solving 0=RC+T => T=-RC
                estimate_Tvec=(-mat(rmat)*mat(camera_pos).T).A1
            else:
                estimate_Tvec=X[3:6]
            
            eRvec,_ = cv2.Rodrigues(rmat)
            ppts2d,jac=cv2.projectPoints(pts3d,eRvec,estimate_Tvec,K,distortion)
            ppts2d=ppts2d.reshape(-1,2)
            ret=(ppts2d-pts2d).flatten()
            return ret.flatten()
    else:
        print('Error bad representation')
        return None
                
    #### define defferent bounds
 
    #bounds=([-0.5,-0.5,-1,-3,-3,-3],[0.5,0.5,1,3,3,3])
    #rl=15.0/180.0*np.pi
    #bounds=([-rl,-rl,-1,-3,-3,0],[rl,rl,1,3,3,3])
    bounds=([-np.inf]*6,[np.inf]*6)

    ### different methods for least sq.

    #res=least_squares(cost,np.hstack((Rvec.flatten(),Tvec.flatten())),
    #        '3-point',method='lm',xtol=1e-12,ftol=1e-12)#,bounds=bounds)
    #res=least_squares(cost,np.hstack((Rvec.flatten(),Tvec.flatten())),\
    #            '2-point',bounds=bounds,method='trf', ftol=1e-12)
    if transfer_first:
        X0 = np.zeros(6)
    else:
        X0=np.hstack((Rvec.flatten(),Tvec.flatten()))

    
    if estimation is not None:
        if 'alt' in estimation:
            bounds[0][5]=estimation['alt']-zeps
            bounds[1][5]=estimation['alt']+zeps
            X0[5]=np.clip(X0[5],bounds[0][5],bounds[1][5])
        if 'rvec' in estimation:
            estimated_rvec=estimation['rvec'].flatten()
            prev_rvec=Rvec.flatten()
            #wight=1
            #combined_rvec=prev_rvec*wight+estimated_rvec*(1-wight)
            X0[:3]=utils.rotationMatrixToEulerAngles(cv2.Rodrigues(prev_rvec)[0])
            estimated_angs=utils.rotationMatrixToEulerAngles(cv2.Rodrigues(estimated_rvec)[0])
            bounds[0][:3]=estimated_angs-reps
            bounds[1][:3]=estimated_angs+reps
            #if((X0[:3]>bounds[1][:3]).any() or (X0[:3]<bounds[0][:3]).any()):
            #    print('`x0` is infeasible. trying estimation as x0', estimated_angs,X0[:3])
            X0[:3]=estimated_angs
            #X0[:3]=np.clip(X0[:3],bounds[0][:3],bounds[1][:3])

    res=least_squares(cost,X0,'3-point',bounds=bounds,method='trf')
    #res=least_squares(cost,X0,'3-point',method='trf')
    #res=least_squares(cost,np.hstack((Rvec.flatten(),Tvec.flatten())),\
    #            '2-point',method='dogbox')
    rot_ret=res.x[:3]

    if repres=='eulerang': 
        #conver x[:3] back to axis angle
        rot_ret=cv2.Rodrigues(utils.eulerAnglesToRotationMatrix(rot_ret))[0]

    #retuning Tvec
    if alt_est_mod:
        #solving 0=RC+T => T=-RC
        Rest,_=cv2.Rodrigues(rot_ret)
        estimate_Tvec=-Rest @ res.x[3:6].T
    else:
        estimate_Tvec=res.x[3:6]
    

    if transfer_first:
        estimate_Tvec += Tvec.flatten()
        rotm=cv2.Rodrigues(Rvec)[0]
        pts3d=((mat(rotm)*mat(pts3d).T+Tvec.reshape((3,1)) )).T.A

   #print('X=',res.message)
    return True,rot_ret,estimate_Tvec
 
