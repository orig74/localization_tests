# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
from scipy.optimize import least_squares,basinhopping
import numpy as np
import cv2
import utils


#todo: no mater what the representation Rvrc shuld be rotation vector and shuld be converted in PnP function
#todo: finish transfer_first
def myPnP_axisAng(  pts3d, pts2d, K, distortion, 
                    estimation_vec, #0-2 Rotation matrix ,3-5 camera position (not Tvec) 
                    estimation_bounds):


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
        #ret=(ppts2d-pts2d).flatten()
        ret=(ppts2d-pts2d)
        return ret.flatten()
    
    bounds=estimation_bounds

    X0=estimation_vec

    res=least_squares(cost,X0,'3-point',bounds=bounds,method='trf')
    return True,res.x


def myPnP_Euler(  pts3d, pts2d, K, distortion, 
                    estimation_vec, #0-2 euler angles ,3-5 camera position (not Tvec) 
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
        ret=(ppts2d-pts2d)
        #return (ret[:,0]**2+ret[:,1]**2).sum()
        #return np.median(ret,axis=0)#ret.flatten()
        return ret.flatten()


    X0=estimation_vec

    res=least_squares(cost,X0,'3-point',bounds=estimation_bounds,method='trf')
    #res=least_squares(cost,X0,'3-point',bounds=estimation_bounds,method='dogbox')
    #import pdb;pdb.set_trace()
    #res=basinhopping(cost,X0 )
    #import pdb;pdb.set_trace()
    #res=brute(cost,zip(*estimation_bounds))

    #print('X=',res.message)
    #print('X=',res.x)
    return True,res.x


class ContSolvePnP(object):
    def __init__(self, K,distortion):
        self.K=K
        self.distortion=distortion

class ContSolvePnPopenCV(ContSolvePnP):
    def __init__(self,*args,**kargs):
        super().__init__(*args,**kargs)
        self.X0= np.array(\
                        [   0.0,0.0,0.0, #rvec
                            0.0,0.0,-7.0, # strting at positive alt Tvec=-Camera_pos
                            ])

    def solve(self,pts3d,p2):
            Rvec=self.X0[:3]
            Tvec=self.X0[3:]
            resPnP,Rvec,Tvec=cv2.solvePnP(pts3d,p2,self.K,self.distortion,Rvec,Tvec,True)
            self.X0[:3]=Rvec
            self.X0[3:]=Tvec
            return resPnP,Rvec,Tvec


class ContSolvePnPBound(ContSolvePnP):
    def __init__(self, rotation_bounds, *args, **kargs):
        super().__init__(*args,**kargs)
        self.X0= np.array(\
                        [   0.0,0.0,0.0, #rvec
                            0.0,0.0,0.5, # strting at positive alt Tvec=-Camera_pos
                            ])
        self.rotation_bounds=rotation_bounds


class ContSolvePnPAxisAng(ContSolvePnPBound):
    def solve(self,pts3d,p2,estimate):
        Rvec = estimate.get('rvec',self.X0[:3])
        Rmat,_=cv2.Rodrigues(Rvec)
        cam_pos = self.X0[3:]
        bounds=([-np.inf]*6,[np.inf]*6) 
        if 'alt' in estimate:
            cam_pos[2]=estimate['alt'] 
            zeps=0.1
            bounds[0][5]=cam_pos[2]-zeps
            bounds[1][5]=cam_pos[2]+zeps

        if 'rvec' in estimate:
            reps=np.radians(self.rotation_bounds)
            bounds[0][:3] = Rvec - reps
            bounds[1][:3] = Rvec + reps
        estimation_vec = np.clip(self.X0,bounds[0],bounds[1])
        resPnP,res=myPnP_axisAng(pts3d,p2,self.K,self.distortion,
                    estimation_vec, bounds)
        Rvec = res[:3]
        Rmat,_ = cv2.Rodrigues(Rvec)
        Tvec = -Rmat @ res[3:] #convert camera position to Tvec to be compatible with solvepnp
        return resPnP,Rvec,Tvec

class ContSolvePnPEulerAng(ContSolvePnPBound):
    def solve(self,pts3d,p2,estimate):
        Rvec = estimate.get('rvec',self.X0[:3])
        Rmat,_=cv2.Rodrigues(Rvec)
        cam_pos = self.X0[3:]
            #cam_pos = (-Rmat.T @ Tvec).flatten()
        bounds=([-np.inf]*6,[np.inf]*6) 

        if 'alt' in estimate:
            cam_pos[2]=estimate['alt']
            zeps=0.1
            bounds[0][5]=cam_pos[2]-zeps
            bounds[1][5]=cam_pos[2]+zeps

        eu_angls=utils.rotationMatrixToEulerAngles(Rmat)
                
        if 'rvec' in estimate:
            reps=np.radians(self.rotation_bounds)
            #yaw pitch roll !!! todo: solve the 180 problem maybe transfer point first!!!
            bounds[0][:3] = eu_angls - reps
            bounds[1][:3] = eu_angls + reps
        
        estimation_vec = np.clip(self.X0,bounds[0],bounds[1])
        
        resPnP,res=myPnP_Euler(pts3d,p2,self.K,self.distortion,
                    estimation_vec, bounds)
        self.X0=res
        Rvec,_=cv2.Rodrigues(utils.eulerAnglesToRotationMatrix(res[:3]))

            #converting camera position to Tvec
        Rmat,_=cv2.Rodrigues(Rvec)
        Tvec = -Rmat @ res[3:]
        return resPnP,Rvec,Tvec
 
        





