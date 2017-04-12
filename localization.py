# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
import pandas as pd
import numpy as np
import cv2
import math
from scipy.optimize import least_squares
import time
import traceback,sys
import pickle,os

N_FTRS=400

#start pix (sx,sy) curr image pixel (cx,cy) 
ftr_columns=['cx','cy','sx','sy','ex','ey','ez']
prev_frame=None
features_state=None
def get_s():
    return np.array(features_state.loc[:,'sx':'sy'],dtype='float32').reshape(-1,2)
def get_c():
    return np.array(features_state.loc[:,'cx':'cy'],dtype='float32').reshape(-1,2)
def get_e():
    return np.array(features_state.loc[:,'ex':'ez'],dtype='float32').reshape(-1,3)


colors = np.random.randint(0,255,(2000,3))
#orb=cv2.ORB_create()

def init_of(img):
    global features_state,prev_frame
    gray=cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    p0=cv2.goodFeaturesToTrack(gray,N_FTRS,0.01,20).reshape(-1,2)
    #p0,des=orb.detectAndCompute(gray,None)
    #p0=np.array(list(map(lambda x:x.pt,p0)))
    features_state=pd.DataFrame(index=range(p0.shape[0]),columns=ftr_columns)
    features_state.loc[:,'sx':'sy']=p0
    features_state.loc[:,'cx':'cy']=p0
    prev_frame=gray

def calc_of(img):
    global prev_frame,features_state
    lk_params = dict( winSize  = (15,15),
               maxLevel = 3,
           criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.08))
    gray=cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    #features_state=features_state[pd.notnull(features_state['cx'])]
    p0=get_c().reshape(-1,1,2)
    p1, st, err = cv2.calcOpticalFlowPyrLK(prev_frame, gray, p0, None, **lk_params)

    #cleanning...
    features_state.loc[:,'cx':'cy']=p1.reshape(-1,2)
    features_state=features_state[st.flatten()==1]
    prev_frame=gray

def draw_ftrs(img):
    ftr_pos=np.array(features_state.loc[:,'cx':'cy'],dtype='float32').reshape(-1,2)
    indices=features_state.index.tolist()
    for i,ftr in zip(indices,ftr_pos):
         a,b = ftr
         cv2.circle(img,(a,b),2,colors[i].tolist(),-1)

def isRotationMatrix(R) :
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6
                     
                     
def rotationMatrixToEulerAngles(R) :
    assert(isRotationMatrix(R))
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
    singular = sy < 1e-6
    if not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else:
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0
    return np.array([x, y, z])



def recover_pos(clean=True):
    global features_state
    p1=get_s()
    p2=get_c()
    E,mask=cv2.findEssentialMat(p1,p2,K,cv2.RANSAC,0.99,1.0)
    ret,R,T,mask=cv2.recoverPose(E,p1,p2,K,mask)
    #clean..

    if clean:
        print('cleanning db')
        features_state=features_state[mask.ravel()==255]
    return ret,R,T
   
def triangulate(R,T):
    global features_state
    p1=get_s()
    p2=get_c()
    
    print('trangulating..')
    estimated_alt=1.0
    T=T.ravel()
    Trans=np.array([T[0]/T[2], T[1]/T[2], 1.0]) * estimated_alt 

    
    p1=cv2.undistortPoints(p1.reshape(-1,1,2),K,distortion).reshape(-1,2)
    p2=cv2.undistortPoints(p2.reshape(-1,1,2),K,distortion).reshape(-1,2)

    Proj1=np.hstack((np.eye(3),np.zeros((3,1))))
    Proj2=np.hstack((R,Trans.reshape(3,1)))
    pts=cv2.triangulatePoints(Proj1,Proj2,p1.T,p2.T)
    pts=pts.T
    pts=(pts.T/pts[:,3]).T
    pts3d=pts[:,:3]
    #import pdb;pdb.set_trace()
    features_state.loc[:,'ex':'ez']=pts3d.reshape(-1,3)
 
    #print('{:3} {:>5.2f} {:>5.2f} {:>5.2f}'.format(ret,*(rotationMatrixToEulerAngles(R)*180/math.pi)))

Rvec,Tvec=None,None

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


    tic=time.time()
    #res=least_squares(cost,np.hstack((Rvec.flatten(),Tvec.flatten())),
    #        '2-point',method='lm',xtol=1e-12,ftol=1e-12)#,bounds=bounds)
    #res=least_squares(cost,np.hstack((Rvec.flatten(),Tvec.flatten())),\
    #            '2-point',bounds=bounds,method='trf', ftol=1e-12)
    res=least_squares(cost,np.hstack((Rvec.flatten(),Tvec.flatten())),\
                '2-point',method='trf')
    #res=least_squares(cost,np.hstack((Rvec.flatten(),Tvec.flatten())),\
    #            '2-point',method='dogbox')
    #import pdb;pdb.set_trace()
    print('X=',res.message,time.time()-tic)
    return True,res.x[:3],res.x[3:6]
   


def solve_pos(estimateR):

    global Rvec,Tvec
    try:
        p2=get_c()
        #import pdb;pdb.set_trace()

        ### add noise

        #p2=p2+np.random.normal(0,5,p2.shape)
        ###


        pts3d=get_e()
        if Rvec is None:
            resPnP,Rvec,Tvec=cv2.solvePnP(pts3d,p2,K,distortion)
        else:
            resPnP,Rvec,Tvec=cv2.solvePnP(pts3d,p2,K,distortion,Rvec,Tvec,True)
            #resPnP,Rvec,Tvec=myPnP(pts3d,p2,K,distortion,Rvec,Tvec)
            #resPnP,Rvec,Tvec,inliers=cv2.solvePnPRansac(pts3d,p2,K,distortion,Rvec,Tvec,True)
            

        #if resPnP:
        #    print('len=',len(features_state),np.ravel(Tvec))
        return resPnP,Rvec,Tvec.flatten()
    except Exception:
        print("Exception in user code:")
        print("-"*60)
        traceback.print_exc(file=sys.stdout)
        print("-"*60)

        import pdb;pdb.set_trace()



import viewer

from grabber import file_grabber

def main():
    global K,distortion
    ground_truth=None
    np.set_printoptions(formatter={'all':lambda x: '{:10.3f}'.format(x)})
    if 0:
        K=np.array( [ 5.5061780702480894e+02, 0., 3.1950000000000000e+02, 0.,
               5.5061780702480894e+02, 2.3950000000000000e+02, 0., 0., 1. ]).reshape((3,3)) #sony

        distortion=np.array([-1.4562697048176954e-01, 1.4717208761705844e-01, 0., 0.,-3.1843325596064148e-03])
        if 0:
            cap=cv2.VideoCapture(1)
            cap.set(cv2.CAP_PROP_FPS,60)
        else:
            cap=file_grabber('output.mkv')
    else: #ue4
        K=np.array([160.0,0,160, 0,160.0,120.0,0,0,1]).reshape((3,3))
        distortion=np.zeros(5)
        #cap=file_grabber('output_ue4.avi')
        base_name='manuever2'
        cap=file_grabber(base_name+'.avi')
        if os.path.isfile(base_name+'.pkl'):
            ground_truth=open(base_name+'.pkl','rb')
        #skip a few frames 
        for _ in range(50):
            cap.read()
            if ground_truth:
                gt_pos_data=pickle.load(ground_truth)
                if gt_pos_data:
                    start_alt=gt_pos_data['posz']

        
    view3d=viewer.plot3d()
    view3d.__next__()  




    cnt=0
    start_recover=False
    alt_tresh=-1
    while 1:
        k=cv2.waitKey(1)
        if k!=-1:
            #print('k=',k%256)
            k=k%256
        if k==27:
            #if start_recover:
            #    view3d.send(('stop',None))
            break
        ret,img=cap.read()
        if ground_truth:
            try:
                gt_pos_data=pickle.load(ground_truth)
                if gt_pos_data['posz']-start_alt > 1.5:
                    alt_tresh=gt_pos_data['posz']-start_alt
                Tvec_gt=np.array([gt_pos_data['posy'],-gt_pos_data['posx'],gt_pos_data['posz']])
                R_gt=None #TODO
                view3d.send(('camera_gt',(R_gt,Tvec_gt)))
            except EOFError:
                pass
            #print('gt=',gt_pos_data)
        if ret:
            if cnt==0:
                init_of(img)
            else:
                calc_of(img)
            cnt+=1
            draw_ftrs(img)
            cv2.imshow('img',img)
            if k==ord('a'):
                alt_tresh=1.0
            if alt_tresh>0 and not start_recover:
                ret,R,T=recover_pos()
                triangulate(R,T)
                start_recover=True
            if start_recover:
                #ret,R,T=recover_pos(clean=False)
                resPnP,Rvec,Tvec=solve_pos(None)
                if resPnP:
                    R,_=cv2.Rodrigues(Rvec)
                    view3d.send(('camera',(R,Tvec.ravel())))
                    pts3d=get_e()
                    view3d.send(('pts3d',pts3d))

        #else:
        #    print('Error no image')
            #break

 



if __name__=='__main__':
    import cProfile
    cProfile.run('main()','mainstats')
    import pstats
    p = pstats.Stats('mainstats')
    p.strip_dirs().sort_stats('cumulative').print_stats(100)

