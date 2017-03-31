# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
import pandas as pd
import numpy as np
import cv2
import math

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
K=np.array( [ 5.5061780702480894e+02, 0., 3.1950000000000000e+02, 0.,
           5.5061780702480894e+02, 2.3950000000000000e+02, 0., 0., 1. ]).reshape((3,3)) #sony

distortion=np.array([-1.4562697048176954e-01, 1.4717208761705844e-01, 0., 0.,-3.1843325596064148e-03])
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



def recover_pos():
    global features_state
    p1=get_s()
    p2=get_c()
    E,mask=cv2.findEssentialMat(p1,p2,K,cv2.RANSAC,0.99,1.0)
    ret,R,T,mask=cv2.recoverPose(E,p1,p2,K,mask)
    #clean..

    print('ret=',ret)
    print('cleanning db')
    features_state=features_state[mask.ravel()==255]
    
    print('retake points after cleanning')
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
 
    print('{:3} {:>5.2f} {:>5.2f} {:>5.2f}'.format(ret,*(rotationMatrixToEulerAngles(R)*180/math.pi)))

Rvec,Tvec=None,None
def solve_pos():
    global Rvec,Tvec
    try:
        p2=get_c()
        pts3d=get_e()
        if Rvec is None:
            resPnP,Rvec,Tvec=cv2.solvePnP(pts3d,p2,K,distortion)
        else:
            resPnP,Rvec,Tvec=cv2.solvePnP(pts3d,p2,K,distortion,Rvec,Tvec,True)
            #resPnP,Rvec,Tvec,inliers=cv2.solvePnPRansac(pts3d,p2,K,distortion,Rvec,Tvec,True)
            

        if resPnP:
            print('len=',len(features_state),np.ravel(Tvec))
        return resPnP,Rvec,Tvec
    except:
        import pdb;pdb.set_trace()



import viewer
if __name__=='__main__':
    np.set_printoptions(formatter={'all':lambda x: '{:10.3f}'.format(x)})
    cap=cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FPS,60)
    
    view3d=viewer.plot3d()
    view3d.__next__()  



    cnt=0
    start_recover=False
    while 1:
        ret,img=cap.read()
        if ret:
            if cnt==0:
                init_of(img)
            else:
                calc_of(img)
            cnt+=1
            draw_ftrs(img)
            cv2.imshow('img',img)
            k=cv2.waitKey(1)
            if k==27:
                view3d.send(('stop',None))
                break
            if k==ord('a'):
                recover_pos()
                start_recover=True
            if start_recover:
                resPnP,Rvec,Tvec=solve_pos()
                if resPnP:
                    R,_=cv2.Rodrigues(Rvec)
                    view3d.send(('camera',(R,Tvec.ravel())))
                    pts3d=get_e()
                    view3d.send(('pts3d',pts3d))

        else:
            print('Error no image')
            break

 



