# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
import pandas as pd
import numpy as np
from numpy import matrix as mat
import cv2
import math
import time
import traceback,sys
import pickle,os
import utils
import argparse
import camerasim

parser = argparse.ArgumentParser()
parser.add_argument("--video", type=str, help=\
        "test video base name (e.g. without .avi)\n"
        "for example: data/manuvers_UE4/manuever2")
parser.add_argument("--video_type", default='sim'  ,type=str , help="ue4 , sim , live_rec_gy86")

#for know no live camera 
parser.add_argument("--dev",default=-1, type=int, help="web camera device number")

parser.add_argument("--pnp",default=1, type=int, help="type of pnp method 1-opencv 2-me")
parser.add_argument("--zest", help="use alt estimation",action="store_true")
parser.add_argument("--rest", help="use rotation estimation",action="store_true")
parser.add_argument("--wait", help="wait for space",action="store_true")
parser.add_argument("--ftrang", help="frame trangulation number default -1",type=int, default=-1)
args = parser.parse_args()


from mypnp import myPnP 

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

def init_of_sim(img,cap):
    global features_state
    features_state=pd.DataFrame(index=range(cap.last_points.shape[0]),columns=ftr_columns)
    features_state.loc[:,'sx':'sy']=cap.last_points
    features_state.loc[:,'cx':'cy']=cap.last_points

def calc_of_sim(img,cap):
    global features_state
    features_state.loc[:,'cx':'cy']=cap.last_points



def draw_ftrs(img):
    ftr_pos=np.array(features_state.loc[:,'cx':'cy'],dtype='float32').reshape(-1,2)
    indices=features_state.index.tolist()
    for i,ftr in zip(indices,ftr_pos):
         a,b = ftr
         cv2.circle(img,(a,b),2,colors[i].tolist(),-1)


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
   
def triangulate(R,T,estimated_alt=1.0):
    global features_state
    p1=get_s()
    p2=get_c()
    
    print('trangulating..')
    T=T.ravel()

    trans_T=(-mat(R).T*mat(T).T).A1
    trans=np.array([trans_T[0]/trans_T[2], trans_T[1]/trans_T[2], 1.0]) * estimated_alt 
    trans=(mat(R)*mat(trans).T).A1

    
    p1=cv2.undistortPoints(p1.reshape(-1,1,2),K,distortion).reshape(-1,2)
    p2=cv2.undistortPoints(p2.reshape(-1,1,2),K,distortion).reshape(-1,2)

    Proj1=np.hstack((np.eye(3),np.zeros((3,1))))
    Proj2=np.hstack((R,-trans.reshape((3,1))))

    pts3d_trang=cv2.triangulatePoints(Proj2,Proj1,p2.T,p1.T)
    pts3d_trang=pts3d_trang/pts3d_trang[3,:]
    pts3d_trang=pts3d_trang.T[:,:3]
    #import pdb;pdb.set_trace()
    ###sanity check
    if 0:
        R_vec,_=cv2.Rodrigues(R)
        ppts2d,jac=cv2.projectPoints(pts3d,R_vec,Trans,K,distortion)
        ppts2d=ppts2d.reshape(-1,2)
        ret=(ppts2d-get_c()).flatten()
        import pdb;pdb.set_trace()

    ###

    features_state.loc[:,'ex':'ez']=pts3d_trang.reshape(-1,3)
 
    #print('{:3} {:>5.2f} {:>5.2f} {:>5.2f}'.format(ret,*(rotationMatrixToEulerAngles(R)*180/math.pi)))

Rvec,Tvec=None,None

def solve_pos(estimate):
    global Rvec,Tvec
    try:
        p2=get_c()
        #import pdb;pdb.set_trace()

        ### add noise

        #p2=p2+np.random.normal(0,5,p2.shape)
        ###


        pts3d=get_e()
        if Rvec is None:
            Rvec=np.array([[   0.0],[   0.0],[   0.0]])
            Tvec=-np.array([[   0.0],[   0.0],[   7.0]]) # strting at positive alt Tvec=-Camera_pos

        if args.pnp==1:
            resPnP,Rvec,Tvec=cv2.solvePnP(pts3d,p2,K,distortion,Rvec,Tvec,True)
        if args.pnp==2:
            resPnP,Rvec,Tvec=myPnP(pts3d,p2,K,distortion,Rvec,Tvec,estimate)
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
    sensor_estimate=None
    np.set_printoptions(formatter={'all':lambda x: '{:10.3f}'.format(x)})


    ######## camera options
    sony_cam_mat=np.array( [ 5.5061780702480894e+02, 0., 3.1950000000000000e+02, 0.,
               5.5061780702480894e+02, 2.3950000000000000e+02, 0., 0., 1. ]).reshape((3,3)) #sony
    sony_distortion=np.array([-1.4562697048176954e-01, 1.4717208761705844e-01, 0., 0.,-3.1843325596064148e-03])


    #live camera
    if args.dev>=0: 
        K=sony_cam_mat
        distortion = sony_distortion
        cap=cv2.VideoCapture(args.dev)
        cap.set(cv2.CAP_PROP_FPS,60)

    #syntetic sim
    elif args.video_type == 'sim': 
        K=np.array([160.0,0,160, 0,160.0,120.0,0,0,1]).reshape((3,3))
        cap=camerasim.Capture(K.flatten(),(240,320))
        def _ground_truth():
            while 1:
                try:
                    data=cap.last_position
                    r={}
                    r['posx'],r['posy'],r['posz']=data[:3]
                    r['roll'],r['pitch'],r['yaw']=data[3:]/np.pi*180
                    #r['roll']=r['roll']
                    #r['pitch']=-r['pitch']
                    yield r
                except StopIteration:
                    pass

        ground_truth=_ground_truth()
        distortion=np.zeros(5)
        cap.read()
        start_alt=ground_truth.__next__()['posz']

    #ue4 simulated video
    elif args.video_type == 'ue4':
        base_name=args.video
        K=np.array(eval(open(base_name+'.cam').read())).reshape((3,3)) 


        distortion=np.zeros(5)
        #cap=file_grabber('output_ue4.avi')
        cap=file_grabber(base_name+'.avi')
        if os.path.isfile(base_name+'.pkl'):
            def _ground_truth():
                fd=open(base_name+'.pkl','rb')
                data=None
                while 1:
                    try:
                        data= pickle.load(fd)
                        if data: 
                            data['posx'],data['posy']=data['posy'],-data['posx']
                            data['pitch'],data['roll']=data['roll'],data['pitch']-90
                            data['yaw'],data['pitch']=data['pitch'],data['yaw']
                    except StopIteration:
                        pass
                    yield data

            ground_truth=_ground_truth()
                
        #skip a few frames 
        for _ in range(50):
            cap.read()
            if ground_truth:
                gt_pos_data=ground_truth.__next__()
                if gt_pos_data:
                    start_alt=gt_pos_data['posz']

    elif args.video_type == 'live_rec_gy86':
        from gy86 import VidSyncReader,WinFilter 
        K=sony_cam_mat*np.diag([0.5,0.5,1]) #resolution 320x240 check!!
        distortion = sony_distortion
        base_name=args.video
        cap=VidSyncReader(base_name)
        sensor_estimate=True


    else:
        print('Error unknown args.video_type: ',args.video_type)
        sys.exit(-1)
        
    view3d=viewer.plot3d()
    view3d.__next__()  

    cnt=0
    start_recover=False
    alt_tresh=-1
    last_alt=0
    while 1:
        k=cv2.waitKey(0 if args.wait else 1)
        if args.wait:
            print('fnum=',cnt)
        if k!=-1:
            #print('k=',k%256)
            k=k%256
        if k==27:
            #if start_recover:
            #    view3d.send(('stop',None))
            break
        if k==ord('b'):
            import pdb;pdb.set_trace()
        ret,img=cap.read()
        if type(ret)==dict and 'alt' in ret and not start_recover:
            if cnt==10: #wait afew frames then mark the start altitdude
                start_alt = ret['alt']
                print('start_alt=',ret['alt'])

        if ground_truth:
            try:
                gt_pos_data=ground_truth.__next__()
                if gt_pos_data['posz']<=last_alt: #not climbing anymore
                    alt_tresh=gt_pos_data['posz']-start_alt
                last_alt=gt_pos_data['posz']

                Tvec_gt=np.array([gt_pos_data['posx'],gt_pos_data['posy'],gt_pos_data['posz']])
                eu_vec=np.array([gt_pos_data['roll'],gt_pos_data['pitch'],gt_pos_data['yaw']])
                #eu_vec=np.array([gt_pos_data['pitch'],gt_pos_data['roll'],gt_pos_data['yaw']])
                R_gt = utils.eulerAnglesToRotationMatrix(eu_vec/180.0*np.pi) 
                
                view3d.send(('camera_gt',(time.time(),R_gt,Tvec_gt)))
            except EOFError:
                pass
            except StopIteration:
                pass
            #print('gt=',gt_pos_data)
        if ret:
            if cnt==0:
                if args.video_type == 'sim':
                    init_of_sim(img,cap)
                else:
                    init_of(img)
            else:
                if args.video_type == 'sim':
                    calc_of_sim(img,cap)
                else:
                    calc_of(img)
            cnt+=1
            draw_ftrs(img)
            cv2.imshow('img',img)
            if k==ord('a') or cnt==args.ftrang:
                alt_tresh=1.0
            if k==ord('g'):
                args.wait=False
            if alt_tresh>0 and not start_recover:
                _,R,T=recover_pos()
                if sensor_estimate:
                    relative_rot=cv2.Rodrigues(ret['rot'])[0] #check
                triangulate(R,T,alt_tresh)
                start_recover=True
            if start_recover:
                #ret,R,T=recover_pos(clean=False)
                est_dict={}
                if ground_truth and args.zest:
                    est_dict['alt']=(Tvec_gt[2]-start_alt)
                if ground_truth and args.rest:
                    R_vec_gt,_=cv2.Rodrigues(R_gt)
                    est_dict['rvec']=R_vec_gt.flatten()
                if sensor_estimate:
                    rmat,_=cv2.Rodrigues(ret['rot'])
                    rmat = np.dot(relative_rot,rmat.T) #check
                    R_vec,_=cv2.Rodrigues(rmat)
                    est_alt=ret['alt']-start_alt
                    est_dict['rvec']=R_vec.flatten()
                    est_dict['alt']=est_alt
                    view3d.send(('camera_gt',(time.time(),rmat,(0,0,est_alt))))
                 
                resPnP,Rvec,Tvec=solve_pos(est_dict)
                #import pdb;pdb.set_trace()
                #ffff
                if resPnP:
                    Rest,_=cv2.Rodrigues(Rvec)
                    cam_pos=-mat(Rest).T*mat(Tvec).T
                    view3d.send(('camera',(time.time(),Rest,cam_pos.A1)))
                    pts3d=get_e()
                    view3d.send(('pts3d',pts3d))
                else:
                    print('Error failed pnp')

        #else:
        #    print('Error no image')
            #break

 



if __name__=='__main__':
    if 0:
        import cProfile
        cProfile.run('main()','mainstats')
        import pstats
        p = pstats.Stats('mainstats')
        p.strip_dirs().sort_stats('cumulative').print_stats(100)
    else:
        main()

