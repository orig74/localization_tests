# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
import pandas as pd
import numpy as np
np.random.seed(1001)
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
parser.add_argument("--sim_spread", default=0.06  ,type=float , help="in sim distance spread between points")
parser.add_argument("--sim_npoints", default=3  ,type=int , help="n-number of points in sim nXn grid")



#for know no live camera 
parser.add_argument("--dev",default=-1, type=int, help="web camera device number")

parser.add_argument("--pnp",default=1, type=int, help="type of pnp method 1-opencv 2-me axisand 3-me euler")
parser.add_argument("--zest", help="use alt estimation",action="store_true")
parser.add_argument("--rest", help="use rotation estimation",action="store_true")

parser.add_argument("--skipcvshow", help="skip show opencv window",action="store_true")
parser.add_argument("--wait", help="wait for space",action="store_true")
parser.add_argument("--ftrang", help="frame trangulation number default -1",type=int, default=-1)
parser.add_argument("--override_alt", help="override messured alt",type=float, default=-1)
parser.add_argument("--skip", help="frames to skip in the beginning",type=int, default=50)

parser.add_argument("--point_noise",default=0, type=float, help="adding normal noise to points defult is 0")
parser.add_argument("--rvec_noise",default=0, type=float, help="adding normal noise to attitude sensors defult is 0")
parser.add_argument("--rotation_bounds",default='180,180,180', type=str, help="rotation bounds in the form of x,y,z")
parser.add_argument("--headless", help="running in headless mode",action="store_true")
parser.add_argument("--dumpfile", help="dumping output data file name",type=str , default='')

args = parser.parse_args()


import mypnp

N_FTRS=40

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
    E,mask=cv2.findEssentialMat(p1,p2,K,cv2.RANSAC,0.999,1.0)
    ret,R,T,mask=cv2.recoverPose(E,p1,p2,K,mask)
    #clean..

    if clean:
        print('cleanning db')
        features_state=features_state[mask.ravel()==255]
    return ret,R,T
   
def triangulate(R,T,start_alt,delta_alt):
    global features_state
    p1=get_s()
    p2=get_c()
    
    print('trangulating..')
    T=T.ravel()

    trans_T=(-mat(R).T*mat(T).T).A1
    trans=np.array([trans_T[0]/trans_T[2], trans_T[1]/trans_T[2], 1.0]) * delta_alt
    trans=(mat(R)*mat(trans).T).A1

    
    p1=cv2.undistortPoints(p1.reshape(-1,1,2),K,distortion).reshape(-1,2)
    p2=cv2.undistortPoints(p2.reshape(-1,1,2),K,distortion).reshape(-1,2)

    Proj1=np.eye(4)
    Proj1[2,3]=-start_alt
    #Proj1=np.hstack([np.eye(3),np.array([[0 , 0, -start_alt]]).T])
    Proj2=np.eye(4)
    Proj2[:3,:3]=R
    Proj2[:3,3]=-trans
    #Proj2=np.hstack((R,-trans.reshape((3,1)))) 
    Proj2 = Proj2 @ Proj1
    pts3d_trang=cv2.triangulatePoints(Proj2[:3,:],Proj1[:3,:],p2.T,p1.T)
    pts3d_trang=pts3d_trang/pts3d_trang[3,:]
    pts3d_trang=pts3d_trang.T[:,:3]
    #import pdb;pdb.set_trace()
    
    ### add noise
    if args.point_noise>0:
        #point_noise = np.random.normal(0,args.point_noise,p2.shape)
        point_noise = np.random.normal(0,args.point_noise,pts3d_trang.shape)
        #import pdb;pdb.set_trace()
        pts3d_trang=pts3d_trang+point_noise
    ###

    #pts3d_trang=camerasim.generate_3d_points(args.sim_spread,args.sim_npoints)
    ###sanity check
    if 0:
        R_vec,_=cv2.Rodrigues(R)
        ppts2d,jac=cv2.projectPoints(pts3d_trang,R_vec,trans_T,K,distortion)
        ppts2d=ppts2d.reshape(-1,2)
        ret=(ppts2d-get_c()).flatten()
        #import pylab
        #pylab.figure()
        #pylab.plot(pts3d_trang[:,0],pts3d_trang[:,1],'+')
        #pylab.show()
        import pdb;pdb.set_trace()

    ###

    features_state.loc[:,'ex':'ez']=pts3d_trang.reshape(-1,3)
 
    #print('{:3} {:>5.2f} {:>5.2f} {:>5.2f}'.format(ret,*(rotationMatrixToEulerAngles(R)*180/math.pi)))




import viewer

from grabber import file_grabber

def main():
    global K,distortion
    ground_truth=None
    sensor_estimate=None
    rvec_noise = np.zeros(3)
    np.set_printoptions(formatter={'all':lambda x: '{:10.3f}'.format(x)})


    ######## camera options
    #640x280
    #sony_cam_mat=np.array( [ 5.5411829321732762e+02, 0., 3.1950000000000000e+02, 0.,
    #           5.5411829321732762e+02, 2.3950000000000000e+02, 0., 0., 1. ]).reshape((3,3)) #sony
    #sony_distortion=np.array([-1.5767246702411403e-01, 2.0073777106331972e-01, 0., 0.,
    #           -6.8129177074104305e-02 ])
    #320x240
    sony_cam_mat=np.array( [ 2.8750015980595219e+02, 0., 1.5950000000000000e+02, 0.,
               2.8750015980595219e+02, 1.1950000000000000e+02, 0., 0., 1. ] ).reshape((3,3)) #sony
    sony_distortion=np.array([-2.0038004466909196e-01, 5.1964906681280620e-01, 0., 0.,
               -8.6232173172164162e-01 ])



    #live camera
    if args.dev>=0: 
        K=sony_cam_mat
        distortion = sony_distortion
        cap=cv2.VideoCapture(args.dev)
        cap.set(cv2.CAP_PROP_FPS,60)

    #syntetic sim
    elif args.video_type == 'sim': 
        K=np.array([160.0,0,160, 0,160.0,120.0,0,0,1]).reshape((3,3))
        cap=camerasim.Capture(K.flatten(),(240,320),None,args.sim_spread,args.sim_npoints)
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
        for _ in range(args.skip):
            cap.read()
            if ground_truth:
                gt_pos_data=ground_truth.__next__()
                if gt_pos_data:
                    start_alt=gt_pos_data['posz']

    elif args.video_type == 'live_rec_gy86':
        from gy86 import VidSyncReader 
        K=sony_cam_mat
        distortion = sony_distortion
        base_name=args.video
        cap=VidSyncReader(base_name)
        for _ in range(args.skip):
            cap.read()
        sensor_estimate=True


    else:
        print('Error unknown args.video_type: ',args.video_type)
        sys.exit(-1)
    
    if not args.headless:
        view3d=viewer.plot3d()
        view3d.__next__()  
    if args.dumpfile:
        dumpfile = open(args.dumpfile,'wb')



    if args.pnp==1:
        pnpclass = mypnp.ContSolvePnPopenCV
        spnp=pnpclass(K,distortion)
    else:
        if args.pnp==2:
            pnpclass = mypnp.ContSolvePnPAxisAng
        if args.pnp==3:
            pnpclass = mypnp.ContSolvePnPEulerAng
        spnp=pnpclass(eval('['+args.rotation_bounds+']'),K,distortion)


    cnt=0
    start_recover=False
    delta_alt=-1
    last_alt=0
    filt_campos=None

    while 1:
        if args.headless:
            k=-1
        else:
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
        if ret is False and args.headless:
            if args.dumpfile:
                dumpfile.close()
            break
            
        if ground_truth:
            try:
                gt_pos_data=ground_truth.__next__()
                if gt_pos_data['posz']<=last_alt: #not climbing anymore
                    delta_alt=gt_pos_data['posz']-start_alt
                last_alt=gt_pos_data['posz']

                Tvec_gt=np.array([gt_pos_data['posx'],gt_pos_data['posy'],gt_pos_data['posz']])
                eu_vec=np.array([gt_pos_data['roll'],gt_pos_data['pitch'],gt_pos_data['yaw']])
                #eu_vec=np.array([gt_pos_data['pitch'],gt_pos_data['roll'],gt_pos_data['yaw']])
                R_gt = utils.eulerAnglesToRotationMatrix(eu_vec/180.0*np.pi) 
                
                payload = ('camera_gt',(time.time(),R_gt,Tvec_gt))
                if not args.headless:
                    view3d.send(payload)
                if args.dumpfile:
                    pickle.dump(payload, dumpfile, -1)

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
            if not args.skipcvshow:
                draw_ftrs(img)
                cv2.imshow('img',img)
            if k==ord('a') or cnt==args.ftrang:
                delta_alt=1.0
            if k==ord('g'):
                args.wait=False
            if delta_alt>0 and not start_recover:
                _,R,T=recover_pos()
                if sensor_estimate:
                    delta_alt = ret['alt']-start_alt
                    relative_rot=cv2.Rodrigues(ret['rot'])[0] #check
                triangulate(R,T,start_alt,delta_alt)
                if args.dumpfile:
                    pickle.dump(('pts3d',(time.time(),get_e())), dumpfile, -1)
                start_recover=True
            if start_recover:
                #ret,R,T=recover_pos(clean=False)
                est_dict={}
                if ground_truth and args.zest:
                    est_dict['alt']=Tvec_gt[2]
                    #est_dict['alt']+=np.random.normal(0,0.05)
                if ground_truth and args.rest:
                    R_vec_gt,_=cv2.Rodrigues(R_gt)
                    est_dict['rvec']=R_vec_gt.flatten()
                    #est_dict['rvec']+=np.random.normal(0,np.radians(1),3)
                if sensor_estimate:
                    rmat,_=cv2.Rodrigues(ret['rot'])
                    rmat = np.dot(relative_rot,rmat.T) #check
                    R_vec,_=cv2.Rodrigues(rmat)
                    est_alt=ret['alt']-start_alt
                    if args.rest:
                        est_dict['rvec']=R_vec.flatten()
                    if args.zest: 
                        est_dict['alt']=est_alt
                   
                    payload = ('camera_gt',(time.time(),rmat,(0,0,est_alt)))
                    if not args.headless:
                        view3d.send(payload)
                    if args.dumpfile:
                        pickle.dump(payload,dumpfile,-1)
 
                if 'rvec' in est_dict and args.rvec_noise>0: 
                    rvec_noise = 0.999*rvec_noise+0.001*np.random.normal(0,args.rvec_noise,3)
                    est_dict['rvec'] += rvec_noise    
                if args.pnp>1:
                    resPnP,Rvec,Tvec=spnp.solve(get_e(),get_c(),est_dict)
                else:
                    resPnP,Rvec,Tvec=spnp.solve(get_e(),get_c())

                #ffff
                if resPnP:
                    Rest,_=cv2.Rodrigues(Rvec)
                    cam_pos=-mat(Rest).T*mat(Tvec).T
                    if filt_campos is None:
                        filt_campos=cam_pos
                    w=0.0
                    filt_campos = filt_campos*w+cam_pos*(1-w)
                    tsync=time.time()
                    payload = ('camera',(tsync,Rest,filt_campos.A1))
                    if not args.headless:
                        view3d.send(payload)
                        pts3d=get_e()
                        view3d.send(('pts3d',pts3d)) 
                    if args.dumpfile:
                        if type(ret) is dict and 'odata' in ret:
                            pickle.dump(('odata',(tsync,ret['odata'])),dumpfile,-1)
                        pickle.dump(payload,dumpfile,-1)
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

