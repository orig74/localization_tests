# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4

#to run recording include optitrack:

#first find the rigid body number by running optitrack.py in the example below is 3
#python gy86.py --prefix data/manuvers_optitrack/test%s --video 1 --dev 1 --rec --opti_track=3

import serial,time,struct,math
import numpy as np
import matplotlib.pyplot as plt
import sys
from mpl_toolkits.mplot3d import Axes3D
import argparse


if __name__=="__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--video",default=1, type=str, help="test video name")
    parser.add_argument("--opti_track",default=-1, type=int, help="optitrack rigid body id")
    parser.add_argument("--dev",default=0, type=int, help="v4l2 dev number (camera number)")
    parser.add_argument("--prefix",default='data/manuvers_raw/mov%s', help="video prefix path")
    parser.add_argument("--rec", help="record scenario",action="store_true",default=False)
    parser.add_argument("--sensor_only", help="test sensor",action="store_true",default=False)
    args = parser.parse_args()

    prefix = args.prefix%args.video

lmap = lambda func, *iterable: list(map(func, *iterable))

def reader():
    ser = serial.Serial('/dev/ttyUSB0',115200)
    ser.flush()
    while ser.inWaiting():
        ser.read()#flushing
    while 1:
        while ser.inWaiting()<2:
            yield None
        while 1:
            if ser.read()==b'\xa5':
                if ser.read()==b'\xa5':
                    break
        #synced
        ret={}
        raw_data=ser.read(26)
        chksum=struct.unpack('=H',ser.read(2))[0]
       
        calc_chksum=sum(struct.unpack('H'*13,raw_data))%2**16 
        #if chksum!=calc_chksum:
        #    print('Error, bad checksum',chksum,calc_chksum)
        #    continue

        data=struct.unpack('='+'h'*9+'fi',raw_data)
        ret['a/g']=np.array(lmap(float,data[:6]))
        ret['mag']=np.array(lmap(float,data[6:9]))
        ret['alt']=data[9]
        ret['t_stemp_ms']=data[10]/1000.0
        #print('==== {:.3f}'.format(data[10]/1e6))
        yield ret
        
def file_reader(fname):
    import pickle
    fd = open(fname,'rb')
    try:
        while 1:
            yield pickle.load(fd)
    except EOFError:
        while 1:
            yield None
            time.sleep(0.01)

def norm(X):
    return X/np.sqrt((X**2).sum(axis=0))


class WinFilter():
    def __init__(self,size,remove_dc=False):
        self.arr=[]
        self.size=size
        self.remove_dc=remove_dc
        self.dc=0
    def __call__(self,data):
        if self.remove_dc and self.dc==0:
            self.dc=data
        self.arr.append(data)
        if len(self.arr) > self.size:
            self.arr.pop(0)
        return sum(self.arr)/len(self.arr)-self.dc

def extruct_rot_alt(data):
    import cv2
    acc_vec=data['a/g'][:3]
    mag_vec=data['mag']
    
    # calib vals from: rot_from_mag_acc.ipynb

    mag_max,mag_min=np.array([ 769.,  579.,  408.]),np.array([-617., -669., -619.])
    mag_vec-=mag_min
    mag_vec=2*mag_vec/(mag_max-mag_min)

    acc_max,acc_min=np.array([ 17252.4 , 16250. ,  14839.2]),np.array([-16271.6, -17088.,-18845.6])
    acc_vec-=acc_min
    acc_vec=2*acc_vec/(acc_max-acc_min)

    mag_vec=norm(mag_vec)
    acc_vec=norm(acc_vec)

    a=acc_vec
    aXm=np.cross(a,mag_vec)
    aX_aXm=np.cross(a,aXm)

    rod = cv2.Rodrigues(np.vstack([a,aXm,aX_aXm]))[0]
    return {'rot':rod,'alt':data['alt']}




def vid_sync_reader(prefix):
    import grabber,pickle
    rd=open(prefix+'.pkl','rb')
    cap=grabber.file_grabber(prefix+'.avi')
    last_sensor_data=None
    alt_filter=WinFilter(40,True)
    ag_filter=WinFilter(7)
    last_optitrack_alt=None
    while 1:
        try:
            data=pickle.load(rd)
            if data is not None:
                if 's_sync' in data: #sensor data
                    data['a/g']=ag_filter(data['a/g'])
                    last_sensor_data=extruct_rot_alt(data)
                    last_sensor_data['alt']=alt_filter(last_sensor_data['alt'])
                #    if last_optitrack_alt is not None:
                #        last_sensor_data['alt']=last_optitrack_alt
                if 'o_sync' in data and last_sensor_data: #optitrack
                    last_sensor_data['odata']=data['odata']
                    #last_optitrack_alt=data['odata'].position[1]
                if 'c_sync' in data: #camera_data
                    _,img=cap.read()
                    if last_sensor_data is None: #wait for first sensor data
                        continue
                    yield last_sensor_data,img
        except EOFError:
            while 1:
                yield False,None
                time.sleep(0.01)

     
class VidSyncReader():
    def __init__(self,prefix):
        self.gen=vid_sync_reader(prefix)
    def read(self):
        return self.gen.__next__()


def ploter():
    fig = plt.figure(figsize=(8,6))
    ax1 = fig.add_subplot(4, 1, 1)
    ax1.set_title('acc')
    ax2 = fig.add_subplot(4, 1, 2)
    ax2.set_title('gyro')
    ax3 = fig.add_subplot(4, 1, 3)
    ax3.set_title('mag')
    ax4 = fig.add_subplot(4, 1, 4)
    ax4.set_title('alt')
    ax4.set_ylim(-1,1)
    fig.canvas.draw()   # note that the first draw comes before setting data 
    #fig.canvas.mpl_connect('close_event', handle_close)
    #h1 = ax1.plot([0,1],[0,1],[0,1], lw=3)[0]
    #text = ax1.text(0.8,1.5, '')
    t_start = time.time()
    history=[]

    cnt=0
    mem_len=200
    hdl_list=[]
    alt_ref=None
    last_plot=time.time()
    while True:
        cnt+=1
        gy_data=yield
        history=history[-mem_len:]
        history.append(gy_data) 

        if time.time()-last_plot<0.2 and cnt%10!=0:
            continue        
        last_plot=time.time()

        for hdl in hdl_list:
            hdl[0].remove()
        hdl_list=[]
        
        acc_gyro=np.array(lmap(lambda x:x['a/g'],history))
        mag=np.array(lmap(lambda x:x['mag'],history))
        alt=np.array(lmap(lambda x:x['alt'],history))
        #ts=np.array(lmap(lambda x:x['t_stemp_ms']/1000.0,history))
        if 's_sync' in gy_data:
            ts=np.array(lmap(lambda x:x['s_sync']/1000.0,history))
        else:
            ts=np.array(lmap(lambda x:x['t_stemp_ms']/1000.0,history))
        if alt_ref is None:
            alt_ref=alt[0]

        hdl_list.append(ax1.plot(ts,acc_gyro[:,0],'-b',alpha=0.5)) 
        hdl_list.append(ax1.plot(ts,acc_gyro[:,1],'-g',alpha=0.5)) 
        hdl_list.append(ax1.plot(ts,acc_gyro[:,2],'-r',alpha=0.5)) 
        ax1.set_xlim(ts.min(),ts.max())        
        
        hdl_list.append(ax2.plot(ts,acc_gyro[:,3],'-b',alpha=0.5)) 
        hdl_list.append(ax2.plot(ts,acc_gyro[:,4],'-g',alpha=0.5)) 
        hdl_list.append(ax2.plot(ts,acc_gyro[:,5],'-r',alpha=0.5)) 
        ax2.set_xlim(ts.min(),ts.max())        

        hdl_list.append(ax3.plot(ts,mag[:,0],'-b',alpha=0.5)) 
        hdl_list.append(ax3.plot(ts,mag[:,1],'-g',alpha=0.5)) 
        hdl_list.append(ax3.plot(ts,mag[:,2],'-r',alpha=0.5)) 
        ax3.set_xlim(ts.min(),ts.max())        
        
        hdl_list.append(ax4.plot(ts,alt-alt_ref,'-r',alpha=0.5)) 
        ax4.set_xlim(ts.min(),ts.max())
        #if cnt<100:        
        fig.canvas.draw()
        plt.waitforbuttonpress(timeout=0.001)
                


if  __name__=="__main__":
    if args.sensor_only:
        rd=reader()
        #rd=file_reader(prefix+'.pkl')
        plot=ploter()
        plot.__next__()
        while 1:
            data=rd.__next__()
            #print(data)
            if data is not None:
                if 'a/g' in data:
                    print(data['a/g'][:3],data['mag'])
                    plot.send(data)
            else:
                #print('Error data is None')
                time.sleep(0.01)

    elif not args.rec:
        import grabber,cv2
        rd=file_reader(prefix+'.pkl')
        cap=grabber.file_grabber(prefix+'.avi')
        
        plot=ploter()
        plot.__next__()
        start = time.time()
        frame_cnt=0
        sensor_cnt=0
        while 1:
            data=rd.__next__()
            #print(data)
            if data is not None:
                if 's_sync' in data:
                    while data['s_sync']>time.time()-start:
                        time.sleep(0.001)
                    sensor_cnt+=1

                if 'a/g' in data:
                    plot.send(data)
                if 'c_sync' in data:
                    _,im=cap.read()
                    frame_cnt+=1
                    font = cv2.FONT_HERSHEY_SIMPLEX
                    cv2.putText(im,'%d'%frame_cnt,(100,100), font, 0.3,(255,255,255),1,cv2.LINE_AA)
                    cv2.putText(im,'%d'%sensor_cnt,(100,130), font, 0.3,(255,255,255),1,cv2.LINE_AA)
                    cv2.putText(im,'%.3f'%data['c_sync'],(100,160), font, 0.3,(255,255,255),1,cv2.LINE_AA)

                    cv2.imshow('cv',im)
                    key=cv2.waitKey(0)%256
                    if key==27:
                        break            
            else:
                time.sleep(0.01)

    elif args.rec:
        import subprocess
        import cv2
        import pickle
        pklfd=open(prefix+'.pkl','wb')
        cap=cv2.VideoCapture(args.dev)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,320);
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT,240);
        rd=reader()
        if args.opti_track>0:
            import optitrack 
            ot = optitrack.optitrack_loop(args.opti_track)
        else:
            ot = None
        #http://zulko.github.io/blog/2013/09/27/read-and-write-video-frames-in-python-using-ffmpeg/
        cmd='ffmpeg -y -f rawvideo -pix_fmt rgb24 -s 320x240 -r 30 '\
                +'-i - -an -vcodec libx264 -preset ultrafast -crf 0 {}'\
                .format(prefix+'.avi')
        pr=subprocess.Popen(cmd,shell=True,stdin=subprocess.PIPE)
        tstart=time.time()
        while 1:
            data=rd.__next__()
            if data is not None:
                data['s_sync']=time.time()-tstart
                pickle.dump(data,pklfd,-1)
            if cap.grab():
                _,im=cap.retrieve()
                pickle.dump({'c_sync':time.time()-tstart},pklfd,-1)
                pr.stdin.write(im.tostring())
            if ot:
                odata=ot.__next__()
                if odata:
                    pickle.dump({'o_sync':time.time()-tstart,'odata':odata},pklfd,-1)
                    
                #k=cv2.waitKey(1)
                #if (k%256)==27:
                #    break

   

