# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
import serial,time,struct,math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

lmap = lambda func, *iterable: list(map(func, *iterable))

def reader():
    ser = serial.Serial('/dev/ttyACM0',115200)
    while 1:
        while 1:
            if ser.read()==b'\xa5':
                if ser.read()==b'\xa5':
                    break
        #synced
        ret={}
        raw_data=ser.read(26)
        chksum=struct.unpack('=H',ser.read(2))[0]
       
        calc_chksum=sum(struct.unpack('H'*13,raw_data))%2**16 
        if chksum!=calc_chksum:
            print('Error, bad checksum',chksum,calc_chksum)
            continue

        data=struct.unpack('='+'h'*9+'fi',raw_data)
        #print(data)
        ret['a/g']=np.array(lmap(float,data[:6]))
        ret['mag']=np.array(lmap(float,data[6:9]))
        ret['alt']=data[9]
        ret['t_stemp_ms']=data[10]
        yield ret
        

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

    while True:
        cnt+=1
        gy_data=yield
        history=history[-mem_len:]
        history.append(gy_data) 

        if cnt%10!=0:
            continue        

        for hdl in hdl_list:
            hdl[0].remove()
        hdl_list=[]
        
        acc_gyro=np.array(lmap(lambda x:x['a/g'],history))
        mag=np.array(lmap(lambda x:x['mag'],history))
        alt=np.array(lmap(lambda x:x['alt'],history))
        if alt_ref is None:
            alt_ref=alt[0]

        hdl_list.append(ax1.plot(acc_gyro[:,0],'-b',alpha=0.5)) 
        hdl_list.append(ax1.plot(acc_gyro[:,1],'-g',alpha=0.5)) 
        hdl_list.append(ax1.plot(acc_gyro[:,2],'-r',alpha=0.5)) 
                
        hdl_list.append(ax2.plot(acc_gyro[:,3],'-b',alpha=0.5)) 
        hdl_list.append(ax2.plot(acc_gyro[:,4],'-g',alpha=0.5)) 
        hdl_list.append(ax2.plot(acc_gyro[:,5],'-r',alpha=0.5)) 

        hdl_list.append(ax3.plot(mag[:,0],'-b',alpha=0.5)) 
        hdl_list.append(ax3.plot(mag[:,1],'-g',alpha=0.5)) 
        hdl_list.append(ax3.plot(mag[:,2],'-r',alpha=0.5)) 
        
        hdl_list.append(ax4.plot(alt-alt_ref,'-r',alpha=0.5)) 
        fig.canvas.draw()
        plt.waitforbuttonpress(timeout=0.001)
                


if __name__=="__main__":
    rd=reader()
    plot=ploter()
    plot.__next__()
    while 1:
        data=rd.__next__()
        #print(data)
        if data is not None:
            plot.send(data)
        else:
            print('Error data is None')
            time.sleep(0.01)
