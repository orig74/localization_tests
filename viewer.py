#vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
import time
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import sys
import numpy as np



def plot_camera(ax,R,T):
    ret=[]
    r=np.mat(R)*np.mat([1,0,0]).T
    ret.append(ax.quiver(T[0],T[1],T[2],r[0],r[1],r[2],color='red',pivot='tail'))
    
    r=np.mat(R)*np.mat([0,1,0]).T
    ret.append(ax.quiver(T[0],T[1],T[2],r[0],r[1],r[2],color='green',pivot='tail'))
    
    r=np.mat(R)*np.mat([0,0,1]).T
    ret.append(ax.quiver(T[0],T[1],T[2],r[0],r[1],r[2],color='blue',pivot='tail'))
    return ret

def plot3d():
    fig = plt.figure()
    ax1 = fig.add_subplot(1, 2, 1, projection='3d')
    ax2 = fig.add_subplot(2, 2, 2)
    ax3 = fig.add_subplot(2, 2, 4)
    fig.canvas.draw()   # note that the first draw comes before setting data 
    #fig.canvas.mpl_connect('close_event', handle_close)
    #h1 = ax1.plot([0,1],[0,1],[0,1], lw=3)[0]
    camera=None
    pts3d=None
    #text = ax1.text(0.8,1.5, '')
    ax1.set_ylim([-2,2])
    ax1.set_xlim([-2,2])
    ax1.set_zlim([2,-2])
    ax3.set_ylim([-2,2])
    ax3.set_xlim([-2,2])


    camera_t_data=[[0,0,0]]    
    camera_t_data_gt=[]    
    cam_pos_h=None

    t_start = time.time()
    i=0
    while True:
        cmd,data=yield
        if cmd=='camera':
            if camera is not None:
                for h in camera:
                    h.remove()
            R,T=data
            camera=plot_camera(ax1,R,-T)
            camera_t_data.append(T)
            camera_t_data=camera_t_data[-1000:]
            camera_t_vec=np.vstack(camera_t_data)

            if cam_pos_h is not None:
                for hdl in cam_pos_h:
                    hdl[0].remove()
            cam_pos_h = [ ax2.plot(range(len(camera_t_data)),camera_t_vec[:,i],c) for i,c in enumerate('rgb') ]
            cam_pos_h.append(ax3.plot(camera_t_vec[:,0],camera_t_vec[:,1],'-b')) 
           
            if camera_t_data_gt:
                camera_t_vec_gt=np.vstack(camera_t_data_gt)
                camera_t_vec_gt-=camera_t_vec_gt[0] #start at (0,0)
                cam_pos_h.append(ax3.plot(camera_t_vec_gt[:,0],camera_t_vec_gt[:,1],'-r')) 

            fig.canvas.draw()

        if cmd=='camera_gt':
            R,T=data
            camera_t_data_gt.append(T)
            camera_t_data_gt=camera_t_data_gt[-1000:]


        if cmd=='pts3d':
            if pts3d is not None:
                pts3d.remove()
            #import pdb;pdb.set_trace()
            pts3d=ax1.scatter(data[:,0],data[:,1],data[:,2],c='b')

        if cmd=='stop':
            break
        plt.waitforbuttonpress(timeout=0.001)
        i+=1


if __name__=="__main__":
    pl=plot3d()
    pl.__next__()
    for i in range(1000):
        T=np.array([1,1,1])
        R=np.eye(3)
        pl.send(('camera',(R,T)))
    pl.send(('stop',None))
