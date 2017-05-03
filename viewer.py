# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
import time
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import sys
import numpy as np
import utils

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
    fig = plt.figure(figsize=(8,6))
    ax1 = fig.add_subplot(2, 2, 1, projection='3d')
    ax1.set_title('3D')
    ax2 = fig.add_subplot(2, 2, 2)
    ax2.set_title('XYZ')
    ax3 = fig.add_subplot(2, 2, 4)
    ax3.set_title('2D XY plot')
    ax4 = fig.add_subplot(2, 2, 3)
    ax4.set_title('Angles')
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


    t_start = time.time()
    camera_t_data=[]
    camera_r_data=[]    
    camera_t_data_gt=[]
    camera_r_data_gt=[]
    camera_ttags=[]
    camera_gt_ttags=[]

    cam_pos_h=None

    i=0
    cam_cnt=0
    mem_len=1000
    while True:
        cmd,data=yield
        if cmd=='camera':
            ttag,R,T=data
            camera_t_data.append(T)
            camera_t_data=camera_t_data[-mem_len:]
            camera_r_data.append(utils.rotationMatrixToEulerAngles(R)*180.0/np.pi)
            camera_r_data=camera_r_data[-mem_len:]
            camera_ttags.append(ttag-t_start)
            camera_ttags=camera_ttags[-mem_len:]
            cam_cnt+=1
            if cam_cnt%10==0:
                if camera is not None:
                    for h in camera:
                        h.remove()
                camera=plot_camera(ax1,R,-T)
                camera_t_vec=np.vstack(camera_t_data)
                camera_r_vec=np.vstack(camera_r_data)

                if cam_pos_h is not None:
                    for hdl in cam_pos_h:
                        hdl[0].remove()
                cam_pos_h = [ ax2.plot(camera_ttags,camera_t_vec[:,i],c) for i,c in enumerate('rgb') ]
                cam_pos_h.append(ax3.plot(camera_t_vec[:,0],camera_t_vec[:,1],'-b',alpha=0.5)) 
                
                cam_pos_h += [ ax4.plot(camera_ttags,camera_r_vec[:,i],c) for i,c in enumerate('rgb') ]
               
                if camera_t_data_gt:
                    camera_t_vec_gt=np.vstack(camera_t_data_gt)
                    camera_r_vec_gt=np.vstack(camera_r_data_gt)
                    camera_t_vec_gt-=camera_t_vec_gt[0] #start at (0,0)
                    cam_pos_h.append(ax3.plot(camera_t_vec_gt[:,0],camera_t_vec_gt[:,1],'-r',alpha=0.5)) 
                    cam_pos_h += [ ax2.plot(camera_gt_ttags,camera_t_vec_gt[:,i],c,alpha=0.5) for i,c in enumerate('rgb') ]
                    cam_pos_h += [ ax4.plot(camera_gt_ttags,camera_r_vec_gt[:,i],c,alpha=0.5) for i,c in enumerate('rgb') ]

                fig.canvas.draw()
                

        if cmd=='camera_gt':
            ttag,R,T=data
            camera_t_data_gt.append(T)
            camera_t_data_gt=camera_t_data_gt[-mem_len:]
            camera_r_data_gt.append(utils.rotationMatrixToEulerAngles(R)*180.0/np.pi)
            camera_r_data_gt=camera_r_data_gt[-mem_len:]
            camera_gt_ttags.append(ttag-t_start)
            camera_gt_ttags=camera_gt_ttags[-mem_len:]


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
