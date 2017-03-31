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
    ax1 = fig.add_subplot(1, 1, 1, projection='3d')
    fig.canvas.draw()   # note that the first draw comes before setting data 
    #fig.canvas.mpl_connect('close_event', handle_close)
    #h1 = ax1.plot([0,1],[0,1],[0,1], lw=3)[0]
    camera=None
    pts3d=None
    #text = ax1.text(0.8,1.5, '')
    ax1.set_ylim([-2,2])
    ax1.set_xlim([-2,2])
    ax1.set_zlim([2,-2])


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
            fig.canvas.draw()
        if cmd=='pts3d':
            if pts3d is not None:
                pts3d.remove()
            #import pdb;pdb.set_trace()
            pts3d=ax1.scatter(data[:,0],data[:,1],data[:,2])

        if cmd=='stop':
            break
        plt.waitforbuttonpress(timeout=0.001)
        i+=1


if __name__=="__main__":
    pl=plot3d()
    pl.__next__()
    for i in range(1000):
        T=[1,1,1]
        R=np.eye(3)
        pl.send(('camera',(R,T)))
    pl.send(('stop',None))
