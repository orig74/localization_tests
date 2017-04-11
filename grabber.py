# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
import subprocess
import numpy as np
import time
import os,re

class file_grabber():
    def __init__(self,fname):
        #detect image size
        aaa=os.popen('ffprobe {} 2>&1'.format(fname)).read()
        self.size=tuple(map(int,re.search('[0-9]{3,4}x[0-9]{3,4}',aaa).group(0).split('x')))

        cmd='ffmpeg -i {} -f rawvideo -pix_fmt rgb24 -'.format(fname)
        self.pr=subprocess.Popen(cmd,shell=True,stdout=subprocess.PIPE,stdin=subprocess.PIPE)
        #self.size=(640,480)
    def read(self):
        s=self.pr.stdout.read(self.size[0]*self.size[1]*3)
        return True,np.frombuffer(s,dtype='uint8').reshape(self.size[1],self.size[0],3).copy()
    def __del__(self):
        print('kill ffmpeg...')
        pr.stdin.write('q\n')
        time.sleep(1)
        self.pr.kill()

if __name__=='__main__':
    import cv2
    cap=file_grabber('output.mkv')
    while 1:
        _,im=cap.read()
        cv2.imshow('im',im)
        k=cv2.waitKey(0)
        if k==27:
            break
        
