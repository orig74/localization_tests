# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
import socket,os
import pickle
import zmq
import sys
import asyncio
import numpy as np
import argparse,imp
import cv2,struct

sys.path.append(os.environ['UNREAL_PROXY_PATH'])
import config

parser = argparse.ArgumentParser()
parser.add_argument("--config_path", help="config.py file path")
args = parser.parse_args()

config=imp.load_module("config",*imp.find_module('config',[args.config_path]))




context = zmq.Context()
position_struct={}
socket_pub = context.socket(zmq.PUB)
addr="tcp://%s:%d" % (config.zmq_pub_drone_fdm)
print("publishing to: ",addr)
socket_pub.bind(addr)


zmq_sub = context.socket(zmq.SUB)
addr="tcp://%s:%d" % (config.zmq_pub_unreal_proxy)
zmq_sub.connect(addr)
topic=(config.topic_unreal_drone_rgb_camera.decode()%0).encode()#had to do encode decode for python version earlier then 3.5
print('topic is',topic)
zmq_sub.setsockopt(zmq.SUBSCRIBE,topic)

@asyncio.coroutine
def reader():
    ########## init
    ps=position_struct
    ps['posx'],ps['posy'],ps['posz'],ps['roll'],ps['pitch'],ps['yaw']=[0]*6
    ps['pitch']=90
    ps['posz']=2

    for _ in range(100):
        yield from asyncio.sleep(1/30.0) #30Hz
    ######### climb
    ps['posx']=-5.0

    for _ in range(200):
        ps['posz']+=.01
        yield from asyncio.sleep(1/30.0) #30Hz
    for _ in range(100):
        yield from asyncio.sleep(1/30.0) #30Hz

    for rot_pitch,rot_roll in [(0,0),(200,20)]: 
        for ind in range(400):
            if ind==0: 
                step_x=-0.01
                step_y=0
            if ind==100:
                step_x=0
                step_y=0.01
            if ind==200:
                step_x=0.01
                step_y=0
            if ind==300:
                step_x=0
                step_y=-0.01
            ps['posx']+=step_x
            ps['posy']+=step_y
            
            if rot_pitch>0:
                pitch_sign=1 if (ind%rot_pitch)<(rot_pitch//2) else -1
                ps['pitch']+=pitch_sign*0.1
            if rot_roll>0:
                roll_sign=1 if (ind%rot_roll)<(rot_roll//2) else -1
                ps['roll']+=roll_sign*0.1


            yield from asyncio.sleep(1/30.0) #30Hz

    while 1:
        yield from asyncio.sleep(1/30.0) #30Hz


@asyncio.coroutine
def printer():
    while 1:
        print('-->',position_struct)
        yield from asyncio.sleep(1)

@asyncio.coroutine
def pub_position_struct():
    while 1:
        if len(position_struct)>0:
            socket_pub.send_multipart([config.topic_sitl_position_report,pickle.dumps(position_struct,-1)])
        yield from asyncio.sleep(1/30.0) #30Hz

@asyncio.coroutine
def video_recorder():
    outsize=[240,320][::-1]
    vr=cv2.VideoWriter('manuever.avi',cv2.VideoWriter_fourcc('M','J','P','G'),30,tuple(outsize))
    fd=open('data.pkl','wb')
    while 1:
        if len(zmq.select([zmq_sub],[],[],0)[0])>0:
            topic, shape, data = zmq_sub.recv_multipart()
            if topic==b'rgb_camera_0':
                shape=struct.unpack('lll',shape)
                try:
                    img=np.fromstring(data,'uint8').reshape(shape)
                except:
                    import pdb;pdb.set_trace()
                img=cv2.resize(img,tuple(outsize))
                vr.write(img)
                pickle.dump(position_struct,fd)
            #print(topic,',',img.shape)
        yield 




if __name__=="__main__":
    loop = asyncio.get_event_loop()
    tasks=[printer(),pub_position_struct(),reader(),video_recorder()]
    loop.run_until_complete(asyncio.wait(tasks))
    loop.close()
