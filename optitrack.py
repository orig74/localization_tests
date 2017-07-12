# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
#run those two lines
#sudo route del default gw 192.168.0.1
#sudo route add -net 224.0.0.0 netmask 240.0.0.0 enp1s0
import optirx as rx
import select
import sys

version = (3, 0, 0, 0)  # NatNet version to use
#dsock = rx.mkdatasock(\
dsock = rx.mkdatasock(  ip_address='0.0.0.0',
                        multicast_address='239.255.42.99',
                        port=1511)

def optitrack_loop(rigid_body_id):
    global version
    while 1:
        data=None
        while len(select.select([dsock],[],[],0)[0])>0:    
            #print('got pack') 
            data = dsock.recv(rx.MAX_PACKETSIZE)
        if data is not None:
            packet = rx.unpack(data, version=version)
            for pack in packet[3]:
                if pack.id == rigid_body_id:
                    yield pack
        else:
            yield

if __name__ == '__main__':
    if sys.argv[1]=='d':
        last_pos=None
        while True:
            data = dsock.recv(rx.MAX_PACKETSIZE)
            packet = rx.unpack(data, version=version)
            if type(packet) is rx.SenderData:
                version = packet.natnet_version
            #try manually identify the relevant rigid body
            #the list of rigid bodies is in packet 3
            pos = ' '.join(['%d: %.2f'%(pk.id,pk.position[1]) for pk in packet[3]])
            if pos != last_pos:
                print(pos)
                last_pos=pos

            #import pdb;pdb.set_trace()
    else:
        l=optitrack_loop(int(sys.argv[1]))
        while 1:
            pack = l.__next__()
            if pack:
                print('{:.2f} {:.2f} {:.2f}'.format(*pack.position))



