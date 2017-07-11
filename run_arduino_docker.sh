#!/bin/bash
xhost + local:

WORK_DIR=`python -c "import os;print(os.path.abspath(os.path.dirname('./')))"`

echo "----------------"
docker run -it \
-v $WORK_DIR:/home/docker/localization_tests \
-v /tmp/.X11-unix:/tmp/.X11-unix \
-e DISPLAY=$DISPLAY \
-e USERNAME=docker \
-e USER=docker \
-e HOME=/home/docker \
-u $UID \
--net=host \
--privileged \
arduino "/bin/bash"

#--net host \
#make posix_sitl_default jmavsim

