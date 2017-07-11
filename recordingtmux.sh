#!/bin/bash
tmux new-session -d -s dronelab
tmux send-keys "python drone_main_mlink.py"
tmux split-window -h
tmux send-keys "cd ../../dockers/sitl_image_px4/ && ./run_instance.sh $PX4_PATH 0" ENTER
tmux send-keys "cd /PX4/Firmware" ENTER
tmux send-keys "make posix_sitl_default jmavsim" 
tmux split-window -v
tmux send-keys "export DEMO_DIR=${PWD}" ENTER
tmux send-keys "cd ${PROJECT_PATH}/Plugins/UE4PyServer/Source/PyServer" ENTER
tmux send-keys "python config.py --entry_point=unreal_proxy --entry_path=\$DEMO_DIR" ENTER
tmux send-keys "./run.sh" 
#tmux select-window -t 0
tmux att

