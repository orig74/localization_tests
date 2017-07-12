#!/bin/bash
tmux new-session -d -s testing
tmux send-keys "vim gy86.py" ENTER
tmux send-keys ":tabnew optitrack.py" ENTER
tmux split-window -h
tmux split-window -v
tmux send-keys "python localization.py --video_type=live_rec_gy86 --video data/manuvers_optitrack/test2 --pnp 2 --ftrang=190 --repres axisang --zest --rest"
tmux new-window -n recording
tmux split-window -h
tmux send-keys "python optitrack.py d" 
tmux split-window -v
tmux send-keys "./upload.sh" 
tmux split-window -v
tmux send-keys "jupyter notebook" 
tmux split-window -v
tmux select-pane -t 0
tmux send-keys "echo 'python gy86.py --prefix data/manuvers_optitrack/test%s --video 1 --dev 1 --rec --opti_track=3'"


#tmux select-window -t 0
tmux att

