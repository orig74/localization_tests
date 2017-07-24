##experiment data creation

##running pure sim:
### problem presentation
#### no problem 
python localization.py --video_type=sim --pnp 1 --point_noise=0.0 --sim_npoints=4 --sim_spread=0.08 --headless --dumpfile=out/sim_pnp_1_nonoise.pkl

#### local minima error
python localization.py --video_type=sim --pnp 1 --point_noise=0.01 --sim_npoints=4 --sim_spread=0.08 --headless --dumpfile=out/sim_pnp_1_noise.pkl

#### minimization with bounds
### applying constraints to solutions
python localization.py --video_type=sim --pnp 2 --point_noise=0.01 --sim_npoints=4 \
	--sim_spread=0.08 --rotation_bounds="3.0,3.0,3.0" --zest --rest --headless --dumpfile=out/sim_pnp_2_noise_constrains.pkl

python localization.py --video_type=sim --pnp 3 --point_noise=0.01 --sim_npoints=4 \
	--sim_spread=0.08 --rotation_bounds="3.0,3.0,3.0" --zest --rest --headless --dumpfile=out/sim_pnp_3_noise_constrains.pkl



### runnign with solvePnP regular
python localization.py --video_type=live_rec_gy86 --video data/manuvers_optitrack/test12\
			 --pnp 1 --ftrang=140 --headless --dumpfile=out/optilive12_pnp_1.pkl


### running with modified solvePnP without constraints
python localization.py --video_type=live_rec_gy86 --video data/manuvers_optitrack/test12\
			--pnp 2 --ftrang=140  --headless --dumpfile=out/optilive12_pnp_2.pkl

### running on live data without zest (since its only relative alt and no accurate
python localization.py --video_type=live_rec_gy86 --video data/manuvers_optitrack/test12\
		 	--pnp 3 --ftrang=140 --rest --rotation_bounds="3.0,3.0,3.0"  --headless --dumpfile=out/optilive12_pnp_3_constrains.pkl

python localization.py --video_type=live_rec_gy86 --video data/manuvers_optitrack/test12\
 			--pnp 2 --ftrang=140 --rest --rotation_bounds="3.0,3.0,3.0"   --headless --dumpfile=out/optilive12_pnp_2_constrains.pkl


python localization.py --video_type=live_rec_gy86 --video data/manuvers_optitrack/test11\
			 --pnp 2 --ftrang=100 --rest --rotation_bounds="3.0,3.0,3.0" --headless --dumpfile=out/optilive11_pnp_2_constrains.pkl


##adding zest & rest
#python localization.py --video_type=sim --pnp 2 --point_noise=0.5 --zest --rest
