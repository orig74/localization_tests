##experiment data creation

##running pure sim:
### problem presentation
#### no problem 
python localization.py --video_type=sim --pnp 1 --point_noise=0 --rest --zest --sim_npoints=7 --sim_spread=0.2
#### local minima error
python localization.py --video_type=sim --pnp 1 --point_noise=0 --rest --zest --sim_npoints=3 --sim_spread=0.2
#### local minima extream error
python localization.py --video_type=sim --pnp 1 --point_noise=0 --rest --zest --sim_npoints=3 --sim_spread=0.08
#### changing the spread slightly shows better solution
python localization.py --video_type=sim --pnp 1 --point_noise=0 --rest --zest --sim_npoints=3 --sim_spread=0.1

python localization.py --video_type=sim --pnp 2 --point_noise=0.5
python localization.py --video_type=sim --pnp 3 --point_noise=0.5

##adding zest & rest
python localization.py --video_type=sim --pnp 2 --point_noise=0.5 --zest --rest
