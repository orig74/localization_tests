## working scenarioes:

### regular pnp

    python localization.py --video_type=live_rec_gy86 --video data/manuvers_raw/mov_test_14 --pnp 1 --ftrang=210

### mypnp with auler angle representaion

    python localization.py --video_type=live_rec_gy86 --video data/manuvers_raw/mov_test_14 --pnp 2 --ftrang=210 --repres eulerang --zest --rest


    python localization.py --video_type=sim --pnp=2 --zest --rest


    python localization.py --video_type=ue4 --video data/manuvers_UE4/manuever2 --pnp 2 --zest --rest --repres eulerang

    python localization.py --video_type=live_rec_gy86 --video data/manuvers_raw/mov_test_17 --pnp 2 --ftrang=130 --repres axisang --zest --rest

    python localization.py --video_type=sim --pnp=2 --repres axisang --zest --rest --point_noise=0.1 --rvec_noise=6


### record with optitrack

#find referance alt frame
python gy86.py --prefix data/manuvers_optitrack/test%s --video 2
python localization.py --video_type=live_rec_gy86 --video data/manuvers_optitrack/test2 --pnp 2 --ftrang=190 --repres axisang --zest --rest

## sending data files to my station
cp data/manuvers_optitrack/test2.* oga13@linux.cosc.canterbury.ac.nz:learn/localization_tests/data/manuvers_optitrack/

## good result on live
python localization.py --video_type=live_rec_gy86 --video data/manuvers_optitrack/test12 --pnp 3 --ftrang=130 --zest --rest


## last running 
python localization.py --video_type=live_rec_gy86 --video data/manuvers_optitrack/test12 --pnp 3 --ftrang=140 --zest --rest

