## working scenarioes:

### regular pnp

    python localization.py --video_type=live_rec_gy86 --video data/manuvers_raw/mov_test_14 --pnp 1 --ftrang=210

### mypnp with auler angle representaion

    python localization.py --video_type=live_rec_gy86 --video data/manuvers_raw/mov_test_14 --pnp 2 --ftrang=210 --repres eulerang


    python localization.py --video_type=sim --pnp=2 --zest --rest


    python localization.py --video_type=ue4 --video data/manuvers_UE4/manuever2 --pnp 2 --zest --rest --repres eulerang


