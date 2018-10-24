#################  STATION  WMM VALUES ###########################

# To set the QOS parameters for Background queue.
./onebox_util  wifi0 setwmmparams aifs  8     BK_Q   broadcast 0
./onebox_util  wifi0 setwmmparams cwmin 4     BK_Q   broadcast 0
./onebox_util  wifi0 setwmmparams cwmax 11    BK_Q   broadcast 0
./onebox_util  wifi0 setwmmparams txop  0     BK_Q   broadcast 0
./onebox_util  wifi0 setwmmparams acm   0     BK_Q   broadcast 0

# To set the QOS parameters for Best Effort queue.
./onebox_util  wifi0 setwmmparams aifs  3  	BE_Q  broadcast 0
./onebox_util  wifi0 setwmmparams cwmin 4      BE_Q  broadcast 0
./onebox_util  wifi0 setwmmparams cwmax 8     BE_Q  broadcast 0
./onebox_util  wifi0 setwmmparams txop  0      BE_Q  broadcast 0
./onebox_util  wifi0 setwmmparams acm   0      BE_Q  broadcast 0



# To set the QOS parameters for video queue.
./onebox_util  wifi0 setwmmparams aifs  4   VI_Q  broadcast  0
./onebox_util  wifi0 setwmmparams cwmin 3   VI_Q  broadcast  0
./onebox_util  wifi0 setwmmparams cwmax 4   VI_Q  broadcast  0
./onebox_util  wifi0 setwmmparams txop  94  VI_Q  broadcast  0
./onebox_util  wifi0 setwmmparams acm   0   VI_Q  broadcast  0


# To set the QOS parameters for voice queue.
./onebox_util  wifi0 setwmmparams aifs  2    VO_Q  broadcast  0
./onebox_util  wifi0 setwmmparams cwmin 1    VO_Q  broadcast  0
./onebox_util  wifi0 setwmmparams cwmax 3    VO_Q  broadcast  0
./onebox_util  wifi0 setwmmparams txop  47   VO_Q  broadcast  0
./onebox_util  wifi0 setwmmparams acm   0    VO_Q  broadcast  1



