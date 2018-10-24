# usage: ./onebox_util setwmmparams <interface_name> <wmm_param_name> <value> <ac> <ap/sta> <end_of_wmm_params>
# NOTE: These are the default parameters. If user wants to set his own WMM parameters 
# change the values accordingly
# argv[7] if set indicates that the end of wmm parameters and no more parameters needs to be added


#################  AP WMM VALUES ###########################

# To set the QOS parameters for Background queue.
./onebox_util  wifi0 setwmmparams aifs  7     BK_Q   self 0
./onebox_util  wifi0 setwmmparams cwmin 4     BK_Q   self 0
./onebox_util  wifi0 setwmmparams cwmax 10    BK_Q   self 0
./onebox_util  wifi0 setwmmparams txop  0     BK_Q   self 0
./onebox_util  wifi0 setwmmparams acm   0     BK_Q   self 0

# To set the QOS parameters for Best Effort queue.
./onebox_util  wifi0 setwmmparams aifs  3  BE_Q  self  0
./onebox_util  wifi0 setwmmparams cwmin 4  BE_Q  self  0
./onebox_util  wifi0 setwmmparams cwmax 6  BE_Q  self  0
./onebox_util  wifi0 setwmmparams txop  0  BE_Q  self  0
./onebox_util  wifi0 setwmmparams acm   0  BE_Q  self  0


# To set the QOS parameters for video queue.
./onebox_util  wifi0 setwmmparams aifs  1   VI_Q  self   0
./onebox_util  wifi0 setwmmparams cwmin 3   VI_Q  self   0
./onebox_util  wifi0 setwmmparams cwmax 4  VI_Q  self   0
./onebox_util  wifi0 setwmmparams txop  94  VI_Q  self   0
./onebox_util  wifi0 setwmmparams acm   0   VI_Q  self   0


# To set the QOS parameters for voice queue.
./onebox_util  wifi0 setwmmparams aifs  1    VO_Q  self   0
./onebox_util  wifi0 setwmmparams cwmin 2    VO_Q  self   0
./onebox_util  wifi0 setwmmparams cwmax 3    VO_Q  self   0
./onebox_util  wifi0 setwmmparams txop  47   VO_Q  self   0
./onebox_util  wifi0 setwmmparams acm   0    VO_Q  self   1

# To enable WMM
# iwpriv wifi0 wmm 1
