float32 iden_state
# -1 disable
# >x>1e-2  start inject now
# > 0 wait for * sec to start

uint8 IDEN_CHN_ROLL_U_OUT = 0
uint8 IDEN_CHN_PITCH_U_OUT = 1
uint8 IDEN_CHN_YAW_U_OUT = 2
uint8 IDEN_CHN_THR_U_OUT = 3

uint8 inject_channel

uint8 IDEN_INJECT_FREQUENCY_SWEEP = 0
uint8 IDEN_INJECT_TIME_DOMAIN_0 = 1
uint8 IDEN_INJECT_TIME_DOMAIN_1 = 2

uint8 inject_signal_mode
float32 inject_param1
float32 inject_param2
float32 inject_param3
float32 inject_param4


#If using INJECT_MODE ==  IDEN_INJECT_FREQUENCY_SWEEP
# start_omg =  inject_signal_param1
# end_omg =  inject_signal_param2
# time =  inject_signal_param3
# amp = inject_signal_param4