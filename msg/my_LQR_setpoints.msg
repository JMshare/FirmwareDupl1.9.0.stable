# Publishing the setpoints used in my_LQR_control

uint64 timestamp		# time since system start (microseconds)
uint64 timestamp_sample	    # the timestamp the data this control response is based on was sampled

float32 y0		
float32 y1		
float32 y2
float32 y3
float32 y4
float32 y5
float32 y6
float32 y7
float32 y8
float32 y9
float32 y10
float32 y11

float32 c0
float32 c1
float32 c2
float32 c3

float32 k_omg_pp
float32 k_omg_pr
float32 k_omg_rp
float32 k_omg_rr
float32 k_omg_qq

float32 k_eps_phiphi
float32 k_eps_phipsi
float32 k_eps_psiphi
float32 k_eps_psipsi
float32 k_eps_thttht

float32 del_c_omg_p
float32 del_c_omg_q
float32 del_c_omg_r
float32 del_c_eps_phi
float32 del_c_eps_tht
float32 del_c_eps_psi


float32 pitch_setpoint 		# [deg]

int32 proj_dpsi_status 		# status of the dpsi projection (0 off, 1 on, 10 projected+, -10 projected-)
float32 dpsi 				# the dpsi value after projection [deg]

int32 proj_theta_status 	# status of the theta projection (0 off, 1 on, 10 projected+, -10 projected-)

int32 tuner_status 			# status of the gains tuner (0 no change, 1 change detected)
float32 gain_scale_p
float32 gain_scale_d
float32 case_int_f_int		# scheduler interval case + f_int

float32 dt

int32 control_status		# check for the controller outputs, 0 ok, 1 nans

int32 do_recursivels
float32 x_rls1
float32 x_rls2
int32 do_adaptive
float32 kp_rls
float32 kphi_rls

float32 gain_limiter_p
float32 gain_limiter_q
float32 gain_limiter_r
int32 oscillating_p
int32 oscillating_q
int32 oscillating_r

# TOPICS my_LQR_setpoints
