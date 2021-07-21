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
float32 pitch_setpoint_ramp # [deg]

int32 proj_dpsi_status 		# status of the dpsi projection (0 off, 1 on, 10 projected+, -10 projected-)
float32 dpsi 				# the dpsi value after projection [deg]

int32 proj_theta_status 	# status of the theta projection (0 off, 1 on, 10 projected+, -10 projected-)

int32 tuner_status 			# status of the gains tuner (0 no change, 1 change detected)
float32 gain_scale_p_p
float32 gain_scale_d_p
float32 gain_scale_p_q
float32 gain_scale_d_q
float32 gain_scale_p_r
float32 gain_scale_d_r
int32 tuner_mode			# which states I am tunning

float32 case_int_f_int		# scheduler interval case + f_int
int32 scheduler_status 	# scheduler status (0..off, 1..on, 2..limited by airsp)

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


float32 sp_y0
float32 sp_y1		
float32 sp_y2
float32 sp_y3
float32 sp_y4
float32 sp_y5
float32 sp_y6			
float32 sp_y7
float32 sp_y8
float32 sp_y9
float32 sp_y10
float32 sp_y11

float32 del_c_x_q
float32 del_c_x_m
float32 del_c_v_q
float32 del_c_v_m

float32 k_x_qz
float32 k_x_mz
float32 k_v_qvz
float32 k_v_mvz

int32 alt_mode 				# altitude stabilisation 0 off, 1 cruise, 2 slow, -2 slow rejected
float32 c_alt_support    	# remainder after cropping the Del_c_x and Del_c_v to [0,1] -> front engine
int32 c_alt_support_bool	# do this or not?

float32 c_eps_satur_p 		# remainder after cropping the Del_c_eps to [-1,1] -> roll saturation to be coupled to yaw
float32 c_eps_satur_q
float32 c_eps_satur_r
float32 c_eps_satur_m
int32 c_eps_satur_bool 		# do this or not?




# TOPICS my_LQR_setpoints
