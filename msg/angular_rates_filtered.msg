# Taking the rates from the vehicle_attitude and filtering them using the Lowpass filter with cutoff freqn set up by MY_LQR_RATES_CUTOFFFN

uint64 timestamp		# time since system start (microseconds)
uint64 timestamp_sample	    # the timestamp the data this control response is based on was sampled

float32 rollspeed		# filtered angular velocity about X body axis in rad/s
float32 pitchspeed		# filtered angular velocity about Y body axis in rad/s
float32 yawspeed		# filtered angular velocity about Z body axis in rad/s
float32 loop_update_freqn # update rate param in lp_filter_omg and lp_fiter_eps
float32 cutoff_freqn_omg # cutoff param in lp_filter_omg
int32 filter_status_omg 		# filter status (0 = ok, 1 = NaNs, 2 = too high cutofffqn, 22 Nans so lp2, 23 Nans so lp3)

float32 roll		# filtered roll about X body axis in rad
float32 pitch		# filtered pitch about Y body axis in rad
float32 yaw			# filtered yaw about Z body axis in rad
float32 cutoff_freqn_eps # cutoff param in lp_filter_eps
int32 filter_status_eps 		# filter status (0 = ok, 1 = NaNs, 2 = too high cutofffqn, 10 NaNs so eps fed in, 22 Nans so lp2, 23 Nans so lp3)

int32 lpf_order		# order of the lpf used

float32 rc_roll		    # filtered RC roll input 
float32 rc_pitch		# filtered RC pitch input
float32 rc_yaw			# filtered RC yaw input
float32 cutoff_freqn_rc # cutoff param in lp_filter_RC
int32 filter_status_rc 		# filter status (0 = ok, 1 = NaNs, 2 = too high cutofffqn)
# always using 2nd order for RC for safety

float32 hdot		      # filtered hdot
float32 cutoff_freqn_hdot # cutoff param in lp_filter_hdot
int32 filter_status_hdot  # filter status (0 = ok, 1 = NaNs, 2 = too high cutofffqn)


# TOPICS angular_rates_filtered
