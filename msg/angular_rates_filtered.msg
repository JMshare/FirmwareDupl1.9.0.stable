# Taking the rates from the vehicle_attitude and filtering them using the Lowpass filter with cutoff freqn set up by MY_LQR_RATES_CUTOFFFN

uint64 timestamp		# time since system start (microseconds)
uint64 timestamp_sample	    # the timestamp the data this control response is based on was sampled

float32 rollspeed		# filtered angular velocity about X body axis in rad/s
float32 pitchspeed		# filtered angular velocity about Y body axis in rad/s
float32 yawspeed		# filtered angular velocity about Z body axis in rad/s
float32 loop_update_freqn # update rate param in lp_filter_omg and lp_fiter_eps
float32 cutoff_freqn_omg # cutoff param in lp_filter_omg
int32 filter_status_omg 		# filter status (0 = ok, 1 = NaNs, 2 = too high cutofffqn)

float32 roll		# filtered roll about X body axis in rad
float32 pitch		# filtered pitch about Y body axis in rad
float32 yaw			# filtered yaw about Z body axis in rad
float32 cutoff_freqn_eps # cutoff param in lp_filter_eps
int32 filter_status_eps 		# filter status (0 = ok, 1 = NaNs, 2 = too high cutofffqn)

int32 lpf_order		# order of the lpf used

# TOPICS angular_rates_filtered
