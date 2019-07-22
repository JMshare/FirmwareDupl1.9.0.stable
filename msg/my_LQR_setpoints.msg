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

float32 pitch_setpoint 

# TOPICS my_LQR_setpoints