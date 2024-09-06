# Neural Control Message
uint64 timestamp			# time since system start (microseconds)
uint64 timestamp_sample	    # the timestamp the data this control response is based on was sampled

float32[6] wrench           # Forces and Torques
float32[4] motor_thrust     # Thrust per motor
float32[15] observation     # Everything else