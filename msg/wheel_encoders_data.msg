uint64 timestamp	   # time since system start (microseconds)

uint32[6] pulses_per_meter	   # time since system start (microseconds)
float32[6] wheel_diameter    # Cumulative linear meters traveled via an attached wheel for the associated encoder
int64[6] position      # The position of the wheel, in current encoder ticks relative from boot. Positive is forward rotation, negative is reverse rotation
float32[6] distance    # Cumulative linear meters traveled via an attached wheel for the associated encoder
int32[6] qpps         # Speed of each wheel, in encoder pulses per second. Positive is forward, negative is reverse
float32[6] velocity         # Speed of each wheel, in linear meters traveled per second via an attached wheel. Positive is forward, negative is reverse
# TOPICS encoders_data wheel_encoders_data
