time stamp

# 7 DoF target joint values
float64[] q_FRI
# The stiffness, damping and tau should be 7 dimensional vectors
float64[] k_FRI
float64[] d_FRI # Damping is normalised (should be set between [0,1] for all dims)
float64[] tau_FRI # If not set is taken as zeroes