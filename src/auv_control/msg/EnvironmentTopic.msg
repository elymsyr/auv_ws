# Represents the state of the vehicle in the environment
# All arrays are in the order [x, y, z, roll, pitch, yaw]

float64[6] eta      # Position and orientation
float64[6] nu       # Linear and angular velocities
float64[6] nu_dot   # Linear and angular accelerations
