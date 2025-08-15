# Output of the controller, representing desired actuator and predicted state

float64[8] propeller_thrust   # Desired thrust for each of the 8 propellers
float64[12] x_next            # The predicted state [eta; nu] at the next time step
