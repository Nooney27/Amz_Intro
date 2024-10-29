import numpy as np
import matplotlib.pyplot as plt

class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0
        self.integral = 0
    
    def calculate_control(self, error, dt):
        # Proportional term
        P = self.Kp * error
        
        # Integral term
        self.integral += error * dt
        I = self.Ki * self.integral
        
        # Derivative term
        D = self.Kd * (error - self.prev_error) / dt
        self.prev_error = error
        
        # Combine terms to get control output
        control = P + I + D
        return control

# Simulation parameters
Kp, Ki, Kd = 3.0, 0.8, 1.00 # Play with this values to see how the robot behaves
pid = PIDController(Kp, Ki, Kd)

total_time = 10  # seconds
time_step = 0.1  # seconds
desired_position = 10  # Target position for the robot

# Initialize variables for simulation
position = 0  # Starting position
velocity = 0  # Starting velocity
positions = []  # To store position over time for visualization
time_steps = int(total_time / time_step)

# Run the PID control simulation
for _ in range(time_steps):
    error = desired_position - position  # Calculate current error
    control_signal = pid.calculate_control(error, time_step)  # Calculate PID output
    velocity += control_signal * time_step  # Update velocity
    position += velocity * time_step  # Update position based on velocity
    
    positions.append(position)  # Store position for visualization

# Plot the position over time
plt.figure(figsize=(10, 5))
plt.plot(np.arange(0, total_time, time_step), positions, label="Robot Position")
plt.axhline(y=desired_position, color='r', linestyle='--', label="Desired Position")
plt.xlabel("Time (s)")
plt.ylabel("Position")
plt.title("PID Control Simulation: Robot Moving to Desired Position")
plt.legend()
plt.grid()
plt.show()
