#!/usr/bin/python3
"""
STATEFUL and SYNCHRONIZED Node to control an 8-thruster AUV.
- Starts the simulation with a key press.
- Resets its internal state when the simulation resets.

@author: Gemini Assistant for elymsyr
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Bool
from std_srvs.srv import Empty # Service type for pausing/unpausing
import sys
import termios
import tty
import numpy as np

# --- Mappings from keyboard to motion direction ---
msg = """
STATEFUL & SYNCHRONIZED 8-Thruster AUV Keyboard Controller
---------------------------
p : PLAY / UNPAUSE the simulation (use this to start!)

w/s: surge forward/backward
a/d: sway left/right
r/f: heave up/down

q/e: yaw left/right
up/down arrow: pitch up/down
left/right arrow: roll left/right

space key: EMERGENCY STOP (resets all motion commands to zero)
CTRL-C to quit
"""

move_bindings = {
    'w': np.array([1, 0, 0, 0, 0, 0]), 's': np.array([-1, 0, 0, 0, 0, 0]),
    'd': np.array([0, 1, 0, 0, 0, 0]), 'a': np.array([0, -1, 0, 0, 0, 0]),
    'r': np.array([0, 0, 1, 0, 0, 0]), 'f': np.array([0, 0, -1, 0, 0, 0]),
    'e': np.array([0, 0, 0, 0, 0, 1]), 'q': np.array([0, 0, 0, 0, 0, -1]),
    '\x1b[A': np.array([0, 0, 0, 0, 1, 0]), '\x1b[B': np.array([0, 0, 0, 0, -1, 0]),
    '\x1b[C': np.array([0, 0, 0, 1, 0, 0]), '\x1b[D': np.array([0, 0, 0, -1, 0, 0]),
}

def get_key():
    # ... (this function is unchanged)
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        if key == '\x1b':
            key += sys.stdin.read(2)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key

def motion_to_thrusters(motion_vec, force_per_unit):
    # ... (this function is unchanged)
    forces = np.zeros(8)
    surge, sway, heave, roll, pitch, yaw = motion_vec * force_per_unit
    forces[0] = surge - yaw; forces[1] = surge + yaw
    forces[2] = sway; forces[3] = sway
    forces[4:] += heave
    forces[[4, 5]] += pitch; forces[[6, 7]] -= pitch
    forces[[4, 6]] -= roll; forces[[5, 7]] += roll
    return forces.tolist()

# --- Refactored Class to hold state ---
class KeyboardController(Node):
    def __init__(self):
        super().__init__('keyboard_controller_8d')
        
        # Publishers and Subscribers
        self.thrust_pub = self.create_publisher(Float32MultiArray, '/ucat/thruster_cmd', 10)
        self.reset_sub = self.create_subscription(Bool, '/ucat/reset', self.reset_callback, 10)
        
        # Service Clients
        self.unpause_client = self.create_client(Empty, '/unpause_physics')
        while not self.unpause_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /unpause_physics not available, waiting again...')

        # Member variables to hold state
        self.target_motion = np.zeros(6)
        self.speed_step = 1.0
        self.force_per_unit_speed = 5.0
        self.thruster_cmd_msg = Float32MultiArray()

    def reset_callback(self, msg):
        """Called when the /ucat/reset topic receives a message."""
        if msg.data:
            self.get_logger().info('Reset signal received! Resetting teleop controller state.')
            self.target_motion = np.zeros(6)

    def unpause_simulation(self):
        """Calls the Gazebo service to unpause physics."""
        self.get_logger().info('Unpausing simulation...')
        self.unpause_client.call_async(Empty.Request())

    def publish_command(self):
        """Calculates and publishes the thruster forces."""
        final_forces = motion_to_thrusters(self.target_motion, self.force_per_unit_speed)
        self.thruster_cmd_msg.data = final_forces
        self.thrust_pub.publish(self.thruster_cmd_msg)
        print(f"Motion CMD: {np.round(self.target_motion, 1)} | Thruster Forces: {np.round(final_forces, 1)}", end='\r')
        
    def run_keyboard_loop(self):
        """The main loop to listen for keyboard input."""
        print(msg)
        while rclpy.ok():
            key = get_key()
            if key == '\x03':  # CTRL-C
                break
            
            if key in move_bindings:
                self.target_motion += move_bindings[key] * self.speed_step
            elif key == ' ':
                self.target_motion = np.zeros(6)
            elif key == 'p':
                self.unpause_simulation()
            
            self.publish_command()
            
def main(args=None):
    rclpy.init(args=args)
    controller_node = KeyboardController()
    
    try:
        controller_node.run_keyboard_loop()
    except Exception as e:
        print(e)
    finally:
        print("\nStopping all thrusters...")
        controller_node.target_motion = np.zeros(6)
        controller_node.publish_command()
        controller_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()