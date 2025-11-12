import sys
import tty
import termios
import select
import time


class KeyboardController:
    """Map keyboard input to robot velocity commands"""
    
    def __init__(self, acceleration=0.4, deceleration=0.2, max_speed=10.0):
        """
        Args:
            acceleration: Acceleration increment per update when key pressed
            deceleration: Deceleration rate (units per second)
            max_speed: Maximum velocity for vx and vy
        """
        self.acceleration = acceleration
        self.deceleration = deceleration
        self.max_speed = max_speed
        
        # Current velocity state
        self.vx = 0.0
        self.vy = 0.0
        
        # Currently pressed keys
        self.pressed_w = False
        self.pressed_s = False
        self.pressed_a = False
        self.pressed_d = False
        
        # Last time each key was pressed
        self.last_w_time = 0.0
        self.last_s_time = 0.0
        self.last_a_time = 0.0
        self.last_d_time = 0.0
        
        # Last update time for time-based deceleration
        self.last_update_time = time.time()
        
        # Terminal settings for raw input
        self.old_settings = None
    
    def start(self):
        """Initialize keyboard input (raw mode)"""
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
    
    def stop(self):
        """Restore terminal settings"""
        if self.old_settings:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
    
    def check_keys(self):
        """
        Check for key presses (non-blocking) and update pressed state
        Call this frequently in the main loop
        """
        # Check if key is pressed (non-blocking)
        while select.select([sys.stdin], [], [], 0)[0]:
            key = sys.stdin.read(1)
            self._process_key(key)
    
    def get_velocity(self):
        """
        Get current velocity based on which keys are pressed
        
        Returns:
            (vx, vy): Velocity tuple
        """
        # Update velocities based on currently pressed keys
        self._update_velocities()
        
        # Return current velocities
        return (self.vx, self.vy)
    
    def reset_keys(self):
        """Reset all key press states"""
        self.pressed_w = False
        self.pressed_s = False
        self.pressed_a = False
        self.pressed_d = False
    
    def _update_velocities(self):
        """Update velocities based on pressed keys - incremental acceleration/deceleration"""
        current_time = time.time()
        dt = current_time - self.last_update_time
        self.last_update_time = current_time
        
        # Calculate time since last key press for each axis
        time_since_w = current_time - self.last_w_time
        time_since_s = current_time - self.last_s_time
        time_since_a = current_time - self.last_a_time
        time_since_d = current_time - self.last_d_time
        
        # VX: Forward/Backward (W/S keys only)
        if self.pressed_w and time_since_w < 0.1:
            # Accelerate forward
            self.vx = min(self.vx + self.acceleration, self.max_speed)
        elif self.pressed_s and time_since_s < 0.1:
            # Accelerate backward
            self.vx = max(self.vx - self.acceleration, -self.max_speed)
        else:
            # Decelerate proportionally to elapsed time
            time_since_vx_key = min(time_since_w, time_since_s)
            if self.vx > 0:
                decay = self.deceleration * time_since_vx_key
                self.vx = max(0, self.vx - decay * dt)
            elif self.vx < 0:
                decay = self.deceleration * time_since_vx_key
                self.vx = min(0, self.vx + decay * dt)
        
        # VY: Left/Right (A/D keys only)
        if self.pressed_a and time_since_a < 0.1:
            # Accelerate left
            self.vy = min(self.vy + self.acceleration, self.max_speed)
        elif self.pressed_d and time_since_d < 0.1:
            # Accelerate right
            self.vy = max(self.vy - self.acceleration, -self.max_speed)
        else:
            # Decelerate proportionally to elapsed time
            time_since_vy_key = min(time_since_a, time_since_d)
            if self.vy > 0:
                decay = self.deceleration * time_since_vy_key
                self.vy = max(0, self.vy - decay * dt)
            elif self.vy < 0:
                decay = self.deceleration * time_since_vy_key
                self.vy = min(0, self.vy + decay * dt)
    
    def _process_key(self, key):
        """
        Process keyboard input and mark keys as pressed
        
        Key mapping:
            w: Forward (increment vx)
            s: Backward (decrement vx)
            a: Left (increment vy)
            d: Right (decrement vy)
            space: Emergency stop
            q: Quit
        
        Args:
            key: Character from keyboard
        """
        key = key.lower()
        current_time = time.time()
        
        # Mark which keys are pressed this frame
        if key == 'w':
            self.pressed_w = True
            self.pressed_s = False  # Cancel opposite
            self.last_w_time = current_time
        elif key == 's':
            self.pressed_s = True
            self.pressed_w = False  # Cancel opposite
            self.last_s_time = current_time
        elif key == 'a':
            self.pressed_a = True
            self.pressed_d = False  # Cancel opposite
            self.last_a_time = current_time
        elif key == 'd':
            self.pressed_d = True
            self.pressed_a = False  # Cancel opposite
            self.last_d_time = current_time
        
        # Emergency stop
        elif key == ' ':
            self.vx = 0.0
            self.vy = 0.0
            self.pressed_w = False
            self.pressed_s = False
            self.pressed_a = False
            self.pressed_d = False
        
        # Quit
        elif key == 'q':
            raise KeyboardInterrupt
    
    def print_help(self):
        """Print keyboard controls"""
        print("\n=== Keyboard Controls ===")
        print("W: Forward")
        print("S: Backward")
        print("A: Left")
        print("D: Right")
        print("")
        print("Hold to accelerate, release to decelerate")
        print("Wheel speed limit: ±4.0")
        print("")
        print("Space: Emergency stop")
        print("Q: Quit")
        print("=========================\n")
