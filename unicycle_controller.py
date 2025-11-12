import math
import numpy as np


class UnicycleController:
    """Unicycle robot kinematics and command formatting"""
    
    def __init__(self, wheel_base=0.2, wheel_radius=0.05):
        """
        Args:
            wheel_base: Distance between wheels in meters (L)
            wheel_radius: Wheel radius in meters (R)
        """
        self.wheel_base = wheel_base
        self.wheel_radius = wheel_radius
    
    def compute_command(self, v, vy, omega):
        """
        Convert unicycle model velocities to wheel speeds
        
        Args:
            v: Linear velocity in forward direction (m/s)
            vy: Lateral velocity (ignored for differential drive)
            omega: Angular velocity (rad/s)
        
        Returns:
            Formatted command string for robot
        """
        # Differential drive kinematics:
        # v_left = v - (L/2) * omega
        # v_right = v + (L/2) * omega
        
        v_left = v - (self.wheel_base / 2.0) * omega
        v_right = v + (self.wheel_base / 2.0) * omega
        
        return self._format_message(v_right, v_left)

    
    def compute_command_from_direction(self, vx, vy, scale=1.0):
        """
        Convert directional velocity to wheel speeds
        Direct assignment: both vx and vy contribute to both wheels for true diagonal motion
        
        Args:
            vx: Desired forward velocity (positive = forward)
            vy: Desired lateral velocity (positive = left)
            scale: Velocity scaling factor
        
        Returns:
            Formatted command string for robot
        """
        # For a differential drive robot to move diagonally, we need both wheels
        # to spin in the same direction but at different speeds.
        # Simple approach: directly assign vx and vy to wheels
        # Left wheel gets vx + vy contribution
        # Right wheel gets vx - vy contribution
        
        v_left = (vx + vy) * scale
        v_right = (vx - vy) * scale
        
        return self._format_message(v_right, v_left)
    
    def _format_message(self, v_right, v_left, limit=8.0):
        """
        Format wheel speeds into robot command string
        
        Args:
            v_right: Right wheel speed
            v_left: Left wheel speed
            limit: Maximum allowed wheel speed
        Returns:
            Formatted string "IR+XX.XXL+XX.XXF\n"
        """
        # clamp to range
        clamp = lambda x: max(-limit, min(limit, x))
        vr, vl = clamp(v_right), clamp(v_left)

        # zero deadband
        deadband = lambda x: 0.0 if abs(x) < 0.01 else x
        vr, vl = deadband(vr), deadband(vl)

        # fixed protocol formatting
        fmt = lambda x: f"{x:+06.2f}"
        return f"IR{fmt(vr)}L{fmt(vl)}F"
    
    
    def stop_command(self):
        """Generate stop command"""
        return "IR+00.00L+00.00F\n"
