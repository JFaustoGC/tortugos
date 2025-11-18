"""
Trajectory executor - Sends precomputed trajectories to real robots.

This script takes a reference trajectory and executes it on a physical robot
using open-loop control (no vision feedback).
"""
import time
import numpy as np
from robot import Robot
from unicycle_controller import UnicycleController
from references import vc_trajectory, follower_reference


class TrajectoryExecutor:
    """Executes precomputed trajectories on physical robots."""
    
    def __init__(self, robot_name, mac_address, rfcomm_port="/dev/rfcomm0",
                 wheel_radius=0.06471/2, wheel_base=0.07782):
        """
        Initialize trajectory executor.
        
        Args:
            robot_name: Name identifier for the robot
            mac_address: Bluetooth MAC address
            rfcomm_port: RFCOMM device path
            wheel_radius: Wheel radius in meters
            wheel_base: Distance between wheels in meters
        """
        self.robot = Robot(
            name=robot_name,
            mac_address=mac_address,
            rfcomm_port=rfcomm_port
        )
        
        self.controller = UnicycleController(
            wheel_base=wheel_base,
            wheel_radius=wheel_radius
        )
        
        self.wheel_base = wheel_base
        self.wheel_radius = wheel_radius
    
    def connect(self):
        """Connect to the robot."""
        return self.robot.connect()
    
    def disconnect(self):
        """Disconnect from the robot."""
        self.robot.disconnect()
    
    def unicycle_to_wheels(self, v, omega):
        """
        Convert unicycle velocities (v, omega) to wheel angular velocities.
        
        Using the differential drive Jacobian:
        [v]     = [R/2   R/2  ] [omega_right]
        [omega]   [R/L  -R/L  ] [omega_left ]
        
        Inverse:
        [omega_right] = 1/R * [v + (L/2)*omega]
        [omega_left ] = 1/R * [v - (L/2)*omega]
        
        Args:
            v: Linear velocity (m/s)
            omega: Angular velocity (rad/s)
        
        Returns:
            (omega_right, omega_left): Wheel angular velocities (rad/s)
        """
        omega_right = (v + (self.wheel_base / 2) * omega) / self.wheel_radius
        omega_left = (v - (self.wheel_base / 2) * omega) / self.wheel_radius
        
        return omega_right, omega_left
    
    def execute_trajectory(self, trajectory_func, duration, dt=0.05, 
                          trajectory_params=None, verbose=True):
        """
        Execute a trajectory function on the robot.
        
        Args:
            trajectory_func: Function that takes (t, **params) and returns 
                           (x, y, theta, v, omega) or similar
            duration: Total execution time in seconds
            dt: Control update interval in seconds
            trajectory_params: Dict of additional parameters for trajectory_func
            verbose: Print progress messages
        
        Returns:
            success: True if trajectory completed successfully
        """
        if trajectory_params is None:
            trajectory_params = {}
        
        if not self.robot.is_connected():
            print(f"[{self.robot.name}] Not connected. Call connect() first.")
            return False
        
        if verbose:
            print(f"\n[{self.robot.name}] Starting trajectory execution")
            print(f"  Duration: {duration}s")
            print(f"  Update rate: {1/dt:.1f} Hz")
            print("  Press Ctrl+C to stop\n")
        
        try:
            start_time = time.time()
            last_update = start_time
            
            while True:
                current_time = time.time()
                elapsed = current_time - start_time
                
                # Stop after duration
                if elapsed >= duration:
                    if verbose:
                        print(f"\n[{self.robot.name}] Trajectory completed after {elapsed:.1f}s")
                    break
                
                # Check if it's time to update
                if current_time - last_update >= dt:
                    # Get reference at current time
                    traj_data = trajectory_func(elapsed, **trajectory_params)
                    
                    # Extract v and omega from trajectory data
                    # Expected formats:
                    # VC: (x, y, theta, xd, yd, xdd, ydd, theta_dot, v)
                    # Follower ref: (x, y, theta, theta_dot, v)
                    
                    if len(traj_data) == 9:  # VC trajectory
                        v = traj_data[8]  # v is last element
                        theta_dot = traj_data[7]  # omega
                    elif len(traj_data) == 5:  # Follower reference
                        v = traj_data[4]
                        theta_dot = traj_data[3]
                    else:
                        raise ValueError(f"Unexpected trajectory data format: {len(traj_data)} elements")
                    
                    omega = theta_dot
                    
                    # Convert to wheel speeds using Jacobian
                    omega_right, omega_left = self.unicycle_to_wheels(v, omega)
                    
                    # Convert wheel angular velocities to linear velocities for command
                    v_right = omega_right * self.wheel_radius
                    v_left = omega_left * self.wheel_radius
                    
                    # Format and send command
                    command = self.controller._format_message(v_right, v_left)
                    
                    if verbose:
                        print(f"t={elapsed:.2f}s | v={v:.3f} ω={omega:.3f} | "
                              f"ω_R={omega_right:.2f} ω_L={omega_left:.2f} | {command}    ",
                              end='\r')
                    
                    if not self.robot.send_message(command):
                        print(f"\n[{self.robot.name}] Failed to send command")
                        if not self.robot.reconnect():
                            print(f"[{self.robot.name}] Reconnection failed")
                            return False
                    
                    last_update = current_time
            
            return True
        
        except KeyboardInterrupt:
            if verbose:
                print(f"\n[{self.robot.name}] Trajectory interrupted by user")
            return False
        
        finally:
            # Always send stop command
            if verbose:
                print(f"[{self.robot.name}] Sending stop command...")
            self.robot.send_message(self.controller.stop_command())
            time.sleep(0.5)


def main():
    """Example: Execute virtual center trajectory on RAM06."""
    
    # Robot configuration
    RAM06_MAC = "98:D3:32:20:28:46"
    
    # Physical parameters
    WHEEL_RADIUS = 0.06471 / 2  # meters
    WHEEL_BASE = 0.07782         # meters
    
    # Trajectory parameters
    DURATION = 50.0  # seconds
    UPDATE_RATE = 0.05  # 20 Hz
    
    # Create executor
    executor = TrajectoryExecutor(
        robot_name="RAM06",
        mac_address=RAM06_MAC,
        rfcomm_port="/dev/rfcomm0",
        wheel_radius=WHEEL_RADIUS,
        wheel_base=WHEEL_BASE
    )
    
    print("Connecting to RAM06...")
    if executor.connect():
        print("Connected successfully!\n")
        
        # Execute VC trajectory
        print("Executing Virtual Center circular trajectory...")
        success = executor.execute_trajectory(
            trajectory_func=vc_trajectory,
            duration=DURATION,
            dt=UPDATE_RATE,
            verbose=True
        )
        
        if success:
            print("\nTrajectory execution completed successfully!")
        else:
            print("\nTrajectory execution failed or was interrupted.")
        
        executor.disconnect()
    else:
        print("\nFailed to connect to RAM06")
        print(f"Make sure Bluetooth is paired and run:")
        print(f"  sudo rfcomm bind /dev/rfcomm0 {RAM06_MAC}")


if __name__ == "__main__":
    main()
