import time
import math
from robot import Robot
from unicycle_controller import UnicycleController


def generate_circle_trajectory(linear_velocity=1.0, angular_velocity=0.1, dt=0.05):
    """
    Generate a circular trajectory for a differential drive robot
    
    Args:
        linear_velocity: Constant forward velocity in m/s
        angular_velocity: Constant angular velocity in rad/s
        dt: Time step in seconds
    
    Yields:
        (vx, vy): Velocity commands computed from unicycle model
    """
    while True:
        # For circular motion in unicycle model:
        # v = linear velocity (forward)
        # omega = angular velocity (turning rate)
        # 
        # To convert to (vx, vy) for the controller, we need to invert:
        # For a differential drive, the relationship is:
        # vx = v (forward velocity becomes vx)
        # vy = L * omega / 2 (turning contributes to lateral motion)
        # But actually, for differential drive vx and vy map directly to wheel speeds
        # 
        # The correct mapping is:
        # v_left = v - (L/2)*omega
        # v_right = v + (L/2)*omega
        # 
        # And from compute_command_from_direction:
        # v_left = vx + vy
        # v_right = vx - vy
        # 
        # Therefore:
        # vx + vy = v - (L/2)*omega
        # vx - vy = v + (L/2)*omega
        # 
        # Solving:
        # 2*vx = 2*v  =>  vx = v
        # 2*vy = -(L)*omega  =>  vy = -(L/2)*omega
        
        vx = linear_velocity
        vy = -(0.07782 / 2.0) * angular_velocity  # Using wheel base
        
        yield (vx, vy)


if __name__ == "__main__":
    # Robot physical parameters
    WHEEL_RADIUS = 0.06471 / 2  # meters
    WHEEL_BASE = -0.07782        # meters (negative indicates sign convention)
    UPDATE_RATE = 0.05           # seconds (20 Hz)
    
    # Trajectory parameters
    LINEAR_VELOCITY = 2.0   # m/s (linear speed)
    ANGULAR_VELOCITY = 8.0  # rad/s (rotation speed)
    CIRCLE_RADIUS = LINEAR_VELOCITY / ANGULAR_VELOCITY  # Calculated radius: v/ω
    TRAJECTORY_DURATION = 40.0  # seconds (run for 20 seconds)
    
    # Robot configuration
    ram_06 = Robot(
        name="RAM06",
        mac_address="98:D3:32:20:28:46",
        rfcomm_port="/dev/rfcomm0"
    )
    
    ram_05 = Robot(
        name="RAM05",
        mac_address="98:D3:32:10:15:96",
        rfcomm_port="/dev/rfcomm1"
    )
    
    # Controller
    unicycle = UnicycleController(
        wheel_base=abs(WHEEL_BASE),
        wheel_radius=WHEEL_RADIUS
    )
    
    print(f"Connecting to {ram_06.name}...")
    print(f"Connecting to {ram_05.name}...")
    
    if ram_06.connect() and ram_05.connect():
        print(f"Starting circular trajectory:")
        print(f"  Linear velocity: {LINEAR_VELOCITY}m/s")
        print(f"  Angular velocity: {ANGULAR_VELOCITY} rad/s")
        print(f"  Calculated radius: {CIRCLE_RADIUS:.1f}m")
        print(f"  Duration: {TRAJECTORY_DURATION}s")
        print("\nPress Ctrl+C to stop")
        
        # Generate trajectory
        trajectory = generate_circle_trajectory(
            linear_velocity=LINEAR_VELOCITY,
            angular_velocity=ANGULAR_VELOCITY,
            dt=UPDATE_RATE
        )
        
        try:
            start_time = time.time()
            last_update = start_time
            
            while True:
                current_time = time.time()
                elapsed = current_time - start_time
                
                # Stop after trajectory duration
                if elapsed >= TRAJECTORY_DURATION:
                    print(f"\nTrajectory completed after {elapsed:.1f}s")
                    break
                
                # Check if enough time has passed since last update
                if current_time - last_update >= UPDATE_RATE:
                    # Get next velocity from trajectory (vx, vy)
                    vx, vy = next(trajectory)
                    
                    # Compute robot command using direction method
                    command = unicycle.compute_command_from_direction(vx, vy, scale=1.0)
                    
                    # Debug output
                    print(f"t={elapsed:.1f}s | vx={vx:.2f}, vy={vy:.3f} | cmd={command}     ", end='\r')
                    
                    # Send to robot
                    if not ram_06.send_message(command) or not ram_05.send_message(command):
                        print("\nFailed to send. Connection may be lost.")
                        if not ram_06.reconnect() or not ram_05.reconnect():
                            print("Failed to reconnect. Exiting.")
                            break
                    
                    last_update = current_time
        
        except KeyboardInterrupt:
            print("\nTrajectory interrupted by user")
        finally:
            print("\nSending stop command...")
            ram_06.send_message(unicycle.stop_command())
            ram_05.send_message(unicycle.stop_command())
            time.sleep(0.5)
            ram_06.disconnect()
            ram_05.disconnect()
    else:
        print(f"\nFailed to connect to {ram_06.name} or {ram_05.name}")
        print("Make sure:")
        print(f"1. Bluetooth device is paired: {ram_06.mac_address} and {ram_05.mac_address}")
        print(f"2. Run: sudo rfcomm bind {ram_06.rfcomm_port} {ram_06.mac_address}")
        print(f"   sudo rfcomm bind {ram_05.rfcomm_port} {ram_05.mac_address}")
