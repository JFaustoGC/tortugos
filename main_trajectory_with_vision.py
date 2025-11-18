import time
import math
from robot import Robot
from unicycle_controller import UnicycleController
from vision_tracker import VisionTracker


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
    LINEAR_VELOCITY = 1.0   # m/s (linear speed)
    ANGULAR_VELOCITY = 0.1  # rad/s (rotation speed)
    CIRCLE_RADIUS = LINEAR_VELOCITY / ANGULAR_VELOCITY  # Calculated radius: v/ω
    TRAJECTORY_DURATION = 20.0  # seconds (run for 20 seconds)
    
    # Vision tracking parameters
    AREA_WIDTH = 2.45   # meters
    AREA_HEIGHT = 1.47  # meters
    CAMERA_ROI = (360, 180, 1200, 720)  # ROI configuration
    
    # Starting position offset (center of tracking area)
    START_X = AREA_WIDTH / 2.0   # Start at center X (1.2m from left)
    START_Y = AREA_HEIGHT / 2.0  # Start at center Y (0.725m from top)
    
    # Robot configuration
    robot = Robot(
        name="RAM06",
        mac_address="98:D3:32:20:28:46",
        rfcomm_port="/dev/rfcomm0"
    )
    
    # Controller
    unicycle = UnicycleController(
        wheel_base=abs(WHEEL_BASE),
        wheel_radius=WHEEL_RADIUS
    )
    
    # Vision tracker
    vision = VisionTracker(
        camera_id=4,  # Logitech HD Pro Webcam C920
        area_width_m=AREA_WIDTH,
        area_height_m=AREA_HEIGHT,
        roi=CAMERA_ROI
    )
    
    print(f"Connecting to {robot.name}...")
    
    if robot.connect():
        print("Starting vision tracker...")
        if vision.start():
            print(f"Starting circular trajectory:")
            print(f"  Linear velocity: {LINEAR_VELOCITY}m/s")
            print(f"  Angular velocity: {ANGULAR_VELOCITY} rad/s")
            print(f"  Calculated radius: {CIRCLE_RADIUS:.1f}m")
            print(f"  Expected center: X={START_X:.2f}m, Y={START_Y:.2f}m")
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
                        
                        # Get robot position from vision
                        x, y, detected = vision.get_robot_position(show_debug=True)
                        
                        # Debug output with position relative to center
                        if detected:
                            # Calculate position relative to expected center
                            dx = x - START_X
                            dy = y - START_Y
                            distance_from_center = math.sqrt(dx**2 + dy**2)
                            print(f"t={elapsed:.1f}s | vx={vx:.2f}, vy={vy:.3f} | Pos: X={x:.3f}m Y={y:.3f}m | Dist from center: {distance_from_center:.3f}m | cmd={command}     ", end='\r')
                        else:
                            print(f"t={elapsed:.1f}s | vx={vx:.2f}, vy={vy:.3f} | Pos: NOT DETECTED | cmd={command}     ", end='\r')
                        
                        # Send to robot
                        if not robot.send_message(command):
                            print("\nFailed to send. Connection may be lost.")
                            if not robot.reconnect():
                                print("Failed to reconnect. Exiting.")
                                break
                        
                        last_update = current_time
            
            except KeyboardInterrupt:
                print("\nTrajectory interrupted by user")
            finally:
                vision.stop()
                print("\nSending stop command...")
                robot.send_message(unicycle.stop_command())
                time.sleep(0.5)
                robot.disconnect()
        else:
            print("Failed to start vision tracker")
    else:
        print(f"\nFailed to connect to {robot.name}")
        print("Make sure:")
        print(f"1. Bluetooth device is paired: {robot.mac_address}")
        print(f"2. Run: sudo rfcomm bind {robot.rfcomm_port} {robot.mac_address}")
