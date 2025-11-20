import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrow
from robot import Robot
from unicycle_controller import UnicycleController
from vision_tracker import VisionTracker
from references import vc_trajectory, follower_reference
from controllers import follower_control


if __name__ == "__main__":
    # Robot physical parameters
    WHEEL_RADIUS = 0.06471 / 2  # meters
    WHEEL_BASE = -0.07782        # meters (negative indicates sign convention)
    UPDATE_RATE = 0.05           # seconds (20 Hz)
    
    # Vision tracking parameters
    AREA_WIDTH = 2.4   # meters
    AREA_HEIGHT = 1.45  # meters
    CAMERA_ROI = (360, 180, 1200, 720)  # ROI configuration
    
    # Starting position offset (center of tracking area)
    # START_X = AREA_WIDTH / 2.0   # Start at center X (1.2m from left)
    # START_Y = AREA_HEIGHT / 2.0  # Start at center Y (0.725m from top)
    START_X = 0
    START_Y = 0
    
    # Follower offset in body frame (single follower)
    FOLLOWER_OFFSET = np.array([0.0, 0.0])  # No offset, follows virtual center directly
    
    # Controller gains
    GAINS = {
        'cx': 0.8,
        'ct': 10.0,
        'cy': 1.2,
        'consensus': 0.0  # No consensus for single follower
    }
    K_PARAM = 0.00  # Design parameter for alpha
    
    TRAJECTORY_DURATION = 1000.0  # seconds
    
    # Robot configuration
    robot = Robot(
        name="RAM06",
        mac_address="98:D3:32:20:28:46",
        rfcomm_port="/dev/rfcomm0"
    )
    
    # robot = Robot(
    #     name="RAM05",
    #     mac_address="98:D3:32:10:15:96",
    #     rfcomm_port="/dev/rfcomm1"
    # )
    
    # Controller
    unicycle = UnicycleController(
        wheel_base=abs(WHEEL_BASE),
        wheel_radius=WHEEL_RADIUS
    )
    
    # Vision tracker
    vision = VisionTracker(
        camera_id=0,  # Logitech HD Pro Webcam C920
        area_width_m=AREA_WIDTH,
        area_height_m=AREA_HEIGHT,
        roi=CAMERA_ROI
    )
    
    # Setup matplotlib for live plotting
    plt.ion()
    fig, ax = plt.subplots(figsize=(10, 6))
    ax.set_xlim(-AREA_WIDTH/2, AREA_WIDTH/2)
    ax.set_ylim(-AREA_HEIGHT/2, AREA_HEIGHT/2)
    ax.set_xlabel('X (meters)')
    ax.set_ylabel('Y (meters)')
    ax.set_title('Robot Trajectory Tracking')
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')
    
    # Plot formatting
    ax.xaxis.set_major_locator(plt.MultipleLocator(0.2))
    ax.yaxis.set_major_locator(plt.MultipleLocator(0.2))
    ax.xaxis.set_minor_locator(plt.MultipleLocator(0.1))
    ax.yaxis.set_minor_locator(plt.MultipleLocator(0.1))
    ax.grid(which='major', alpha=0.5, linestyle='-', linewidth=0.8)
    ax.grid(which='minor', alpha=0.2, linestyle=':', linewidth=0.5)
    
    # Initialize plot elements
    robot_trail, = ax.plot([], [], 'b-', linewidth=1, label='Robot path', alpha=0.6)
    ref_trail, = ax.plot([], [], 'r--', linewidth=1, label='Reference path', alpha=0.6)
    robot_pos, = ax.plot([], [], 'bo', markersize=8, label='Robot')
    ref_pos, = ax.plot([], [], 'ro', markersize=8, label='Reference')
    
    # Arrow for orientation (will be updated)
    ref_arrow = None
    
    ax.legend(loc='upper right')
    
    # Data storage
    robot_x_history = []
    robot_y_history = []
    ref_x_history = []
    ref_y_history = []
    
    print(f"Connecting to {robot.name}...")
    
    if robot.connect():
        print("Starting vision tracker...")
        if vision.start():
            print(f"Starting circular formation trajectory:")
            print(f"  Radius: 0.5m")
            print(f"  Period: 10s")
            print(f"  Follower offset: {FOLLOWER_OFFSET}")
            print(f"  Duration: {TRAJECTORY_DURATION}s")
            print("\nPress Ctrl+C to stop")
            
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
                        # Get virtual center trajectory
                        vc_state = vc_trajectory(elapsed)
                        
                        # Get follower reference
                        x_ref, y_ref, theta_ref, theta_dot_ref, v_ref = follower_reference(
                            vc_state, FOLLOWER_OFFSET
                        )
                        
                        # Offset reference to tracking area center
                        x_ref += START_X
                        y_ref += START_Y
                        
                        # Store reference trajectory
                        ref_x_history.append(x_ref)
                        ref_y_history.append(y_ref)
                        
                        # Get robot position from vision
                        x, y, theta, detected = vision.get_robot_position(show_debug=True)
                        
                        if detected:
                            # Store robot trajectory
                            robot_x_history.append(x)
                            robot_y_history.append(y)
                            
                            
                            
                            # Current state
                            state = [x, y, theta]
                            
                            # Reference (x, y, theta, omega, v)
                            ref = (x_ref, y_ref, theta_ref, theta_dot_ref, v_ref)
                            
                            # Compute control (no neighbors for single follower)
                            v_cmd, w_cmd = follower_control(
                                state=state,
                                ref=ref,
                                neighbors_errors=None,
                                connectivity_row=None,
                                gains=GAINS,
                                k=K_PARAM
                            )
                            
                            # Convert to wheel commands
                            command = unicycle.compute_command_from_velocities(v_cmd, w_cmd)
                            
                            # Calculate error for display
                            error_x = x_ref - x
                            error_y = y_ref - y
                            error_dist = np.sqrt(error_x**2 + error_y**2)
                            
                            # Update plot
                            robot_trail.set_data(robot_x_history, robot_y_history)
                            ref_trail.set_data(ref_x_history, ref_y_history)
                            robot_pos.set_data([x], [y])
                            ref_pos.set_data([x_ref], [y_ref])
                            
                            # Update reference orientation arrow
                            if ref_arrow:
                                ref_arrow.remove()
                            arrow_length = 0.15
                            dx = arrow_length * np.cos(theta_ref)
                            dy = arrow_length * np.sin(theta_ref)
                            ref_arrow = FancyArrow(
                                x_ref, y_ref, dx, dy,
                                width=0.03, head_width=0.08, head_length=0.06,
                                color='red', alpha=0.7, zorder=5
                            )
                            ax.add_patch(ref_arrow)
                            
                            plt.pause(0.001)
                            
                            print(f"t={elapsed:.1f}s | Ref: ({x_ref:.3f}, {y_ref:.3f}) θ={np.degrees(theta_ref):.1f}° | "
                                  f"Pos: ({x:.3f}, {y:.3f}) | Error: {error_dist:.3f}m | "
                                  f"v={v_cmd:.2f} w={w_cmd:.2f}     ", end='\r')
                        else:
                            # No detection, stop robot
                            command = unicycle.stop_command()
                            print(f"t={elapsed:.1f}s | Pos: NOT DETECTED | Stopping robot     ", end='\r')
                        
                        # Send to robot
                        if not robot.send_message(command):
                            print("\nFailed to send. Connection may be lost.")
                            if not robot.reconnect():
                                print("Failed to reconnect. Exiting.")
                                break
                        
                        last_update = current_time
            
            finally:
                try:
                    vision.stop()
                except KeyboardInterrupt:
                    print("\nVision stop interrupted, forcing release")

                print("\nSending stop command...")
                robot.send_message(unicycle.stop_command())
                time.sleep(0.5)
                robot.disconnect()
                plt.ioff()
                plt.show()

        else:
            print("Failed to start vision tracker")
    else:
        print(f"\nFailed to connect to {robot.name}")
        print("Make sure:")
        print(f"1. Bluetooth device is paired: {robot.mac_address}")
        print(f"2. Run: sudo rfcomm bind {robot.rfcomm_port} {robot.mac_address}")