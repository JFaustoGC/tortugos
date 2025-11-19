"""
Test Vision Coordinates with Keyboard Control
Control RAM06 with keyboard while displaying real-time vision tracking data.
"""

import time
import math
from robot import Robot
from unicycle_controller import UnicycleController
from keyboard_controller import KeyboardController
from vision_tracker import VisionTracker


def main():
    # Robot physical parameters
    WHEEL_RADIUS = 0.06471 / 2  # meters
    WHEEL_BASE = 0.07782         # meters
    UPDATE_RATE = 0.05           # seconds (20 Hz)
    
    # Vision tracking parameters
    AREA_WIDTH = 2.4   # meters
    AREA_HEIGHT = 1.45  # meters
    CAMERA_ROI = (360, 180, 1200, 720)
    MARKER_ID = None  # Track any ArUco marker
    
    # Robot configuration (RAM06)
    robot = Robot(
        name="RAM06",
        mac_address="98:D3:32:20:28:46",
        rfcomm_port="/dev/rfcomm0"
    )
    
    # Controllers
    unicycle = UnicycleController(
        wheel_base=WHEEL_BASE,
        wheel_radius=WHEEL_RADIUS
    )
    keyboard = KeyboardController(
        acceleration=0.4, 
        deceleration=0.2, 
        max_speed=10.0
    )
    
    # Vision tracker
    vision = VisionTracker(
        camera_id=4,
        area_width_m=AREA_WIDTH,
        area_height_m=AREA_HEIGHT,
        roi=CAMERA_ROI
    )
    
    if MARKER_ID is not None:
        vision.set_target_marker(MARKER_ID)
    
    print("=" * 70)
    print("Vision Coordinate Testing with Keyboard Control - RAM06")
    print("=" * 70)
    print(f"Tracking area: {AREA_WIDTH}m x {AREA_HEIGHT}m")
    print(f"Camera: Logitech HD Pro Webcam C920 (ID: 4)")
    print(f"ROI: {CAMERA_ROI}")
    print("=" * 70)
    print()
    
    # Connect to robot
    print(f"Connecting to {robot.name}...")
    if not robot.connect():
        print("Failed to connect to robot!")
        return
    print("Connected!\n")
    
    # Start vision tracking
    print("Starting vision tracker...")
    if not vision.start():
        print("Failed to start vision tracker!")
        robot.disconnect()
        return
    print("Vision tracker ready!\n")
    
    # Print keyboard controls
    keyboard.print_help()
    print("\nAdditional Info:")
    print("  - Position coordinates are in meters from center (0, 0)")
    print("  - Angle is in degrees (0° = right, 90° = up, ±180° = left)")
    print("=" * 70)
    print()
    
    keyboard.start()
    
    try:
        last_update = time.time()
        position_history = []
        
        while True:
            # Continuously check for key presses
            keyboard.check_keys()
            
            current_time = time.time()
            if current_time - last_update >= UPDATE_RATE:
                # Get velocity from keyboard
                vx, vy = keyboard.get_velocity()
                
                # Compute robot command
                command = unicycle.compute_command_from_direction(vx, vy, scale=1.0)
                
                # Get robot position and orientation from vision
                x, y, theta, detected = vision.get_robot_position(show_debug=True)
                
                if detected:
                    # Convert theta to degrees for display
                    theta_deg = math.degrees(theta)
                    
                    # Store position history
                    position_history.append((current_time, x, y, theta_deg))
                    
                    # Keep only last 100 positions
                    if len(position_history) > 100:
                        position_history.pop(0)
                    
                    # Calculate velocity from position changes (if enough history)
                    velocity_x = 0.0
                    velocity_y = 0.0
                    if len(position_history) >= 2:
                        dt_pos = position_history[-1][0] - position_history[-2][0]
                        if dt_pos > 0:
                            velocity_x = (position_history[-1][1] - position_history[-2][1]) / dt_pos
                            velocity_y = (position_history[-1][2] - position_history[-2][2]) / dt_pos
                    
                    # Calculate distance from origin
                    distance_from_origin = math.sqrt(x**2 + y**2)
                    
                    # Display comprehensive status
                    print(f"\n{'='*70}")
                    print(f"Time: {current_time - time.time() + current_time:.2f}s")
                    print(f"{'-'*70}")
                    print(f"POSITION:")
                    print(f"  X:       {x:+8.4f} m")
                    print(f"  Y:       {y:+8.4f} m")
                    print(f"  Distance from origin: {distance_from_origin:8.4f} m")
                    print(f"{'-'*70}")
                    print(f"ORIENTATION:")
                    print(f"  Theta:   {theta_deg:+8.2f} °")
                    print(f"  Theta:   {theta:+8.4f} rad")
                    print(f"{'-'*70}")
                    print(f"ESTIMATED VELOCITY (from vision):")
                    print(f"  Vx:      {velocity_x:+8.4f} m/s")
                    print(f"  Vy:      {velocity_y:+8.4f} m/s")
                    print(f"  Speed:   {math.sqrt(velocity_x**2 + velocity_y**2):8.4f} m/s")
                    print(f"{'-'*70}")
                    print(f"COMMANDED VELOCITY (from keyboard):")
                    print(f"  Vx:      {vx:+8.4f}")
                    print(f"  Vy:      {vy:+8.4f}")
                    print(f"{'-'*70}")
                    print(f"Command: {command}")
                    print(f"{'='*70}", end='')
                    
                else:
                    print(f"\n{'='*70}")
                    print(f"⚠ ROBOT NOT DETECTED")
                    print(f"Commanded Vel: vx={vx:+.2f}, vy={vy:+.2f}")
                    print(f"Command: {command}")
                    print(f"{'='*70}", end='')
                
                # Send command to robot
                if not robot.send_message(command):
                    print("\nFailed to send. Connection may be lost.")
                    if not robot.reconnect():
                        print("Failed to reconnect. Exiting.")
                        break
                
                last_update = current_time
    
    except KeyboardInterrupt:
        print("\n\nStopping robot...")
    
    finally:
        keyboard.stop()
        vision.stop()
        print("Sending stop command...")
        robot.send_message(unicycle.stop_command())
        time.sleep(0.5)
        robot.disconnect()
        
        # Print summary statistics
        if position_history:
            print("\n" + "="*70)
            print("SESSION SUMMARY")
            print("="*70)
            x_vals = [p[1] for p in position_history]
            y_vals = [p[2] for p in position_history]
            theta_vals = [p[3] for p in position_history]
            
            print(f"Samples collected: {len(position_history)}")
            print(f"\nX Position Range:")
            print(f"  Min: {min(x_vals):+8.4f} m")
            print(f"  Max: {max(x_vals):+8.4f} m")
            print(f"  Avg: {sum(x_vals)/len(x_vals):+8.4f} m")
            print(f"\nY Position Range:")
            print(f"  Min: {min(y_vals):+8.4f} m")
            print(f"  Max: {max(y_vals):+8.4f} m")
            print(f"  Avg: {sum(y_vals)/len(y_vals):+8.4f} m")
            print(f"\nTheta Range:")
            print(f"  Min: {min(theta_vals):+8.2f} °")
            print(f"  Max: {max(theta_vals):+8.2f} °")
            print(f"  Avg: {sum(theta_vals)/len(theta_vals):+8.2f} °")
            print("="*70)
        
        print("\nDisconnected. Goodbye!")


if __name__ == "__main__":
    main()