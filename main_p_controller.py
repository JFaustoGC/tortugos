#!/usr/bin/env python3
"""
P Controller with Vision Feedback using ArUco markers
Drives the robot to target position (1.0, 1.0) using camera feedback.
Uses ArUco marker detection for position and orientation.
"""

import time
import math
from robot import Robot
from unicycle_controller import UnicycleController
from vision_tracker import VisionTracker

def main():
    # Robot parameters
    # ROBOT_NAME = "RAM06"
    # ROBOT_MAC = "98:D3:32:20:28:46"
    ROBOT_NAME = "RAM05"
    ROBOT_MAC = "98:D3:32:10:15:96"
    RFCOMM_PORT = "/dev/rfcomm1"
    UPDATE_RATE = 0.05  # 20 Hz
    
    # P Controller parameters
    TARGET_X = 1.0  # meters
    TARGET_Y = 1.0  # meters
    KP = 2.0  # Proportional gain
    MAX_SPEED = 8.0  # Maximum velocity (protocol limit)
    POSITION_TOLERANCE = 0.05  # meters (5cm) - consider reached when within this distance
    
    # Vision tracking parameters
    AREA_WIDTH = 2.4   # meters
    AREA_HEIGHT = 1.45  # meters
    CAMERA_ROI = (360, 180, 1200, 720)
    MARKER_ID = None  # Track any ArUco marker (set to specific ID if needed)
    
    print("=" * 60)
    print("P Controller with Vision Feedback (ArUco)")
    print("=" * 60)
    print(f"Target position: ({TARGET_X:.2f}, {TARGET_Y:.2f}) m")
    print(f"Kp = {KP}, Max speed = {MAX_SPEED} m/s")
    print(f"Position tolerance: {POSITION_TOLERANCE:.3f} m")
    print("=" * 60)
    print("\nPress Ctrl+C to stop")
    print()
    
    # Initialize components
    robot = Robot(ROBOT_NAME, ROBOT_MAC, RFCOMM_PORT)
    controller = UnicycleController(wheel_radius=0.032355, wheel_base=0.07782)
    vision = VisionTracker(
        camera_id=4,
        area_width_m=AREA_WIDTH,
        area_height_m=AREA_HEIGHT,
        roi=CAMERA_ROI
    )
    
    # Set target marker ID if specified
    if MARKER_ID is not None:
        vision.set_target_marker(MARKER_ID)
        print(f"Tracking ArUco marker ID: {MARKER_ID}")
    else:
        print("Tracking any ArUco marker")
    
    try:
        # Connect to robot
        print("Connecting to robot...")
        if not robot.connect():
            print("Failed to connect to robot!")
            return
        print("Connected!\n")
        
        # Start vision tracking
        print("Starting vision tracker...")
        vision.start()
        print("Vision tracker ready!\n")
        
        # Wait for initial detection
        print("Waiting for ArUco marker detection...")
        detected = False
        x, y, theta = 0.0, 0.0, 0.0
        while not detected:
            x, y, theta, detected = vision.get_robot_position()
            if not detected:
                print("ArUco marker not detected, retrying...")
                time.sleep(0.5)
        
        print(f"Robot detected at position: ({x:.3f}, {y:.3f}) m, orientation: {math.degrees(theta):.1f}°\n")
        print("Starting P controller...\n")
        
        # Control loop
        last_time = time.time()
        target_reached = False
        
        while True:
            current_time = time.time()
            dt = current_time - last_time
            
            if dt >= UPDATE_RATE:
                last_time = current_time
                
                # Get current position and orientation from vision
                x, y, theta, detected = vision.get_robot_position(show_debug=True)
                
                if detected:
                    # Calculate error (target - current)
                    error_x = TARGET_X - x
                    error_y = TARGET_Y - y
                    
                    # Calculate distance to target
                    distance = math.sqrt(error_x**2 + error_y**2)
                    
                    # Check if target reached
                    if distance < POSITION_TOLERANCE:
                        if not target_reached:
                            print(f"\n{'='*60}")
                            print(f"TARGET REACHED! Distance: {distance:.4f} m")
                            print(f"Final position: ({x:.3f}, {y:.3f}) m")
                            print(f"{'='*60}\n")
                            target_reached = True
                        
                        # Stop the robot
                        vx = 0.0
                        vy = 0.0
                    else:
                        target_reached = False
                        
                        # P controller: velocity proportional to error in global frame
                        error_vx_global = KP * error_x
                        error_vy_global = KP * error_y
                        
                        # Transform to robot's local frame
                        cos_theta = math.cos(theta)
                        sin_theta = math.sin(theta)
                        vx = cos_theta * error_vx_global + sin_theta * error_vy_global
                        vy = -sin_theta * error_vx_global + cos_theta * error_vy_global
                        
                        # Limit maximum speed
                        speed = math.sqrt(vx**2 + vy**2)
                        if speed > MAX_SPEED:
                            scale = MAX_SPEED / speed
                            vx *= scale
                            vy *= scale
                    
                    # Send command to robot
                    message = controller.compute_command_from_direction(vx, vy, scale=1.0)
                    robot.send_message(message)
                    
                    # Display status
                    print(f"Pos: ({x:.3f}, {y:.3f}) θ:{math.degrees(theta):6.1f}° | "
                          f"Error: ({error_x:+.3f}, {error_y:+.3f}) | "
                          f"Dist: {distance:.3f} m | "
                          f"Vel: ({vx:+.2f}, {vy:+.2f})", end='\r')
                else:
                    print("ArUco marker not detected! Stopping...", end='\r')
                    # Stop if robot not detected
                    message = controller.compute_command_from_direction(0.0, 0.0, scale=1.0)
                    robot.send_message(message)
            
            time.sleep(0.01)  # Small sleep to prevent CPU spinning
    
    except KeyboardInterrupt:
        print("\n\nStopping robot...")
        # Send stop command
        message = controller.compute_command_from_direction(0.0, 0.0, scale=1.0)
        robot.send_message(message)
        time.sleep(0.1)
    
    finally:
        robot.disconnect()
        vision.stop()
        print("Disconnected. Goodbye!")

if __name__ == "__main__":
    main()
