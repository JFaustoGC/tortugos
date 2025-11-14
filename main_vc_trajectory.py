#!/usr/bin/env python3
"""
Virtual Center Trajectory with Vision Feedback
Sends VC trajectory commands to the robot while tracking position with camera.
"""

import time
import math
import numpy as np
from robot import Robot
from unicycle_controller import UnicycleController
from vision_tracker import VisionTracker


def vc_trajectory(t):
    """
    Virtual Center reference trajectory - circular motion
    
    Args:
        t: time in seconds
    
    Returns:
        x, y, theta, xd, yd, xdd, ydd, theta_dot, v
    """
    r = 1.0  # radius in meters
    omega = 1.0  # angular velocity in rad/s
    
    x = r * np.cos(omega * t)
    y = r * np.sin(omega * t)
    xd = -r * omega * np.sin(omega * t)
    yd = r * omega * np.cos(omega * t)
    xdd = -r * omega ** 2 * np.cos(omega * t)
    ydd = -r * omega ** 2 * np.sin(omega * t)
    
    theta = np.arctan2(yd, xd)
    theta_dot = (xd * ydd - yd * xdd) / (xd ** 2 + yd ** 2)
    v = np.sqrt(xd ** 2 + yd ** 2)
    
    return x, y, theta, xd, yd, xdd, ydd, theta_dot, v


def main():
    # Robot parameters
    ROBOT_NAME = "RAM05"
    ROBOT_MAC = "98:D3:32:10:15:96"
    RFCOMM_PORT = "/dev/rfcomm1"
    UPDATE_RATE = 0.05  # 20 Hz
    
    # Trajectory parameters
    TRAJECTORY_DURATION = 50.0  # seconds
    
    # Vision tracking parameters
    AREA_WIDTH = 2.4   # meters
    AREA_HEIGHT = 1.45  # meters
    CAMERA_ROI = (360, 180, 1200, 720)
    MARKER_ID = None  # Track any ArUco marker
    
    print("=" * 60)
    print("Virtual Center Trajectory Execution")
    print("=" * 60)
    print(f"Trajectory: Circular, r=1.0m, ω=1.0rad/s")
    print(f"Duration: {TRAJECTORY_DURATION}s")
    print(f"Update rate: {UPDATE_RATE}s ({1/UPDATE_RATE:.0f} Hz)")
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
        print("Starting trajectory execution...\n")
        
        # Trajectory execution loop
        start_time = time.time()
        last_update = start_time
        
        while True:
            current_time = time.time()
            elapsed_time = current_time - start_time
            dt = current_time - last_update
            
            # Check if trajectory duration exceeded
            if elapsed_time >= TRAJECTORY_DURATION:
                print("\n\nTrajectory completed!")
                break
            
            if dt >= UPDATE_RATE:
                last_update = current_time
                
                # Get current position and orientation from vision
                x, y, theta, detected = vision.get_robot_position(show_debug=True)
                
                if detected:
                    # Compute VC reference at current time
                    x_ref, y_ref, theta_ref, xd, yd, xdd, ydd, theta_dot, v_ref = vc_trajectory(elapsed_time)
                    
                    # Compute angular velocity from trajectory
                    w_ref = theta_dot
                    
                    # Send VC velocity commands directly to robot
                    # Note: Using compute_command which expects (v, vy, omega)
                    # For VC following, we use v_ref and w_ref directly
                    message = controller.compute_command(v_ref, 0.0, w_ref)
                    robot.send_message(message)
                    
                    # Calculate tracking error
                    error_x = x_ref - x
                    error_y = y_ref - y
                    error_theta = math.atan2(math.sin(theta_ref - theta), math.cos(theta_ref - theta))
                    distance_error = math.sqrt(error_x**2 + error_y**2)
                    
                    # Display status
                    print(f"t:{elapsed_time:5.1f}s | "
                          f"Pos:({x:+.3f},{y:+.3f}) θ:{math.degrees(theta):6.1f}° | "
                          f"Ref:({x_ref:+.3f},{y_ref:+.3f}) θ:{math.degrees(theta_ref):6.1f}° | "
                          f"Err:{distance_error:.3f}m | "
                          f"v:{v_ref:+.2f} ω:{w_ref:+.2f}", 
                          end='\r', flush=True)
                else:
                    print(f"t:{elapsed_time:5.1f}s | ArUco marker not detected! Stopping...                    ", 
                          end='\r', flush=True)
                    # Stop if robot not detected
                    message = controller.compute_command(0.0, 0.0, 0.0)
                    robot.send_message(message)
            
            time.sleep(0.01)  # Small sleep to prevent CPU spinning
    
    except KeyboardInterrupt:
        print("\n\nStopping robot...")
    
    finally:
        # Send stop command
        message = controller.compute_command(0.0, 0.0, 0.0)
        robot.send_message(message)
        time.sleep(0.1)
        robot.disconnect()
        vision.stop()
        print("Disconnected. Goodbye!")


if __name__ == "__main__":
    main()
