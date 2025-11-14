#!/usr/bin/env python3
"""
Follower Controller with Vision Feedback
Single follower tracking VC trajectory using camera feedback.
"""

import time
import math
import numpy as np
from robot import Robot
from unicycle_controller import UnicycleController
from vision_tracker import VisionTracker


def wrap_angle(a):
    """Wrap angle to [-pi, pi]"""
    return np.arctan2(np.sin(a), np.cos(a))


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


def follower_reference(vc_state, P):
    """
    Compute follower reference from VC state and offset
    
    Args:
        vc_state: (x_vc, y_vc, theta_vc, xd_vc, yd_vc, xdd_vc, ydd_vc, theta_dot_vc, v_vc)
        P: [Px, Py] offset in VC body frame
    
    Returns:
        x_f, y_f, theta_f, theta_dot_f, v_f
    """
    x_vc, y_vc, theta_vc, xd_vc, yd_vc, xdd_vc, ydd_vc, theta_dot_vc, v_vc = vc_state
    Px, Py = P
    
    # Follower position
    x_f = x_vc + Px * np.cos(theta_vc) - Py * np.sin(theta_vc)
    y_f = y_vc + Px * np.sin(theta_vc) + Py * np.cos(theta_vc)
    
    # Follower velocity
    xdot_f = xd_vc - Px * np.sin(theta_vc) * theta_dot_vc - Py * np.cos(theta_vc) * theta_dot_vc
    ydot_f = yd_vc + Px * np.cos(theta_vc) * theta_dot_vc - Py * np.sin(theta_vc) * theta_dot_vc
    
    # Follower orientation and speed
    theta_f = np.arctan2(ydot_f, xdot_f)
    v_f = np.sqrt(xdot_f ** 2 + ydot_f ** 2)
    
    # Follower angular velocity
    xddot_f = xdd_vc - Px * (np.cos(theta_vc) * theta_dot_vc ** 2) - Py * (-np.sin(theta_vc) * theta_dot_vc ** 2)
    yddot_f = ydd_vc - Px * (np.sin(theta_vc) * theta_dot_vc ** 2) + Py * (np.cos(theta_vc) * theta_dot_vc ** 2)
    theta_dot_f = (xdot_f * yddot_f - ydot_f * xddot_f) / (xdot_f ** 2 + ydot_f ** 2 + 1e-10)
    
    return x_f, y_f, theta_f, theta_dot_f, v_f


def follower_control(state, ref, cx=0.8, ct=10.0, cy=1.2, k=1.0):
    """
    Follower controller without consensus
    
    Args:
        state: [x, y, theta] current state
        ref: [x_ref, y_ref, theta_ref, w_ref, v_ref] reference
        cx, ct, cy: controller gains
        k: scaling parameter
    
    Returns:
        v, w: linear and angular velocities
    """
    x, y, theta = state
    x_ref, y_ref, theta_ref, w_ref, v_ref = ref
    
    # Position error in global frame
    dx = x_ref - x
    dy = y_ref - y
    
    # Transform to robot body frame
    x_e = np.cos(theta) * dx + np.sin(theta) * dy
    y_e = -np.sin(theta) * dx + np.cos(theta) * dy
    theta_e = wrap_angle(theta_ref - theta)
    
    # Controller
    alpha = np.sqrt(k ** 2 + x_e ** 2 + y_e ** 2)
    v = v_ref * np.cos(theta_e) + cx * x_e
    w = w_ref + ct * theta_e + cy * v_ref * np.sinc(theta_e / np.pi) * (k / alpha) * y_e
    
    # Clip controls
    v = np.clip(v, -8.0, 8.0)
    w = np.clip(w, -3.0, 3.0)
    
    return v, w


def main():
    # Robot parameters
    ROBOT_NAME = "RAM05"
    ROBOT_MAC = "98:D3:32:10:15:96"
    RFCOMM_PORT = "/dev/rfcomm1"
    UPDATE_RATE = 0.05  # 20 Hz
    
    # Trajectory parameters
    TRAJECTORY_DURATION = 50.0  # seconds
    FOLLOWER_OFFSET = np.array([0.5, 0.5])  # [Px, Py] offset in VC body frame
    
    # Controller gains
    CX = 0.8
    CT = 10.0
    CY = 1.2
    K = 1.0
    
    # Vision tracking parameters
    AREA_WIDTH = 2.4   # meters
    AREA_HEIGHT = 1.45  # meters
    CAMERA_ROI = (360, 180, 1200, 720)
    MARKER_ID = None  # Track any ArUco marker
    
    print("=" * 60)
    print("Follower Controller with Vision Feedback")
    print("=" * 60)
    print(f"VC Trajectory: Circular, r=1.0m, ω=1.0rad/s")
    print(f"Follower offset: Px={FOLLOWER_OFFSET[0]}, Py={FOLLOWER_OFFSET[1]}")
    print(f"Duration: {TRAJECTORY_DURATION}s")
    print(f"Gains: cx={CX}, ct={CT}, cy={CY}, k={K}")
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
        print("Starting follower control...\n")
        
        # Control loop
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
                    vc_state = vc_trajectory(elapsed_time)
                    
                    # Compute follower reference
                    x_ref, y_ref, theta_ref, theta_dot_ref, v_ref = follower_reference(vc_state, FOLLOWER_OFFSET)
                    
                    # Current state
                    state = [x, y, theta]
                    ref = [x_ref, y_ref, theta_ref, theta_dot_ref, v_ref]
                    
                    # Compute control
                    v_cmd, w_cmd = follower_control(state, ref, cx=CX, ct=CT, cy=CY, k=K)
                    
                    # Send command to robot
                    message = controller.compute_command(v_cmd, 0.0, w_cmd)
                    robot.send_message(message)
                    
                    # Calculate tracking error
                    error_x = x_ref - x
                    error_y = y_ref - y
                    error_theta = wrap_angle(theta_ref - theta)
                    distance_error = math.sqrt(error_x**2 + error_y**2)
                    
                    # Store reference position for vision display
                    vision.set_reference_position(x_ref, y_ref, theta_ref)
                    
                    # Display status
                    print(f"t:{elapsed_time:5.1f}s | "
                          f"Pos:({x:+.3f},{y:+.3f}) θ:{math.degrees(theta):6.1f}° | "
                          f"Ref:({x_ref:+.3f},{y_ref:+.3f}) θ:{math.degrees(theta_ref):6.1f}° | "
                          f"Err:{distance_error:.3f}m {math.degrees(error_theta):+6.1f}° | "
                          f"Cmd: v:{v_cmd:+.2f} ω:{w_cmd:+.2f}", 
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
