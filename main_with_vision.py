import time
from robot import Robot
from unicycle_controller import UnicycleController
from keyboard_controller import KeyboardController
from vision_tracker import VisionTracker


if __name__ == "__main__":
    # Robot physical parameters
    WHEEL_RADIUS = 0.06471 / 2  # meters
    WHEEL_BASE = -0.07782        # meters (negative indicates sign convention)
    UPDATE_RATE = 0.05           # seconds (20 Hz)
    
    # Vision tracking parameters
    AREA_WIDTH = 2.0   # meters
    AREA_HEIGHT = 1.5  # meters
    
    # Robot configuration
    robot = Robot(
        name="RAM06",
        mac_address="98:D3:32:20:28:46",
        rfcomm_port="/dev/rfcomm0"
    )
    
    # Controllers
    unicycle = UnicycleController(
        wheel_base=abs(WHEEL_BASE),
        wheel_radius=WHEEL_RADIUS
    )
    keyboard = KeyboardController(acceleration=0.4, deceleration=0.2, max_speed=10.0)
    
    # Vision tracker
    vision = VisionTracker(
        camera_id=4,  # Logitech HD Pro Webcam C920
        area_width_m=AREA_WIDTH,
        area_height_m=AREA_HEIGHT
    )
    
    print(f"Connecting to {robot.name}...")
    
    if robot.connect():
        print("Starting vision tracker...")
        if vision.start():
            keyboard.print_help()
            keyboard.start()
            
            try:
                last_update = time.time()
                
                while True:
                    # Continuously check for key presses
                    keyboard.check_keys()
                    
                    # Check if enough time has passed since last update
                    current_time = time.time()
                    if current_time - last_update >= UPDATE_RATE:
                        # Get velocity based on all keys pressed during this window
                        vx, vy = keyboard.get_velocity()
                        
                        # Compute robot command (direction mode)
                        command = unicycle.compute_command_from_direction(vx, vy, scale=1.0)
                        
                        # Get robot position from vision
                        x, y, detected = vision.get_robot_position(show_debug=True)
                        
                        # Debug output
                        if detected:
                            print(f"vx={vx:.2f}, vy={vy:.2f} | Pos: X={x:.3f}m Y={y:.3f}m | cmd={command}     ", end='\r')
                        else:
                            print(f"vx={vx:.2f}, vy={vy:.2f} | Pos: NOT DETECTED | cmd={command}     ", end='\r')
                        
                        # Send to robot
                        if not robot.send_message(command):
                            print("Failed to send. Connection may be lost.")
                            if not robot.reconnect():
                                print("Failed to reconnect. Exiting.")
                                break
                        
                        last_update = current_time
            
            except KeyboardInterrupt:
                print("\nStopping robot...")
            finally:
                keyboard.stop()
                vision.stop()
                print("Sending stop command...")
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
