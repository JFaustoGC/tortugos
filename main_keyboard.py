import time
from robot import Robot
from unicycle_controller import UnicycleController
from keyboard_controller import KeyboardController


if __name__ == "__main__":
    # Robot physical parameters
    WHEEL_RADIUS = 0.06471 / 2  # meters
    WHEEL_BASE = -0.07782        # meters (negative indicates sign convention)
    UPDATE_RATE = 0.05           # seconds (20 Hz)
    
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
    
    # Controllers
    unicycle = UnicycleController(
        wheel_base=abs(WHEEL_BASE),  # Use absolute value for calculations
        wheel_radius=WHEEL_RADIUS
    )
    keyboard = KeyboardController(acceleration=0.4, deceleration=0.2, max_speed=10.0)
    
    print(f"Connecting to {ram_06.name}...")
    print(f"Connecting to {ram_05.name}...")
    
    if ram_06.connect() and ram_05.connect():
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
                    
                    # Debug output
                    print(f"vx={vx:.2f}, vy={vy:.2f} | cmd={command}     ", end='\r')
                    
                    # Send to robot
                    if not ram_06.send_message(command) or not ram_05.send_message(command):
                        print("Failed to send. Connection may be lost.")
                        if not ram_06.reconnect() or not ram_05.reconnect():
                            print("Failed to reconnect. Exiting.")
                            break
                    
                    last_update = current_time
        
        except KeyboardInterrupt:
            print("\nStopping robot...")
        finally:
            keyboard.stop()
            print("Sending stop command...")
            ram_06.send_message(unicycle.stop_command())
            time.sleep(0.5)
            ram_06.disconnect()
    else:
        print(f"\nFailed to connect to {ram_06.name}")
        print("Make sure:")
        print(f"1. Bluetooth device is paired: {ram_06.mac_address}")
        print(f"2. Run: sudo rfcomm bind {ram_06.rfcomm_port} {ram_06.mac_address}")
