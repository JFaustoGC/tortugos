import time
from robot import Robot


if __name__ == "__main__":
    # Robot configuration
    robot = Robot(
        name="RAM06",
        mac_address="98:D3:32:20:28:46",
        rfcomm_port="/dev/rfcomm0"
    )
    
    MOVE_MESSAGE = "IR+01.00L+01.00F\n"
    STOP_MESSAGE = "IR+00.00L+00.00F\n"
    
    print(f"Connecting to {robot.name}...")
    
    if robot.connect():
        print(f"Sending movement commands continuously...")
        print("Press Ctrl+C to stop.\n")
        
        try:
            while True:
                # Check connection health
                if not robot.is_connected():
                    print("Connection lost! Attempting to reconnect...")
                    if not robot.reconnect():
                        print("Failed to reconnect. Exiting.")
                        break
                    print("Reconnected successfully!\n")
                
                # Send movement command
                if not robot.send_message(MOVE_MESSAGE):
                    print("Failed to send. Connection may be lost.")
                
                time.sleep(0.1)
        
        except KeyboardInterrupt:
            print("\nStopping robot...")
        finally:
            print("Sending stop command...")
            robot.send_message(STOP_MESSAGE)
            time.sleep(0.5)
            robot.disconnect()
    else:
        print(f"\nFailed to connect to {robot.name}")
        print("Make sure:")
        print(f"1. Bluetooth device is paired: {robot.mac_address}")
        print(f"2. Run: sudo rfcomm bind {robot.rfcomm_port} {robot.mac_address}")