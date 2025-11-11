import serial
import time


class Robot:
    """Bluetooth robot controller"""
    
    def __init__(self, name, mac_address, rfcomm_port="/dev/rfcomm0", baudrate=9600):
        self.name = name
        self.mac_address = mac_address
        self.rfcomm_port = rfcomm_port
        self.baudrate = baudrate
        self.serial = None
        self.timeout = 1
    
    def connect(self):
        """Connect to the robot via Bluetooth"""
        try:
            self.serial = serial.Serial(
                self.rfcomm_port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                write_timeout=self.timeout,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False
            )
            time.sleep(2)
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()
            print(f"[{self.name}] Connected to {self.rfcomm_port} at {self.baudrate} baud")
            return True
        except serial.SerialException as e:
            print(f"[{self.name}] Error connecting: {e}")
            return False
    
    def send_message(self, message):
        """Send a message to the robot"""
        try:
            if self.serial and self.serial.is_open:
                self.serial.write(message.encode())
                self.serial.flush()
                return True
        except serial.SerialException as e:
            print(f"[{self.name}] Error sending message: {e}")
        return False
    
    def is_connected(self):
        """Check if robot is connected"""
        try:
            return self.serial and self.serial.is_open
        except:
            return False
    
    def reconnect(self, max_attempts=3):
        """Attempt to reconnect to the robot"""
        for attempt in range(max_attempts):
            print(f"[{self.name}] Reconnection attempt {attempt + 1}/{max_attempts}...")
            if self.connect():
                return True
            time.sleep(1)
        return False
    
    def disconnect(self):
        """Disconnect from the robot"""
        if self.serial and self.serial.is_open:
            self.serial.close()
            print(f"[{self.name}] Disconnected")
