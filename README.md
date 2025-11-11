# Tortugos - Bluetooth Robot Controller

Python controller for managing multiple Bluetooth-connected robots.

## Setup

1. Pair your Bluetooth device with your system
2. Bind the device to a serial port:
   ```bash
   ./connect_robot.sh <MAC_ADDRESS> <RFCOMM_PORT>
   ```
   Example:
   ```bash
   ./connect_robot.sh 98:D3:32:20:28:46 /dev/rfcomm0
   ```

3. Run the controller:
   ```bash
   python main.py
   ```

## Project Structure

- `robot.py` - Robot class for Bluetooth communication
- `main.py` - Main control script
- `connect_robot.sh` - Script to bind Bluetooth devices to serial ports

## Robot Configuration

Current robots:
- **RAM06**: MAC `98:D3:32:20:28:46`, Port `/dev/rfcomm0`

## Requirements

- Python 3.x
- pyserial
- rfcomm (Linux Bluetooth utilities)
