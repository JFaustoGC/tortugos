#!/bin/bash
# Script to connect Bluetooth robots

if [ -z "$1" ] || [ -z "$2" ]; then
    echo "Usage: $0 <mac_address> <rfcomm_port>"
    echo "Example: $0 98:D3:32:20:28:46 /dev/rfcomm0"
    exit 1
fi

MAC_ADDRESS="$1"
RFCOMM_PORT="$2"

echo "Connecting to Bluetooth device $MAC_ADDRESS on $RFCOMM_PORT..."

# Release any existing binding
sudo rfcomm release "$RFCOMM_PORT" 2>/dev/null

# Bind the device
sudo rfcomm bind "$RFCOMM_PORT" "$MAC_ADDRESS"

if [ $? -eq 0 ]; then
    echo "Successfully bound $MAC_ADDRESS to $RFCOMM_PORT"
else
    echo "Failed to bind device. Make sure Bluetooth is enabled and device is paired."
    exit 1
fi
