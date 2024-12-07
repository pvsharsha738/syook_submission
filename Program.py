import time
import numpy as np
from bluepy.btle import Scanner, DefaultDelegate, Peripheral

# Define a motion detection threshold
MOVEMENT_THRESHOLD = 1.0  # Adjust this value based on the sensitivity required

class MyDelegate(DefaultDelegate):
    def __init__(self, mac_address):
        DefaultDelegate.__init__(self)
        self.mac_address = mac_address
        self.accel_data = None

    def handleNotification(self, cHandle, data):
        """
        This method is called when a notification is received from the BLE device.
        Here, we're assuming the data contains the accelerometer data.
        """
        # Extract 3-axis acceleration data (this will depend on the data format of your BLE tag)
        # Assuming data comes in the form of 6 bytes for X, Y, Z (2 bytes per axis)
        x = int.from_bytes(data[0:2], byteorder='little', signed=True)
        y = int.from_bytes(data[2:4], byteorder='little', signed=True)
        z = int.from_bytes(data[4:6], byteorder='little', signed=True)
        # Save the accelerometer data
        self.accel_data = np.array([x, y, z])

def calculate_magnitude(accel_data):
    """Calculate the magnitude of the acceleration vector."""
    return np.linalg.norm(accel_data)

def detect_motion(accel_data):
    """Detect if the object is moving based on the acceleration vector's magnitude."""
    magnitude = calculate_magnitude(accel_data)
    if magnitude > MOVEMENT_THRESHOLD:
        return "Moving"
    else:
        return "Stationary"

def scan_and_connect():
    """Scan for nearby BLE devices and connect to the target device."""
    scanner = Scanner()
    devices = scanner.scan(10.0)  # Scan for 10 seconds
    target_device = None
    # Find the device with the desired MAC address (replace with your device's address)
    target_mac = "XX:XX:XX:XX:XX:XX"  # Replace with your BLE device MAC address
    for dev in devices:
        if dev.addr == target_mac.lower():
            target_device = dev
            break
    if target_device is None:
        print("Device not found")
        return None
    print(f"Found device: {target_device.addr}")
    peripheral = Peripheral(target_device)
    peripheral.setDelegate(MyDelegate(target_device.addr))
    
    return peripheral

def main():
    peripheral = scan_and_connect()
    if peripheral is None:
        return
    print("Connected to device. Waiting for data...")
    
    try:
        while True:
            if peripheral.waitForNotifications(1.0):
                # Handle notifications
                pass
            if peripheral.delegate.accel_data is not None:
                # Process accelerometer data
                accel_data = peripheral.delegate.accel_data
                print(f"Acceleration Data: {accel_data}")
                status = detect_motion(accel_data)
                print(f"Device status: {status}")
    except KeyboardInterrupt:
        print("Terminating...")
    finally:
        peripheral.disconnect()

if __name__ == "__main__":
    main()
