import serial
import serial.tools.list_ports
import time
import sys
import argparse
from typing import Optional, List

class SerialHandler():
    PORT_DEFAULT = "/dev/ttyAM0"
    BAUD_DEFAULT = 9600
    def __init__(self, port=PORT_DEFAULT, baud_rate=BAUD_DEFAULT, timeout=2.0):
        self.port = port
        self.baud_rate = baud_rate
        self.serial_timeout = timeout
        self.connected  = False
        self.max_number_attempts = 6

        if self.port is None:
            print(f"No serial port provided. Attempting auto-connection...")
            
            ### List ports and attempt to auto-connect to first one if needed ###
            self.port = self.auto_detect_port()
            if self.port is None:
                print(f"Error: No serial ports detected. Connect your MCU or specifiy an exact port.")
                self.print_serial_ports()

        ### Attempt to open the port ###
        self.serial_ = self.open(self.port)

        num_attempts = self.max_number_attempts
        try:
            while self.serial_ == -1:
                if (num_attempts == 0):
                    print(f"Max number of port connection attempts reached. Goodbye")
                    break
                else:
                    num_attempts -= 1
                ### Prompt user if they would like to attempt auto-connection
                print(f"Couldn't open port{self.port}!")
                answer = input("Attempt port auto-detection? [Y/n]: ").strip().lower()
                if answer not in ("y","yes"):
                    ports = self.list_serial_ports()
                    print("User declined port auto-detection\n")

                    while(self.port is None):
                        self.port = self.get_port_name(ports=ports)
                else:
                    self.port = self.auto_detect_port()

                ### Attempt to open the port and create a serial connection ###
                self.serial_ = self.open(self.port)
                
        except KeyboardInterrupt:
            print(f"\n\n Serial connection interrupted by user. Goodbye.")
            self.port = None
            self.serial_ = -1

        ### Finally, if we have a successful connection, toggle flag ###
        if(self.serial_ != -1 ):
            self.connected = True
        
        
    def get_port_name(self, ports) -> Optional[str]:
        print("Avaiable ports:")
        for i, p in enumerate(ports):
            print(f" [{i}] {p}")
            user_index = input("Please select a number to connect to that port: ").strip()
                    
            # Check for user-input to make sure they entered a valid index number
            try:
                port_attempt = ports[int(user_index)]
            except (ValueError, IndexError):
                print("Invalid selection. Please try again")
                return None

        return port_attempt


    def open_serial(self, port:str, baud:int, timeout:float = 2.0) -> serial.Serial:
        """Open and return a serial connection, printing the status."""
        print(f"\Connecting to {port} at {baud} ...", end="", flush=True)

        ser = serial.Serial(port, baud, timeout=timeout)
        # Give MCU time to reset after a pulse
        time.sleep(2.0)
        ser.reset_input_buffer()
        print("Connected!\n")
        return ser

    def open(self, port):
        """Attempt to open a requested port. Return -1 if there was an issue."""
        try:
            serial_connection = self.open_serial(port=self.port, baud=self.baud_rate, 
                                            timeout=self.serial_timeout)
        except serial.SerialException as exec:
            print(f"ERROR: Could not open {port}: {exec}")
            return -1
        
        if serial_connection:
            return serial_connection

    
    def print_serial_ports(self):
        """ Print a sorted list of all available serial port names."""
        print(f"+++ DISPLAYING SERIAL PORTS LIST +++\n")
        ports = self.list_serial_ports()
        if ports:
            print("Available serial ports")
            for p in ports:
                print(f" {p}")
        else:
            print("No serial ports found.")

    def list_serial_ports(self) -> List[str]:
        """ Return a sorted list of available serial port names. """
        return sorted(p.device for p in serial.tools.list_ports.comports())

    def auto_detect_port(self) -> Optional[str]:
        """Return the first available serial port, or None if none found."""
        ports = self.list_serial_ports()
        return ports[0] if ports else None

if __name__ == "__main__":
    # Test script for SerialHandler class
    
    print("=== Testing SerialHandler Class ===\n")
    
    # Test 1: List available serial ports
    print("Test 1: Listing available serial ports")
    handler = SerialHandler()
    ports = handler.list_serial_ports()
    print(f"Available ports: {ports}")
    handler.print_serial_ports()
    print()
    
    # Test 2: Auto-detect port
    print("Test 2: Auto-detecting port")
    detected_port = handler.auto_detect_port()
    if detected_port:
        print(f"Auto-detected port: {detected_port}")
    else:
        print("No ports auto-detected")
    print()
    
    # Test 3: Attempt connection (this will try to connect if ports are available)
    print("Test 3: Attempting connection")
    if ports:
        # Create a new handler with a specific port for testing
        test_handler = SerialHandler(port=ports[0], baud_rate=9600)
        if test_handler.connected:
            print("Successfully connected!")
            # Note: In a real test, you might want to send/receive data here
            # For now, just check connection status
        else:
            print("Connection failed (expected if no hardware connected)")
    else:
        print("No ports available for connection test")
    
    print("\n=== Test Complete ===")