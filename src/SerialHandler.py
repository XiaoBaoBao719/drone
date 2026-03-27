import serial
import serial.tools.list_ports
import time
import sys
import argparse

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
        if(self.serial_):
            self.connected = False
        
        
    def get_port_name(self, ports) -> str | None:
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
        if self.ports_list:
            print("Avaiable serial ports")
            for p in self.ports_list:
                print(f" {p}")
        else:
            print("No serial ports found.")

    def list_serial_ports(self) -> list[str]:
        """ Return a sorted list of available serial port names. """
        return sorted(p.device for p in serial.tools.list_ports.comports())

    def auto_detect_port(self) -> str | None:
        """Return the first available serial port, or None if none found."""
        ports = self.list_serial_ports()
        return ports[0] if ports else None

