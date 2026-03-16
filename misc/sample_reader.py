import serial
import time

# Configure serial port
ser = serial.Serial('/dev/ttyACM1', 9600, timeout=1)
time.sleep(2)  # Wait for Arduino to reset

print("Reading three numbers from Arduino...\n")

try:
    while True:
        # Read line from serial
        line = ser.readline().decode('utf-8').strip()
        
        print(f"Raw line read from serial: '{line}'")  # Debug: print the raw line read from serial
        if line:
            # Split by comma and convert to integers
            numbers = line.split(',')
            if len(numbers) == 3:
                num1, num2, num3 = int(numbers[0]), int(numbers[1]), int(numbers[2])
                print(f"Received: {num1}, {num2}, {num3}")
            
except KeyboardInterrupt:
    print("\nStopped")
    ser.close()