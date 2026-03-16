import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib.animation import FuncAnimation

import math
import numpy as np
from pyquaternion import Quaternion

import serial
import re
import time
from threading import Thread

class AttitudePlotter:
    def __init__(self, serial_port='/dev/ttyACM1', baud_rate=9600):
        self.serial_port = serial_port
        self.baud_rate = baud_rate
        self.serial_ = None

        self.q = Quaternion(axis=[1, 0, 0], angle=np.pi/4)  # default orientation
        self.data_list = []                                 # List to store received quaternion data for plotting

    def setup_serial(self):
        try:
            self.serial_ = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            time.sleep(2)                                   # Wait for Arduino to reset
            print(f"Serial port {self.serial_port} opened successfully.")
        except serial.SerialException as e:
            print(f"Error opening serial port {self.serial_port}: {e}")
            self.serial_ = None

        # # If self.serial_ is valid start a new thread to read serial data
        if self.serial_ is not None:
            serial_thread = Thread(target=self.run_serial_reader, daemon=True)
            serial_thread.start()
        #     serial_thread.join()                          # Wait for the serial thread to finish (it will run indefinitely until interrupted)

    def run_serial_reader(self):
        try:
            while True:
                self.receive_serial_data()
                time.sleep(0.001)                            # Adjust the sleep time as needed to control the reading frequency
        except KeyboardInterrupt:
            print("Serial reading interrupted by user.")
        except Exception as e:
            print(f"Error in serial reader: {e}")   

    def receive_serial_data(self):
        # Read attitude data from serial and update the local array
        input_q = [1, 0 , 0, 0]  # default quaternion
        try:
            line_bytes = self.serial_.readline()  # Read a line of bytes from the serial port

            # print(f"Raw bytes read from serial: {line_bytes}")  # Debug: print the raw bytes read from serial
            line = line_bytes.decode('utf-8').strip()  # decode the bytes and strip whitespace
            # print(f"Raw line read from serial: '{line}'")  # Debug: print the raw line read from serial
            if line:
                # print(f"Received line: {line}")  # Debug: print the raw line received
                vals = line.split(',')
                if len(vals) == 4:
                    try:
                        input_q = [float(val) for val in vals]
                        # print(f'Updated quaternion: {input_q}')
                    except ValueError:
                        print(f"Error converting quaternion values: {vals}")
                else:
                    print(f"Unexpected data format: {line}")
            else:
                print("No data received.")
        except serial.SerialException as e:
            print(f"Serial error: {e}")
        except KeyboardInterrupt:
            print("Serial reading interrupted by user.")
        except Exception as e:
            print(f"Error: {e}")

        # print(f'Received quaternion: {input_q}')
        # Quaternion dataframe from serial in format (w, x, y, z) for pyquaternion
        q_recved = Quaternion(q1=input_q[0], q2=input_q[1], q3=input_q[2], q4=input_q[3])  # Update the quaternion with the received values

        self.data_list.append(q_recved)


    def update_plot(self, frame):
        ''' Animation function callback to plot the cube '''

        # Update the data list queue with the latest quaternion
        if len(self.data_list) > 10:  # Limit the size of the data
            self.data_list = self.data_list[-10:]  # Keep only the last 10 quaternions

        if self.data_list:
            self.q = self.data_list[-1]  # Update the current quaternion to the latest received

        q_x = self.q.x
        q_y = self.q.y
        q_z = self.q.z
        q_w = self.q.w

        # print(f"Current quaternion: w={q_w}, x={q_x}, y={q_y}, z={q_z}")  # Print the current quaternion for debugging
        euler_x, euler_y, euler_z = self.quaternion_to_euler(q_x, q_y, q_z, q_w)  # Get the Euler angles in degrees for debugging
        # print(f"Euler angles (degrees): {theta_angles * (180.0 / np.pi)}")  # Print the Euler angles in degrees for debugging
        print(f"Current euler angles (degrees): {euler_x}, {euler_y}, {euler_z}")  # Print the Euler angles in degrees for debugging

        # Clear the plot and set limits
        ax.clear()
        ax.set_xlim(-1, 1)
        ax.set_ylim(1, -1)
        ax.set_zlim(-1, 1)
        ax.invert_zaxis()
        ax.invert_yaxis()
        ax.set_xlabel("x label")
        ax.set_ylabel("y label")
        ax.set_zlabel("z label")
        # plot the cube
        self.plot_cube_quaternion('gray')

    def plot_cube_quaternion(self, color):
        ''' Plot a cube rotated by the given quaternion '''
        v = np.array([[ 1, 0, 0], [ -0.2, -1, -0.5],
                    [ -0.2, 1, -0.5],  [ 0,0,0], [ -0.2, 0, 0.3]]).transpose()

        v_rot = np.zeros((3,5))
        for i in range(5):
            v_rot[:,i] = self.q.rotate(v[:,i])

        v_rot = v_rot.transpose()
        verts = [ [v_rot[0],v_rot[1],v_rot[3]], [v_rot[0],v_rot[2],v_rot[3]], [v_rot[0],v_rot[3],v_rot[4]]]
        ax.add_collection3d(Poly3DCollection(verts, facecolors= color, linewidths=1, edgecolors='b', alpha=1))

    def close(self):
        ''' Close the serial connection '''
        if self.serial_ is not None and self.serial_.is_open:
            self.serial_.flush()
            self.serial_.close()
            print(f"Serial port {self.serial_port} closed.")

    # Source - https://stackoverflow.com/q/53033620
    # Posted by Amir, modified by community. See post 'Timeline' for change history
    # Retrieved 2026-03-15, License - CC BY-SA 4.0

    def quaternion_to_euler(self, x, y, z, w):
        ''' Convert quaternion (x, y, z, w) to Euler angles (X, Y, Z) in degrees '''
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        X = math.degrees(math.atan2(t0, t1))

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        Y = math.degrees(math.asin(t2))

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        Z = math.degrees(math.atan2(t3, t4))

        return X, Y, Z


if __name__ == "__main__":
    print("Starting serial reader and plotter...")
    # # Create a figure with 3D axes
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    ax.set_aspect('auto')

    attitude_plotter = AttitudePlotter(serial_port='/dev/ttyACM1', baud_rate=9600)
    attitude_plotter.setup_serial()  # Initialize the serial connection

    # # Define an animation callback to update the plot
    try:
        anim = FuncAnimation(fig, attitude_plotter.update_plot, frames=200, interval=200)
        plt.show()
    except KeyboardInterrupt:
        print("Animation interrupted by user.")

    attitude_plotter.close()
    print("Attitude plotter closed")