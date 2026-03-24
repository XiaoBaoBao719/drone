import numpy as np
from scipy.integrate import odeint
import serial
import json
import struct
from datetime import datetime
import time
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib.animation import FuncAnimation
from pyquaternion import Quaternion
import math

class QuadcopterDynamics:
    def __init__(self):
        # Physical parameters (match your DroneController.h)
        self.mass = 1.2  # kg
        self.L = 0.225   # arm length (m)
        self.Ixx = 0.0069
        self.Iyy = 0.0069
        self.Izz = 0.0132
        self.kT = 8.54e-6  # thrust coefficient
        self.kD = 1.66e-7  # drag coefficient
        self.g = 9.81

        # State: [x, y, z, vx, vy, vz, roll, pitch, yaw, p, q, r]
        self.state = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], dtype=float)

    def motor_thrust(self, pwm):
        """Convert PWM (1000-2000 Î¼s) to thrust force"""
        # Normalize: 1000 â†’ 0, 2000 â†’ 1
        normalized = (pwm - 1000) / 1000
        normalized = np.clip(normalized, 0, 1)
        return self.kT * (normalized ** 2)  # Quadratic relationship

    def dynamics(self, state, pwm_commands):
        """
        Calculate state derivatives using quadcopter dynamics equations
        pwm_commands: [motor1, motor2, motor3, motor4] in Î¼s
        """
        x, y, z, vx, vy, vz, roll, pitch, yaw, p, q, r = state

        # Motor thrusts
        thrusts = np.array([self.motor_thrust(pwm) for pwm in pwm_commands])
        total_thrust = np.sum(thrusts)

        # Torques (X-quad configuration)
        # tau_roll = L * (T4 - T2)
        # tau_pitch = L * (T1 - T3)
        # tau_yaw = kD * (T1 - T2 + T3 - T4)
        tau_roll = self.L * (thrusts[3] - thrusts[1])
        tau_pitch = self.L * (thrusts[0] - thrusts[2])
        tau_yaw = self.kD * (thrusts[0] - thrusts[1] + thrusts[2] - thrusts[3])

        # Linear acceleration (Newton's second law)
        cos_roll = np.cos(roll)
        sin_roll = np.sin(roll)
        cos_pitch = np.cos(pitch)
        sin_pitch = np.sin(pitch)

        ax = (1 / self.mass) * (total_thrust * (sin_roll * np.sin(yaw) + cos_roll * sin_pitch * np.cos(yaw)))
        ay = (1 / self.mass) * (total_thrust * (cos_roll * sin_pitch * np.sin(yaw) - sin_roll * np.cos(yaw)))
        az = (1 / self.mass) * (total_thrust * cos_roll * cos_pitch) - self.g

        # Angular acceleration (Euler's equations)
        p_dot = (tau_roll + (self.Iyy - self.Izz) * q * r) / self.Ixx
        q_dot = (tau_pitch + (self.Izz - self.Ixx) * p * r) / self.Iyy
        r_dot = (tau_yaw + (self.Ixx - self.Iyy) * p * q) / self.Izz

        # Angular rates to Euler angle rates
        tan_pitch = np.tan(pitch)
        sec_pitch = 1 / np.cos(pitch)

        roll_dot = p + q * sin_roll * tan_pitch + r * cos_roll * tan_pitch
        pitch_dot = q * cos_roll - r * sin_roll
        yaw_dot = q * sin_roll * sec_pitch + r * cos_roll * sec_pitch

        return np.array([vx, vy, vz, ax, ay, az,
                        roll_dot, pitch_dot, yaw_dot,
                        p_dot, q_dot, r_dot])

    def step(self, pwm_commands, dt):
        """Integrate dynamics for time step dt"""
        new_state = self.state + self.dynamics(self.state, pwm_commands) * dt
        self.state = new_state
        return self.state

    def get_imu_data(self, noise=True):
        """Generate IMU readings (accelerometer + gyroscope + magnetometer)"""
        x, y, z, vx, vy, vz, roll, pitch, yaw, p, q, r = self.state

        # Accelerometer (world frame accel + gravity)
        g_vec = np.array([
            self.g * np.sin(pitch),
            -self.g * np.sin(roll) * np.cos(pitch),
            -self.g * np.cos(roll) * np.cos(pitch)
        ])

        # Gravity-free body frame acceleration
        body_accel = np.array([
            (vx - vx) / 0.004,  # approximate derivative
            (vy - vy) / 0.004,
            (vz - vz) / 0.004
        ])

        accel = body_accel + g_vec
        if noise:
            accel += np.random.normal(0, 0.1, 3)

        # Gyroscope
        gyro = np.array([p, q, r])
        if noise:
            gyro += np.random.normal(0, 0.01, 3)

        # Magnetometer (yaw reference)
        mag = np.array([
            np.cos(yaw),
            np.sin(yaw),
            0
        ])
        if noise:
            mag += np.random.normal(0, 0.05, 3)

        return {
            'accel': accel,
            'gyro': gyro,
            'mag': mag
        }

class SITLPlotter:
    def __init__(self):
        self.fig = plt.figure(figsize=(12, 8))
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_aspect('auto')
        
        # Data storage
        self.positions = []  # List of (x, y, z) positions
        self.orientations = []  # List of quaternions
        self.trajectory = []  # Full trajectory for plotting
        
        # Plot elements
        self.drone_plot = None
        self.trajectory_plot = None
        
    def euler_to_quaternion(self, roll, pitch, yaw):
        """Convert Euler angles (radians) to quaternion"""
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)

        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy

        return Quaternion(w, x, y, z)
    
    def update_data(self, state):
        """Update with new state data"""
        x, y, z, vx, vy, vz, roll, pitch, yaw, p, q, r = state
        
        # Convert Euler angles to quaternion
        q = self.euler_to_quaternion(roll, pitch, yaw)
        
        self.positions.append((x, y, z))
        self.orientations.append(q)
        self.trajectory.append((x, y, z))
        
        # Keep only recent data for performance
        if len(self.positions) > 100:
            self.positions = self.positions[-100:]
            self.orientations = self.orientations[-100:]
    
    def plot_drone(self, position, quaternion, color='red'):
        """Plot drone as a 3D quadcopter model"""
        x, y, z = position
        
        # Define quadcopter vertices (simple cross shape)
        # Arms along X and Y axes
        arm_length = 0.3
        vertices = np.array([
            # Main body (center)
            [0, 0, 0],
            # Arms
            [arm_length, 0, 0], [-arm_length, 0, 0],  # X arms
            [0, arm_length, 0], [0, -arm_length, 0],  # Y arms
            # Propellers (slightly above arms)
            [arm_length, 0, 0.05], [-arm_length, 0, 0.05],
            [0, arm_length, 0.05], [0, -arm_length, 0.05]
        ]).T
        
        # Rotate vertices by quaternion
        rotated_vertices = np.zeros_like(vertices)
        for i in range(vertices.shape[1]):
            vec = vertices[:, i]
            rotated_vec = quaternion.rotate(vec)
            rotated_vertices[:, i] = rotated_vec
        
        # Translate to position
        rotated_vertices[0] += x
        rotated_vertices[1] += y  
        rotated_vertices[2] += z
        
        rotated_vertices = rotated_vertices.T
        
        # Define faces (simple lines for arms and propellers)
        faces = [
            # Arms
            [rotated_vertices[0], rotated_vertices[1]],  # X arm
            [rotated_vertices[0], rotated_vertices[3]],  # Y arm front
            [rotated_vertices[0], rotated_vertices[4]],  # Y arm back
            # Propellers
            [rotated_vertices[5], rotated_vertices[6]],  # X prop
            [rotated_vertices[7], rotated_vertices[8]]   # Y prop
        ]
        
        if self.drone_plot:
            self.drone_plot.remove()
        
        self.drone_plot = self.ax.add_collection3d(Poly3DCollection(
            faces, facecolors=color, linewidths=2, edgecolors='black', alpha=0.8
        ))
    
    def update_plot(self, frame):
        """Animation update function"""
        self.ax.clear()
        
        # Set plot limits based on trajectory
        if self.trajectory:
            traj = np.array(self.trajectory)
            x_min, x_max = traj[:, 0].min() - 1, traj[:, 0].max() + 1
            y_min, y_max = traj[:, 1].min() - 1, traj[:, 1].max() + 1
            z_min, z_max = traj[:, 2].min() - 0.5, traj[:, 2].max() + 0.5
            
            self.ax.set_xlim(x_min, x_max)
            self.ax.set_ylim(y_min, y_max)
            self.ax.set_zlim(z_min, z_max)
        else:
            self.ax.set_xlim(-2, 2)
            self.ax.set_ylim(-2, 2)
            self.ax.set_zlim(-1, 1)
        
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_zlabel('Z (m)')
        self.ax.set_title('Drone SITL Simulation')
        
        # Plot trajectory
        if len(self.trajectory) > 1:
            traj = np.array(self.trajectory)
            self.trajectory_plot = self.ax.plot3D(
                traj[:, 0], traj[:, 1], traj[:, 2], 
                'b-', alpha=0.7, linewidth=1, label='Trajectory'
            )[0]
        
        # Plot current drone position and orientation
        if self.positions and self.orientations:
            current_pos = self.positions[-1]
            current_ori = self.orientations[-1]
            self.plot_drone(current_pos, current_ori, 'red')
            
            # Plot starting point
            if len(self.trajectory) > 1:
                start_pos = self.trajectory[0]
                self.ax.scatter(*start_pos, color='green', s=50, label='Start')
                self.ax.scatter(*current_pos, color='red', s=50, label='Current')
        
        self.ax.legend()
        self.ax.grid(True)
        
        return self.drone_plot, self.trajectory_plot

class SerialBridge:
    """Handles communication with Arduino over serial"""

    def __init__(self, port='COM3', baudrate=115200, timeout=0.1, serial_port=None):
        if serial_port is not None:
            self.serial = serial_port
        else:
            self.serial = serial.Serial(port, baudrate, timeout=timeout)
        self.pwm_commands = [1500, 1500, 1500, 1500]  # neutral

    def read_pwm(self):
        """Read PWM values from Arduino"""
        if self.serial.in_waiting >= 8:  # Expect 4Ã—2-byte PWM values
            try:
                data = self.serial.read(8)
                self.pwm_commands = struct.unpack('<HHHH', data)
                return True
            except:
                return False
        return False

    def send_imu(self, imu_data):
        """Send IMU data to Arduino as JSON"""
        packet = json.dumps({
            'accel': imu_data['accel'].tolist(),
            'gyro': imu_data['gyro'].tolist(),
            'mag': imu_data['mag'].tolist(),
            'timestamp_ms': int(datetime.now().timestamp() * 1000)
        }).encode()
        self.serial.write(packet + b'\n')

class SITLSimulator:
    def __init__(self, port='COM3', serial_port=None):
        self.dynamics = QuadcopterDynamics()
        self.bridge = SerialBridge(port, serial_port=serial_port)
        self.dt = 0.004  # 250 Hz
        self.log = []
        self.plotter = SITLPlotter()
        
    def run(self, duration_sec=60, show_plot=True):
        start_time = datetime.now()
        step_count = 0
        
        if show_plot:
            # Set up animation
            self.anim = FuncAnimation(self.plotter.fig, self.plotter.update_plot, 
                                      interval=100, blit=False)

        while (datetime.now() - start_time).total_seconds() < duration_sec:
            # Read PWM from external controller (keyboard app)
            self.bridge.read_pwm()

            # Simulate one step
            self.dynamics.step(self.bridge.pwm_commands, self.dt)

            # Update plotter with current state
            self.plotter.update_data(self.dynamics.state)

            # Generate and send IMU data
            imu_data = self.dynamics.get_imu_data()
            self.bridge.send_imu(imu_data)

            # Log state + IMU + PWM
            self.log.append({
                'step': step_count,
                'state': self.dynamics.state.copy(),
                'pwm': self.bridge.pwm_commands.copy(),
                'imu': {k: v.tolist() for k, v in imu_data.items()}
            })

            step_count += 1
            time.sleep(self.dt)

            if show_plot and step_count % 10 == 0:
                plt.pause(0.001)

        if show_plot:
            plt.show()

    def save_log(self, filename='sitl_log.json'):
        with open(filename, 'w') as f:
            json.dump(self.log, f, indent=2)

if __name__ == '__main__':
    sim = SITLSimulator('COM3')  # Adjust port for your setup
    sim.run(duration_sec=30, show_plot=True)
    sim.save_log()
