import serial
import struct
import json
import time
import math
import threading
import msvcrt

class FakeSerial:
    def __init__(self, timeout=0.05):
        self._rx = []
        self._lock = threading.Lock()
        self.peer = None
        self.timeout = timeout

    def set_peer(self, peer):
        self.peer = peer

    @property
    def in_waiting(self):
        with self._lock:
            return sum(len(x) for x in self._rx)

    def write(self, data):
        if self.peer is not None:
            with self.peer._lock:
                self.peer._rx.append(data)
            return len(data)
        return 0

    def readline(self):
        deadline = time.time() + (self.timeout if self.timeout is not None else 0)
        buf = b''
        while True:
            if self._rx:
                with self._lock:
                    chunk = self._rx.pop(0)
                buf += chunk
                if b'\n' in buf:
                    idx = buf.index(b'\n') + 1
                    line = buf[:idx]
                    remainder = buf[idx:]
                    if remainder:
                        with self._lock:
                            self._rx.insert(0, remainder)
                    return line
            if self.timeout is not None and time.time() > deadline:
                return buf
            time.sleep(0.001)

    def read(self, size=1):
        deadline = time.time() + (self.timeout if self.timeout is not None else 0)
        result = b''
        while len(result) < size:
            if self._rx:
                with self._lock:
                    chunk = self._rx.pop(0)
                need = size - len(result)
                result += chunk[:need]
                remainder = chunk[need:]
                if remainder:
                    with self._lock:
                        self._rx.insert(0, remainder)
            else:
                if self.timeout is not None and time.time() > deadline:
                    break
                time.sleep(0.001)
        return result

class KeyboardInput:
    def __init__(self):
        self.lock = threading.Lock()
        self.roll = 0.0  # filtered value sent to controller
        self.pitch = 0.0
        self.yaw = 0.0
        self.throttle = 1100.0

        self.target_roll = 0.0  # desired goal based on key commands
        self.target_pitch = 0.0
        self.target_yaw = 0.0
        self.target_throttle = 1100.0

        self.smoothing_factor = 0.15  # 0.0..1.0 (higher faster response)


    def process_key(self, key):
        if key == b'w':
            self.target_pitch += 5.0
        elif key == b's':
            self.target_pitch -= 5.0
        elif key == b'a':
            self.target_roll -= 5.0
        elif key == b'd':
            self.target_roll += 5.0
        elif key == b'q':
            self.target_throttle -= 10.0
        elif key == b'e':
            self.target_throttle += 10.0
        elif key == b'z':
            self.target_yaw -= 5.0
        elif key == b'c':
            self.target_yaw += 5.0

        # clamp targets
        self.target_roll = max(-150.0, min(150.0, self.target_roll))
        self.target_pitch = max(-150.0, min(150.0, self.target_pitch))
        self.target_yaw = max(-150.0, min(150.0, self.target_yaw))
        self.target_throttle = max(1000.0, min(2000.0, self.target_throttle))

    def update_filtered(self):
        # smooth transition from current to target
        self.roll += (self.target_roll - self.roll) * self.smoothing_factor
        self.pitch += (self.target_pitch - self.pitch) * self.smoothing_factor
        self.yaw += (self.target_yaw - self.yaw) * self.smoothing_factor
        self.throttle += (self.target_throttle - self.throttle) * self.smoothing_factor


    def monitor(self):
        while True:
            if msvcrt.kbhit():
                key = msvcrt.getch().lower()
                if key == b'x':
                    break
                with self.lock:
                    self.process_key(key)
            time.sleep(0.01)

    def get(self):
        with self.lock:
            return self.roll, self.pitch, self.yaw, self.throttle

class RateModeController:
    def __init__(self, port='COM3', baud=115200, serial_port=None):
        if serial_port is not None:
            self.serial = serial_port
        else:
            self.serial = serial.Serial(port, baud, timeout=0.05)
        self.input_mgr = KeyboardInput()

        # PID gains (from RateModeR4)
        self.PGainX = 1.67
        self.PGainY = 1.67
        self.PGainZ = 1.67
        self.IGainX = 0.0015
        self.IGainY = 0.0015
        self.IGainZ = 0.0012
        self.DGainX = 0.01
        self.DGainY = 0.01
        self.DGainZ = 0.01

        self.prev_error_x = 0.0
        self.prev_error_y = 0.0
        self.prev_error_z = 0.0
        self.prev_i_x = 0.0
        self.prev_i_y = 0.0
        self.prev_i_z = 0.0
        self.prev_setpt_x = 0.0
        self.prev_setpt_y = 0.0
        self.prev_setpt_z = 0.0

        self.angle_x = 0.0
        self.angle_y = 0.0
        self.angle_z = 0.0

        self.IMU_SAMPLE_FREQ_S = 0.004
        self.filter_degree = 8
        self.input_history = {
            'throttle': [0.0]*self.filter_degree,
            'roll': [0.0]*self.filter_degree,
            'pitch': [0.0]*self.filter_degree,
            'yaw': [0.0]*self.filter_degree,
        }
        self.lp_index = 0

    def pid(self, setpt, prev_setpt, err, prev_err, p_gain, i_gain, prev_i, d_gain):
        p_term = p_gain * err

        d_term = d_gain * (setpt - prev_setpt) / (self.IMU_SAMPLE_FREQ_S * 1000.0)

        i_dt = ((err + prev_err) / 2.0) * (self.IMU_SAMPLE_FREQ_S * 1000.0)
        i_term = prev_i + i_gain * i_dt
        i_term = max(-100.0, min(100.0, i_term))

        output = p_term + i_term - d_term
        output = max(-400.0, min(400.0, output))

        return output, i_term

    def run(self, duration_sec=60):
        print('Starting RateMode controller interface. Press x to exit.')

        key_thread = threading.Thread(target=self.input_mgr.monitor, daemon=True)
        key_thread.start()

        start_time = time.time()

        while (time.time() - start_time) < duration_sec:
            line = self.serial.readline().decode('utf-8', errors='ignore').strip()
            if not line:
                time.sleep(self.IMU_SAMPLE_FREQ_S)
                continue

            try:
                imu_json = json.loads(line)
            except json.JSONDecodeError:
                continue

            accel = imu_json.get('accel', [0,0,0])
            gyro = imu_json.get('gyro', [0,0,0])
            mag = imu_json.get('mag', [0,0,0])

            dt = self.IMU_SAMPLE_FREQ_S
            self.angle_x += gyro[0] * dt * (180.0 / math.pi)
            self.angle_y += gyro[1] * dt * (180.0 / math.pi)
            self.angle_z += gyro[2] * dt * (180.0 / math.pi)

            # read (smoothed) user input setpoints
            self.input_mgr.update_filtered()
            roll_sp, pitch_sp, yaw_sp, thr_sp = self.input_mgr.get()

            self.input_history['roll'][self.lp_index] = roll_sp
            self.input_history['pitch'][self.lp_index] = pitch_sp
            self.input_history['yaw'][self.lp_index] = yaw_sp
            self.input_history['throttle'][self.lp_index] = thr_sp
            self.lp_index = (self.lp_index + 1) % self.filter_degree

            filt_roll = sum(self.input_history['roll']) / self.filter_degree
            filt_pitch = sum(self.input_history['pitch']) / self.filter_degree
            filt_yaw = sum(self.input_history['yaw']) / self.filter_degree
            filt_thr = sum(self.input_history['throttle']) / self.filter_degree

            err_x = filt_roll - self.angle_x
            err_y = filt_pitch - self.angle_y
            err_z = filt_yaw - self.angle_z

            pid_x, self.prev_i_x = self.pid(filt_roll, self.prev_setpt_x, err_x, self.prev_error_x, self.PGainX, self.IGainX, self.prev_i_x, self.DGainX)
            pid_y, self.prev_i_y = self.pid(filt_pitch, self.prev_setpt_y, err_y, self.prev_error_y, self.PGainY, self.IGainY, self.prev_i_y, self.DGainY)
            pid_z, self.prev_i_z = self.pid(filt_yaw, self.prev_setpt_z, err_z, self.prev_error_z, self.PGainZ, self.IGainZ, self.prev_i_z, self.DGainZ)

            self.prev_setpt_x = filt_roll
            self.prev_setpt_y = filt_pitch
            self.prev_setpt_z = filt_yaw
            self.prev_error_x = err_x
            self.prev_error_y = err_y
            self.prev_error_z = err_z

            motor1 = 1.025 * (filt_thr - pid_x - pid_y - pid_z)
            motor2 = 1.025 * (filt_thr - pid_x + pid_y + pid_z)
            motor3 = 1.025 * (filt_thr + pid_x + pid_y - pid_z)
            motor4 = 1.025 * (filt_thr + pid_x - pid_y + pid_z)

            for m in [motor1, motor2, motor3, motor4]:
                if m < 1000: m = 1000
                if m > 2000: m = 2000

            px = int(max(1000, min(2000, motor1)))
            py = int(max(1000, min(2000, motor2)))
            pz = int(max(1000, min(2000, motor3)))
            pw = int(max(1000, min(2000, motor4)))

            frame = struct.pack('<HHHH', px, py, pz, pw)
            self.serial.write(frame)

            print(f"R:{filt_roll:6.1f} P:{filt_pitch:6.1f} Y:{filt_yaw:6.1f} T:{filt_thr:.0f} | "
                  f"M:[{px},{py},{pz},{pw}] | Angles:[{self.angle_x:.1f},{self.angle_y:.1f},{self.angle_z:.1f}]")

            time.sleep(self.IMU_SAMPLE_FREQ_S)

        print('Simulation complete.')

if __name__ == '__main__':
    controller = RateModeController('COM3', 115200)
    controller.run(duration_sec=180)
