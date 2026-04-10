# -*- coding: utf-8 -*-
"""
Quadcopter Physics Simulator
============================
A full 6-DOF quadcopter simulator with:
  - Rigid body dynamics (Newton-Euler equations)
  - Motor thrust & drag models
  - PID attitude + altitude controllers
  - Real-time 3D matplotlib visualization
  - Keyboard control (arrow keys + W/S for altitude)

Requirements: numpy, matplotlib
    pip install numpy matplotlib
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import time, threading

# ─────────────────────────────────────────────
#  PHYSICAL CONSTANTS
# ─────────────────────────────────────────────
g = 9.81          # m/s²
dt = 0.005        # simulation timestep (s)
ANIM_INTERVAL = 50 # ms between animation frames

# Quadcopter parameters
MASS   = 0.5      # kg
L      = 0.175    # arm length (m)
KT     = 1.916e-08 # thrust coefficient  (N / rpm^2) -> hover at ~8000 RPM
KD     = 3.066e-10 # drag torque coeff   (N.m / rpm^2)
IMAT   = np.diag([4.856e-3, 4.856e-3, 8.801e-3])  # inertia tensor (kg·m²)
MAX_RPM= 14000.0
MIN_RPM= 100.0

# Motor layout (X-config):
#   1(CW) --front-- 0(CCW)
#      \            /
#       \          /
#   2(CCW)--back--3(CW)
#
# Motor positions relative to body center
MOTOR_POS = np.array([
    [ L,  L, 0],   # 0 front-right  CCW
    [-L,  L, 0],   # 1 front-left   CW
    [-L, -L, 0],   # 2 rear-left    CCW
    [ L, -L, 0],   # 3 rear-right   CW
])
MOTOR_DIR = np.array([-1, 1, -1, 1])  # +1 CW, -1 CCW (torque sign)

# Hover RPM
HOVER_RPM = np.sqrt(MASS * g / (4 * KT))

# ─────────────────────────────────────────────
#  MATH UTILITIES
# ─────────────────────────────────────────────
def Rx(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1,0,0],[0,c,-s],[0,s,c]])

def Ry(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c,0,s],[0,1,0],[-s,0,c]])

def Rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c,-s,0],[s,c,0],[0,0,1]])

def rotation_matrix(phi, theta, psi):
    """ZYX Euler → rotation matrix (body→world)"""
    return Rz(psi) @ Ry(theta) @ Rx(phi)

def euler_rates_to_body_rates(phi, theta, dphi, dtheta, dpsi):
    """Convert Euler angle rates to body angular rates [p, q, r]"""
    T = np.array([
        [1,  0,         -np.sin(theta)         ],
        [0,  np.cos(phi),  np.sin(phi)*np.cos(theta)],
        [0, -np.sin(phi),  np.cos(phi)*np.cos(theta)],
    ])
    return T @ np.array([dphi, dtheta, dpsi])

# ─────────────────────────────────────────────
#  PID CONTROLLER
# ─────────────────────────────────────────────
class PID:
    def __init__(self, kp, ki, kd, limit=None):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.limit = limit
        self.integral = 0.0
        self.prev_err = 0.0

    def update(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_err) / dt
        self.prev_err = error
        out = self.kp * error + self.ki * self.integral + self.kd * derivative
        if self.limit:
            out = np.clip(out, -self.limit, self.limit)
        return out

    def reset(self):
        self.integral = 0.0
        self.prev_err = 0.0

# ─────────────────────────────────────────────
#  QUADCOPTER STATE
# ─────────────────────────────────────────────
class Quadcopter:
    def __init__(self):
        # Position & velocity (world frame)
        self.pos = np.array([0.0, 0.0, 1.0])  # start 1m up
        self.vel = np.zeros(3)
        # Euler angles & rates (ZYX: roll=phi, pitch=theta, yaw=psi)
        self.euler = np.zeros(3)   # [phi, theta, psi]
        self.omega = np.zeros(3)   # body angular rates [p, q, r]
        # Motor RPMs
        self.rpm = np.full(4, HOVER_RPM)
        # History for plots
        self.hist_pos   = []
        self.hist_euler = []
        self.hist_t     = []
        self.t = 0.0

    def forces_torques(self):
        """Compute total thrust & torques from motor RPMs"""
        thrusts = KT * self.rpm**2          # per-motor thrust (N)
        total_thrust = np.sum(thrusts)

        # Torques about body axes
        tau_roll  = L * (thrusts[0] - thrusts[1] - thrusts[2] + thrusts[3])
        tau_pitch = L * (thrusts[0] + thrusts[1] - thrusts[2] - thrusts[3])
        tau_yaw   = np.sum(MOTOR_DIR * KD * self.rpm**2)
        return total_thrust, np.array([tau_roll, tau_pitch, tau_yaw])

    def step(self):
        phi, theta, psi = self.euler
        R = rotation_matrix(phi, theta, psi)

        F, tau = self.forces_torques()

        # Translational dynamics (world frame)
        thrust_world = R @ np.array([0, 0, F])
        accel = thrust_world / MASS - np.array([0, 0, g])
        # Simple drag
        accel -= 0.1 * self.vel

        self.vel += accel * dt
        self.pos += self.vel * dt
        # Ground constraint
        if self.pos[2] < 0:
            self.pos[2] = 0.0
            self.vel[2] = max(0, self.vel[2])

        # Rotational dynamics (body frame)
        alpha = np.linalg.inv(IMAT) @ (tau - np.cross(self.omega, IMAT @ self.omega))
        # Simple angular drag
        alpha -= 0.1 * self.omega

        self.omega += alpha * dt

        # Integrate Euler angles via transformation
        p, q, r = self.omega
        phi, theta, psi = self.euler
        cphi, sphi = np.cos(phi), np.sin(phi)
        ctheta = np.cos(theta)
        dphi   = p + (q * sphi + r * cphi) * np.tan(theta)
        dtheta = q * cphi - r * sphi
        dpsi   = (q * sphi + r * cphi) / ctheta if abs(ctheta) > 1e-6 else 0

        self.euler += np.array([dphi, dtheta, dpsi]) * dt
        # Wrap yaw
        self.euler[2] = (self.euler[2] + np.pi) % (2 * np.pi) - np.pi

        self.t += dt
        self.hist_pos.append(self.pos.copy())
        self.hist_euler.append(np.degrees(self.euler.copy()))
        self.hist_t.append(self.t)

# ─────────────────────────────────────────────
#  FLIGHT CONTROLLER
# ─────────────────────────────────────────────
class FlightController:
    def __init__(self, quad: Quadcopter):
        self.quad = quad
        # Setpoints
        self.target_alt  = 1.0   # m
        self.target_roll = 0.0   # rad
        self.target_pitch= 0.0   # rad
        self.target_yaw  = 0.0   # rad

        # PIDs
        self.pid_alt   = PID(8.0,  0.5,  4.0,  limit=MASS*g*2)
        self.pid_roll  = PID(6.0,  0.1,  2.0,  limit=0.5)
        self.pid_pitch = PID(6.0,  0.1,  2.0,  limit=0.5)
        self.pid_yaw   = PID(4.0,  0.05, 1.0,  limit=0.1)

        self.hover_thrust = MASS * g   # Newtons
        self.base_rpm = HOVER_RPM

    def update(self):
        q = self.quad
        phi, theta, psi = q.euler

        # Altitude (world z)
        err_alt   = self.target_alt - q.pos[2]
        err_roll  = self.target_roll  - phi
        err_pitch = self.target_pitch - theta
        err_yaw   = self.target_yaw   - psi
        # Wrap yaw error
        err_yaw = (err_yaw + np.pi) % (2*np.pi) - np.pi

        u_alt   = self.pid_alt.update(err_alt,   dt)
        u_roll  = self.pid_roll.update(err_roll,  dt)
        u_pitch = self.pid_pitch.update(err_pitch,dt)
        u_yaw   = self.pid_yaw.update(err_yaw,   dt)

        # Base thrust RPM²  (hover + altitude correction)
        # F = KT * rpm²  →  rpm = sqrt(F/KT)
        base_f  = (self.hover_thrust + u_alt) / 4.0
        base_f  = max(base_f, 0.01)
        base_rpm2 = base_f / KT

        # Roll / pitch / yaw mixing
        # Motor layout (see MOTOR_POS):
        #   0: +roll -pitch -yaw  (FR, CCW)
        #   1: -roll -pitch +yaw  (FL, CW)
        #   2: -roll +pitch -yaw  (RL, CCW)
        #   3: +roll +pitch +yaw  (RR, CW)
        dr = u_roll  / (4 * KT * 2*L + 1e-9)
        dp = u_pitch / (4 * KT * 2*L + 1e-9)
        dy = u_yaw   / (4 * KD + 1e-9)

        rpm2 = np.array([
            base_rpm2 + dr - dp - dy,
            base_rpm2 - dr - dp + dy,
            base_rpm2 - dr + dp - dy,
            base_rpm2 + dr + dp + dy,
        ])
        rpm2 = np.clip(rpm2, MIN_RPM**2, MAX_RPM**2)
        q.rpm = np.sqrt(rpm2)

# ─────────────────────────────────────────────
#  VISUALIZATION
# ─────────────────────────────────────────────
def make_quad_artists(ax, pos, euler, arm_len=L):
    """Return 3D line artists for the quad body"""
    phi, theta, psi = euler
    R = rotation_matrix(phi, theta, psi)
    artists = []
    colors = ['#FF4B4B', '#FF4B4B', '#00D4FF', '#00D4FF']
    for i, mp in enumerate(MOTOR_POS):
        world_mp = pos + R @ mp
        line, = ax.plot(
            [pos[0], world_mp[0]],
            [pos[1], world_mp[1]],
            [pos[2], world_mp[2]],
            color=colors[i], linewidth=3, solid_capstyle='round'
        )
        dot, = ax.plot([world_mp[0]], [world_mp[1]], [world_mp[2]],
                       'o', color=colors[i], markersize=8)
        artists += [line, dot]
    # Body dot
    center, = ax.plot([pos[0]], [pos[1]], [pos[2]],
                      'o', color='#FFE566', markersize=10, zorder=10)
    artists.append(center)
    return artists

# ─────────────────────────────────────────────
#  MAIN SIMULATION LOOP
# ─────────────────────────────────────────────
def run():
    quad = Quadcopter()
    ctrl = FlightController(quad)

    # ── Keyboard state ──
    keys = {
        'up':False,'down':False,'left':False,'right':False,
        'w':False,'s':False,'q':False,'e':False
    }

    # ── Figure layout ──
    fig = plt.figure(figsize=(14, 8), facecolor='#0d0d1a')
    fig.suptitle('QUADCOPTER SIMULATOR', fontsize=14,
                 color='#FFE566', fontweight='bold', fontfamily='monospace', y=0.98)

    gs = fig.add_gridspec(3, 3, hspace=0.45, wspace=0.35,
                          left=0.05, right=0.97, top=0.92, bottom=0.07)

    ax3d  = fig.add_subplot(gs[:, :2], projection='3d')
    ax_alt= fig.add_subplot(gs[0, 2])
    ax_rp = fig.add_subplot(gs[1, 2])
    ax_rpm= fig.add_subplot(gs[2, 2])

    for ax in [ax_alt, ax_rp, ax_rpm]:
        ax.set_facecolor('#111126')
        ax.tick_params(colors='#8888aa', labelsize=7)
        for spine in ax.spines.values():
            spine.set_edgecolor('#333355')

    ax3d.set_facecolor('#0d0d1a')
    ax3d.tick_params(colors='#8888aa', labelsize=7)

    # Control hints
    hint = ("↑↓←→: pitch/roll  W/S: altitude  Q/E: yaw  ESC: quit")
    fig.text(0.01, 0.01, hint, fontsize=8, color='#8888aa', fontfamily='monospace')

    # ── Sim thread ──
    SIM_STEPS_PER_FRAME = int(ANIM_INTERVAL / 1000 / dt)
    running = [True]

    def sim_loop():
        while running[0]:
            # Apply key commands to setpoints
            speed = 0.01
            yaw_speed = 0.02
            if keys['w']: ctrl.target_alt   += 0.02
            if keys['s']: ctrl.target_alt   -= 0.02
            if keys['up']:    ctrl.target_pitch -= speed
            if keys['down']:  ctrl.target_pitch += speed
            if keys['left']:  ctrl.target_roll  -= speed
            if keys['right']: ctrl.target_roll  += speed
            if keys['q']:     ctrl.target_yaw   -= yaw_speed
            if keys['e']:     ctrl.target_yaw   += yaw_speed

            ctrl.target_alt   = max(0.0, min(10.0, ctrl.target_alt))
            ctrl.target_roll  = np.clip(ctrl.target_roll,  -0.5, 0.5)
            ctrl.target_pitch = np.clip(ctrl.target_pitch, -0.5, 0.5)

            for _ in range(SIM_STEPS_PER_FRAME):
                ctrl.update()
                quad.step()

            time.sleep(ANIM_INTERVAL / 1000)

    sim_thread = threading.Thread(target=sim_loop, daemon=True)
    sim_thread.start()

    # ── Animation update ──
    TRAIL_LEN = 200

    def update(frame):
        ax3d.cla()
        ax_alt.cla(); ax_rp.cla(); ax_rpm.cla()

        pos   = quad.pos.copy()
        euler = quad.euler.copy()

        # ── 3D view ──
        ax3d.set_facecolor('#0d0d1a')
        lim = 3
        cx, cy = pos[0], pos[1]
        ax3d.set_xlim(cx-lim, cx+lim)
        ax3d.set_ylim(cy-lim, cy+lim)
        ax3d.set_zlim(0, 6)
        ax3d.set_xlabel('X (m)', color='#8888aa', fontsize=8)
        ax3d.set_ylabel('Y (m)', color='#8888aa', fontsize=8)
        ax3d.set_zlabel('Z (m)', color='#8888aa', fontsize=8)
        ax3d.set_title('3D Position', color='#FFE566', fontsize=9,
                       fontfamily='monospace', pad=2)
        ax3d.tick_params(colors='#8888aa', labelsize=6)
        ax3d.xaxis.pane.fill = False
        ax3d.yaxis.pane.fill = False
        ax3d.zaxis.pane.fill = False
        ax3d.grid(True, color='#1a1a3a', linewidth=0.5)

        # Ground grid
        gx = np.linspace(cx-lim, cx+lim, 8)
        gy = np.linspace(cy-lim, cy+lim, 8)
        for x in gx:
            ax3d.plot([x,x],[gy[0],gy[-1]],[0,0],color='#1a1a3a',lw=0.4)
        for y in gy:
            ax3d.plot([gx[0],gx[-1]],[y,y],[0,0],color='#1a1a3a',lw=0.4)

        # Trail
        hist = quad.hist_pos
        if len(hist) > 2:
            trail = np.array(hist[-TRAIL_LEN:])
            alpha_vals = np.linspace(0.05, 0.7, len(trail)-1)
            for i in range(len(trail)-1):
                ax3d.plot(trail[i:i+2, 0], trail[i:i+2, 1], trail[i:i+2, 2],
                          color='#00D4FF', alpha=float(alpha_vals[i]), linewidth=1)

        # Quad body
        make_quad_artists(ax3d, pos, euler)

        # Target altitude line
        ax3d.plot([pos[0], pos[0]], [pos[1], pos[1]], [pos[2], ctrl.target_alt],
                  '--', color='#FFE56688', linewidth=1)

        # ── Altitude plot ──
        ax_alt.set_facecolor('#111126')
        t_arr = np.array(quad.hist_t[-500:]) if quad.hist_t else np.array([0])
        p_arr = np.array(quad.hist_pos[-500:]) if quad.hist_pos else np.array([[0,0,1]])
        if len(t_arr) > 1:
            ax_alt.plot(t_arr, p_arr[:,2], color='#00D4FF', linewidth=1.2)
            ax_alt.axhline(ctrl.target_alt, color='#FFE566', linewidth=0.8, linestyle='--')
        ax_alt.set_ylabel('Alt (m)', color='#8888aa', fontsize=7)
        ax_alt.set_title('Altitude', color='#FFE566', fontsize=8, fontfamily='monospace')
        ax_alt.tick_params(colors='#8888aa', labelsize=6)
        for sp in ax_alt.spines.values(): sp.set_edgecolor('#333355')

        # ── Roll/Pitch plot ──
        ax_rp.set_facecolor('#111126')
        e_arr = np.array(quad.hist_euler[-500:]) if quad.hist_euler else np.array([[0,0,0]])
        if len(t_arr) > 1:
            ax_rp.plot(t_arr, e_arr[:,0], color='#FF4B4B', linewidth=1, label='Roll')
            ax_rp.plot(t_arr, e_arr[:,1], color='#4BFF8A', linewidth=1, label='Pitch')
            ax_rp.axhline(np.degrees(ctrl.target_roll),  color='#FF4B4B66', linestyle='--', lw=0.7)
            ax_rp.axhline(np.degrees(ctrl.target_pitch), color='#4BFF8A66', linestyle='--', lw=0.7)
        ax_rp.set_ylabel('Degrees', color='#8888aa', fontsize=7)
        ax_rp.set_title('Roll / Pitch', color='#FFE566', fontsize=8, fontfamily='monospace')
        leg = ax_rp.legend(fontsize=6, facecolor='#0d0d1a', edgecolor='#333355',
                           loc='upper left')
        for text in leg.get_texts():
            text.set_color('white')
        ax_rp.tick_params(colors='#8888aa', labelsize=6)
        for sp in ax_rp.spines.values(): sp.set_edgecolor('#333355')

        # ── RPM plot ──
        ax_rpm.set_facecolor('#111126')
        cols = ['#FF4B4B','#FF9B4B','#00D4FF','#4BFF8A']
        labels = ['M0(FR)','M1(FL)','M2(RL)','M3(RR)']
        bar_vals = quad.rpm / MAX_RPM * 100
        bars = ax_rpm.bar(labels, bar_vals, color=cols, alpha=0.85, width=0.6)
        ax_rpm.set_ylim(0, 100)
        ax_rpm.set_ylabel('RPM %', color='#8888aa', fontsize=7)
        ax_rpm.set_title('Motor RPM', color='#FFE566', fontsize=8, fontfamily='monospace')
        ax_rpm.tick_params(colors='#8888aa', labelsize=6)
        ax_rpm.axhline(HOVER_RPM/MAX_RPM*100, color='#FFE566', linestyle='--', lw=0.8)
        for sp in ax_rpm.spines.values(): sp.set_edgecolor('#333355')

        # HUD overlay text
        hud = (f"Pos: ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}) m\n"
               f"Vel: ({quad.vel[0]:.2f}, {quad.vel[1]:.2f}, {quad.vel[2]:.2f}) m/s\n"
               f"Roll:{np.degrees(euler[0]):+6.1f}°  "
               f"Pitch:{np.degrees(euler[1]):+6.1f}°  "
               f"Yaw:{np.degrees(euler[2]):+6.1f}°\n"
               f"Target Alt: {ctrl.target_alt:.2f} m")
        ax3d.text2D(0.01, 0.98, hud, transform=ax3d.transAxes,
                    fontsize=7, color='#ccccee', fontfamily='monospace',
                    verticalalignment='top',
                    bbox=dict(boxstyle='round,pad=0.3', facecolor='#0d0d1a88',
                              edgecolor='#333355', alpha=0.8))

    # ── Key events ──
    key_map = {
        'up': 'up', 'down': 'down', 'left': 'left', 'right': 'right',
        'w': 'w', 's': 's', 'q': 'q', 'e': 'e'
    }

    def on_press(event):
        k = event.key
        if k in key_map:
            keys[key_map[k]] = True
        if k == 'escape':
            running[0] = False
            plt.close('all')

    def on_release(event):
        k = event.key
        if k in key_map:
            keys[key_map[k]] = False

    fig.canvas.mpl_connect('key_press_event', on_press)
    fig.canvas.mpl_connect('key_release_event', on_release)

    ani = FuncAnimation(fig, update, interval=ANIM_INTERVAL, cache_frame_data=False)

    print("=" * 55)
    print("  QUADCOPTER SIMULATOR")
    print("=" * 55)
    print("  Controls (click the plot window first):")
    print("    ↑ / ↓      : pitch forward / backward")
    print("    ← / →      : roll left / right")
    print("    W / S      : increase / decrease altitude")
    print("    Q / E      : yaw left / right")
    print("    ESC        : quit")
    print("-" * 55)
    print(f"  Hover RPM : {HOVER_RPM:.0f}")
    print(f"  Mass      : {MASS} kg")
    print(f"  Arm length: {L} m")
    print("=" * 55)

    plt.show()
    running[0] = False


if __name__ == '__main__':
    run()