import sim
import math
import time
import keyboard
from connection import connect
from sensor import init as sensor_init, get_drone_orientation, get_drone_velocity

TARGET_NAME = 'Quadcopter_target'
MAX_VEL     = 1.5
MAX_YAW     = 0.8
VEL_ALPHA   = 0.4
LOOP_DT     = 0.05

PID_VXY = (0.30, 0.01, 0.05, 0.20)
PID_VZ  = (0.30, 0.01, 0.05, 0.20)
PID_WZ  = (0.30, 0.00, 0.05, 0.40)


class PID:
    def __init__(self, kp, ki, kd, limit=None):
        self.kp, self.ki, self.kd, self.limit = kp, ki, kd, limit
        self._i, self._pe = 0.0, 0.0

    def compute(self, error, dt):
        self._i += error * dt
        out = self.kp * error + self.ki * self._i + self.kd * (error - self._pe) / max(dt, 1e-3)
        self._pe = error
        if self.limit:
            if abs(out) > self.limit:
                self._i -= error * dt
                out = math.copysign(self.limit, out)
        return out


class LPF:
    def __init__(self, alpha):
        self.alpha, self._v = alpha, None

    def update(self, x):
        self._v = x if self._v is None else self.alpha * x + (1 - self.alpha) * self._v
        return self._v


clientID = connect()
sensor_init(clientID)

_, target  = sim.simxGetObjectHandle(clientID, TARGET_NAME, sim.simx_opmode_blocking)
_, tgt_pos = sim.simxGetObjectPosition(clientID, target, -1, sim.simx_opmode_blocking)
_, ori     = sim.simxGetObjectOrientation(clientID, target, -1, sim.simx_opmode_blocking)
tgt_pos    = list(tgt_pos)
tgt_yaw    = ori[2]

pid_vx, pid_vy = PID(*PID_VXY), PID(*PID_VXY)
pid_vz, pid_wz = PID(*PID_VZ),  PID(*PID_WZ)
fx, fy, fz, fw = LPF(VEL_ALPHA), LPF(VEL_ALPHA), LPF(VEL_ALPHA), LPF(VEL_ALPHA)

print("W/S=fwd | A/D=strafe | Left/Right=yaw | Space/Shift=up/down | Esc=quit")
prev_time = time.time()

while True:
    now = time.time()
    dt  = max(now - prev_time, 1e-3)
    prev_time = now

    roll, pitch, yaw = get_drone_orientation()
    lin, ang = get_drone_velocity()
    vx, vy, vz, wz = fx.update(lin[0]), fy.update(lin[1]), fz.update(lin[2]), fw.update(ang[2])

    fwd = strafe = up = yaw_cmd = 0.0
    if keyboard.is_pressed('w') or keyboard.is_pressed('up'):    fwd    =  MAX_VEL
    if keyboard.is_pressed('s') or keyboard.is_pressed('down'):  fwd    = -MAX_VEL
    if keyboard.is_pressed('a'):                                  strafe =  MAX_VEL
    if keyboard.is_pressed('d'):                                  strafe = -MAX_VEL
    if keyboard.is_pressed('left'):                               yaw_cmd = -MAX_YAW
    if keyboard.is_pressed('right'):                              yaw_cmd =  MAX_YAW
    if keyboard.is_pressed('space'):                              up     =  MAX_VEL
    if keyboard.is_pressed('shift'):                              up     = -MAX_VEL

    if fwd == 0 and strafe == 0 and up == 0 and yaw_cmd == 0:
        for p in (pid_vx, pid_vy, pid_vz, pid_wz):
            p.reset()

    cos_y, sin_y = math.cos(yaw), math.sin(yaw)
    des_vx = fwd * cos_y - strafe * sin_y
    des_vy = fwd * sin_y + strafe * cos_y

    tgt_pos[0] += pid_vx.compute(des_vx - vx, dt) * dt
    tgt_pos[1] += pid_vy.compute(des_vy - vy, dt) * dt
    tgt_pos[2] += pid_vz.compute(up - vz,     dt) * dt
    tgt_yaw    += pid_wz.compute(yaw_cmd - wz, dt) * dt

    sim.simxSetObjectPosition(clientID, target, -1, tgt_pos, sim.simx_opmode_oneshot)
    sim.simxSetObjectOrientation(clientID, target, -1, (0.0, 0.0, tgt_yaw), sim.simx_opmode_oneshot)

    print(f"vel: {vx:+.2f} {vy:+.2f} {vz:+.2f} {wz:+.2f} | des: {des_vx:+.2f} {des_vy:+.2f} {up:+.2f} {yaw_cmd:+.2f} | yaw={math.degrees(yaw):.1f}°", end='\r')

    if keyboard.is_pressed('esc'):
        print()
        break

    time.sleep(max(0.0, LOOP_DT - (time.time() - now)))
