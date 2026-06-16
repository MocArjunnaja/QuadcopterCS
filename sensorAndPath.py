import sim

import math
import numpy as np
import cv2

# --- Shared state, diinisialisasi oleh init() ---
_clientID: int = -1
_handles: dict = {}


def init(clientID: int) -> None:
    """Inisialisasi sensor module dengan clientID yang sudah ada."""
    global _clientID, _handles
    _clientID = clientID

    # Nama pakai format deprecated (Quadcopter_X) yang sudah terbukti bekerja
    names = {
        'drone':      'Quadcopter',
        'prox_front': 'proximitySensor_front',
        'prox_right': 'proximitySensor_right',
        'prox_left':  'proximitySensor_left',
        'camera':     'visionSensor',
    }
    for key, name in names.items():
        _, handle = sim.simxGetObjectHandle(clientID, name, sim.simx_opmode_blocking)
        _handles[key] = handle


def get_drone_position() -> list:
    _, pos = sim.simxGetObjectPosition(_clientID, _handles['drone'], -1, sim.simx_opmode_streaming)
    return list(pos)


def get_drone_orientation() -> list:
    _, ori = sim.simxGetObjectOrientation(_clientID, _handles['drone'], -1, sim.simx_opmode_streaming)
    return list(ori)


def get_drone_velocity() -> tuple:
    """Return (linear_vel [vx,vy,vz], angular_vel [wx,wy,wz])."""
    _, lin, ang = sim.simxGetObjectVelocity(_clientID, _handles['drone'], sim.simx_opmode_streaming)
    return list(lin), list(ang)


def get_proximity(direction: str = 'front') -> tuple:
    """Return (detected: bool, point: [x,y,z], distance: float). direction: 'front'|'right'|'left'."""
    handle = _handles[f'prox_{direction}']
    err, detected, point, _, _ = sim.simxReadProximitySensor(_clientID, handle, sim.simx_opmode_streaming)
    if err == sim.simx_return_ok and detected:
        distance = math.sqrt(sum(p ** 2 for p in point))
        return True, list(point), distance
    return False, [0.0, 0.0, 0.0], 0.0


def get_camera_image():
    """Return numpy image (RGB) atau None jika belum tersedia."""
    err, res, image = sim.simxGetVisionSensorImage(
        _clientID, _handles['camera'], 0, sim.simx_opmode_streaming
    )
    if err == sim.simx_return_ok:
        img = np.array(image, dtype=np.int8).view(np.uint8).reshape((res[1], res[0], 3))
        img = np.flipud(img)
        return cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    return None


def _draw_hud(frame: np.ndarray, pos, ori, lin_vel, ang_vel, prox_f, prox_r, prox_l) -> np.ndarray:
    """Overlay telemetry HUD di sudut kanan atas frame."""
    _, w = frame.shape[:2]

    lines = [
        f"Pos    : [{pos[0]:7.3f}, {pos[1]:7.3f}, {pos[2]:7.3f}]",
        f"Ori    : [{math.degrees(ori[0]):6.1f}, {math.degrees(ori[1]):6.1f}, {math.degrees(ori[2]):6.1f}] deg",
        f"Vel    : [{lin_vel[0]:6.3f}, {lin_vel[1]:6.3f}, {lin_vel[2]:6.3f}]",
        f"AngVel : [{ang_vel[0]:6.3f}, {ang_vel[1]:6.3f}, {ang_vel[2]:6.3f}]",
        f"Prox F : {'HIT' if prox_f[0] else '---'}  {prox_f[2]:.2f} m",
        f"Prox R : {'HIT' if prox_r[0] else '---'}  {prox_r[2]:.2f} m",
        f"Prox L : {'HIT' if prox_l[0] else '---'}  {prox_l[2]:.2f} m",
    ]

    font       = cv2.FONT_HERSHEY_SIMPLEX
    scale      = 0.25
    thickness  = 1
    line_h     = 11
    padding    = 6
    text_color = (0, 255, 0)
    bg_color   = (0, 0, 0)

    # lebar teks terpanjang
    max_w = max(cv2.getTextSize(l, font, scale, thickness)[0][0] for l in lines)
    box_w = max_w + padding * 2
    box_h = line_h * len(lines) + padding * 2
    x0    = w - box_w - 8
    y0    = 8

    # background semi-transparan
    overlay = frame.copy()
    cv2.rectangle(overlay, (x0, y0), (x0 + box_w, y0 + box_h), bg_color, -1)
    frame = cv2.addWeighted(overlay, 0.55, frame, 0.45, 0)

    for i, line in enumerate(lines):
        y = y0 + padding + (i + 1) * line_h
        cv2.putText(frame, line, (x0 + padding, y), font, scale, text_color, thickness, cv2.LINE_AA)

    return frame


# --- Standalone test ---
if __name__ == "__main__":
    from connection import connect
    init(connect())

    WIN = "Vision Sensor"
    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)   # resizable
    cv2.resizeWindow(WIN, 800, 600)

    last_frame = None

    while True:
        pos              = get_drone_position()
        ori              = get_drone_orientation()
        lin_vel, ang_vel = get_drone_velocity()
        prox_f           = get_proximity('front')
        prox_r           = get_proximity('right')
        prox_l           = get_proximity('left')

        cam = get_camera_image()
        if cam is not None:
            last_frame = cam

        if last_frame is not None:
            display = _draw_hud(last_frame.copy(), pos, ori, lin_vel, ang_vel, prox_f, prox_r, prox_l)
            cv2.imshow(WIN, display)

        if cv2.waitKey(1) & 0xFF == 27:  # Esc keluar
            break

    cv2.destroyAllWindows()
