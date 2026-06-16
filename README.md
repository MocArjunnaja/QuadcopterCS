# QuadcopterCS

Simulasi UAV quadcopter berbasis **CoppeliaSim** dengan kontrol manual dan perencanaan jalur otomatis menggunakan OMPL. Proyek ini merupakan tugas akhir mata kuliah UAV semester 6.

---

## Fitur

- **Kontrol manual keyboard** dengan PID velocity controller dan low-pass filter
- **Path planning otomatis** menggunakan algoritma RRTConnect (OMPL)
- **Multi-segment path** — drone mengikuti urutan checkpoint: `start → goal1 → goal2 → ...`
- **Smooth yaw tracking** sepanjang jalur yang direncanakan
- **Sensor modul**: proximity sensor (depan, kanan, kiri) dan vision sensor
- **Telemetry HUD** overlay pada live camera feed (posisi, orientasi, kecepatan, jarak obstacle)
- **Visualisasi jalur** 3D di viewport CoppeliaSim

---

## Arsitektur

```
QuadcopterCS/
├── connection.py           # Koneksi ke CoppeliaSim Remote API
├── manual_drone.py         # Kontrol manual keyboard + PID
├── sensorAndPath.py        # Sensor module + vision HUD
├── path_script.lua         # OMPL path planning (dijalankan di dalam sim)
├── pathFollower_script.lua # Path follower dengan smooth yaw
├── sim.py                  # CoppeliaSim Remote API Python bindings
├── simConst.py             # Konstanta Remote API
├── remoteApi.dll           # Remote API DLL (Windows)
└── ETS_UAV.ttt             # Scene CoppeliaSim
```

---

## Prerequisites

| Dependency | Versi |
|---|---|
| Python | 3.9+ |
| CoppeliaSim | 4.x (dengan plugin simOMPL) |
| numpy | latest |
| opencv-python | latest |
| keyboard | latest |

Install Python dependencies:

```bash
pip install numpy opencv-python keyboard
```

Pastikan CoppeliaSim sudah terinstall dan plugin **simOMPL** aktif (untuk path planning).

---

## Cara Penggunaan

### 1. Setup CoppeliaSim

1. Buka CoppeliaSim
2. Load scene `ETS_UAV.ttt`
3. Klik **Start Simulation**

CoppeliaSim harus berjalan terlebih dahulu sebelum menjalankan script Python. Remote API server aktif secara default di port **19997**.

### 2. Kontrol Manual

```bash
python manual_drone.py
```

| Tombol | Aksi |
|---|---|
| `W` / `Up` | Maju |
| `S` / `Down` | Mundur |
| `A` | Geser kiri |
| `D` | Geser kanan |
| `Left Arrow` | Yaw kiri |
| `Right Arrow` | Yaw kanan |
| `Space` | Naik |
| `Shift` | Turun |
| `Esc` | Keluar |

### 3. Monitor Sensor & Kamera

```bash
python sensorAndPath.py
```

Membuka window live camera feed dengan HUD telemetri real-time (posisi, orientasi, kecepatan, pembacaan proximity sensor).

### 4. Path Planning Otomatis

Path planning dijalankan langsung dari dalam CoppeliaSim melalui script Lua:

- **`path_script.lua`** — terpasang pada object `Path` di scene, generate jalur dari `start` → `goal1` → `goal2` → ... menggunakan RRTConnect
- **`pathFollower_script.lua`** — membaca jalur hasil OMPL dan menggerakkan drone target sepanjang waypoints dengan smooth yaw

Konfigurasi waypoint dilakukan dengan mengatur posisi dummy object `start`, `goal1`, `goal2`, dst. di scene editor.

---

## Konfigurasi PID

Parameter PID dan filter dapat disesuaikan di `manual_drone.py`:

```python
MAX_VEL   = 1.5    # m/s maksimum kecepatan translasi
MAX_YAW   = 0.8    # rad/s maksimum kecepatan yaw
VEL_ALPHA = 0.4    # koefisien low-pass filter kecepatan

PID_VXY = (kp=0.30, ki=0.01, kd=0.05, limit=0.20)
PID_VZ  = (kp=0.30, ki=0.01, kd=0.05, limit=0.20)
PID_WZ  = (kp=0.30, ki=0.00, kd=0.05, limit=0.40)
```

---

## Implementasi Teknis

### PID Controller
Menggunakan PID dengan **anti-windup**: integral term dikembalikan jika output saturasi. Reset otomatis saat tidak ada input keyboard.

### Low-Pass Filter
Velocity reading dari simulator difilter dengan LPF first-order untuk mengurangi noise:
```
v_filtered = α · v_raw + (1 - α) · v_filtered_prev
```

### OMPL Path Planning
- Algoritma: **RRTConnect** dengan 5 detik timeout dan 300 simplifikasi langkah
- Search space: pose3D dalam batas `[-14, 14, 0.25] → [14, 14, 3.0]` meter
- Robot collision model di-scale 1.75x untuk margin keamanan
- Jalur multi-segmen digabung menjadi satu path kontinu

---

## Lisensi

MIT License — lihat [LICENSE](LICENSE)
