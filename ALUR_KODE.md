# Alur Kode QuadcopterCS

Dokumen ini menjelaskan alur kerja sistem simulasi UAV quadcopter: bagaimana komponen Python dan script Lua berkomunikasi dengan CoppeliaSim, jalur data antar modul, dan urutan eksekusi tiap fitur.

---

## 1. Gambaran Arsitektur

Sistem terbagi dua sisi yang berkomunikasi lewat **Remote API** CoppeliaSim (port `19997`):

```
        SISI PYTHON (client)                    SISI COPPELIASIM (server)
 ┌──────────────────────────────┐        ┌──────────────────────────────────┐
 │ connection.py  ── connect()  │        │  Scene: ETS_UAV.ttt               │
 │ manual_drone.py ── kontrol   │◄──────►│   Quadcopter (+ base, target)     │
 │ sensorAndPath.py ── sensor   │ Remote │   proximitySensor_{front,..}      │
 │ sim.py / simConst.py (binding)│  API  │   visionSensor                    │
 │ remoteApi.dll (transport)    │        │  Script Lua (embedded):           │
 └──────────────────────────────┘        │   path_script.lua                 │
                                          │   pathFollower_script.lua         │
                                          │   visionSensor.lua                │
                                          └──────────────────────────────────┘
```

Konsep kunci: **drone tidak digerakkan langsung**. Yang digerakkan adalah dummy object `Quadcopter_target`. Controller internal CoppeliaSim membuat drone fisik mengejar posisi/orientasi target itu. Semua kode di repo cuma menggeser target.

| File | Peran | Dijalankan di |
|---|---|---|
| `connection.py` | Buka koneksi Remote API | Python |
| `manual_drone.py` | Kontrol manual keyboard + PID | Python |
| `sensorAndPath.py` | Baca sensor + vision HUD | Python |
| `sim.py`, `simConst.py` | Python binding Remote API | Python |
| `remoteApi.dll` | Transport layer (native) | Python (loaded) |
| `path_script.lua` | Generate path OMPL | Dalam sim (thread) |
| `pathFollower_script.lua` | Gerakkan target ikut path | Dalam sim (thread) |
| `visionSensor.lua` | Arahkan kamera ke target | Dalam sim (actuation) |
| `ETS_UAV.ttt` | Scene (drone, sensor, obstacle) | CoppeliaSim |

---

## 2. Bootstrap Koneksi (`connection.py`)

Titik masuk semua script Python.

```
simxFinish(-1)        # tutup semua koneksi lama (bersih-bersih)
       │
simxStart('127.0.0.1', 19997, ...)
       │
   clientID != -1 ? ── ya ──► return clientID
       │
       └── tidak ──► print gagal, sys.exit(1)
```

`clientID` ini token yang dipakai semua panggilan API berikutnya. Semua modul minta `clientID` dulu sebelum bisa kerja.

---

## 3. Alur Kontrol Manual (`manual_drone.py`)

Fitur utama. Loop real-time yang baca keyboard, jalankan PID, geser target.

### 3.1 Fase Setup (sekali jalan)

```
connect()                          → clientID
sensor_init(clientID)              → register handle sensor
getObjectHandle('Quadcopter_target')
getObjectPosition(target)          → tgt_pos  (state awal)
getObjectOrientation(target)       → tgt_yaw
buat 4 PID + 4 LPF
```

`tgt_pos` dan `tgt_yaw` disimpan sebagai variabel state yang akan **diakumulasi** tiap loop, bukan di-set ulang dari nol.

### 3.2 Loop Kontrol (per ~50 ms, `LOOP_DT`)

```
┌─ hitung dt (waktu nyata antar iterasi)
│
├─ BACA STATE DRONE
│    get_drone_orientation()  → yaw
│    get_drone_velocity()     → lin, ang
│    filter LPF               → vx, vy, vz, wz   (kurangi noise)
│
├─ BACA INPUT KEYBOARD → set target velocity (frame badan drone)
│    W/S      → fwd  ± MAX_VEL
│    A/D      → strafe ± MAX_VEL
│    ←/→      → yaw_cmd ± MAX_YAW
│    Space/Sh → up ± MAX_VEL
│
├─ jika TIDAK ada input → reset semua PID  (cegah drift integral)
│
├─ ROTASI ke frame dunia (pakai yaw drone)
│    des_vx = fwd·cosθ − strafe·sinθ
│    des_vy = fwd·sinθ + strafe·cosθ
│
├─ PID: error = (kecepatan target − kecepatan aktual)
│    tgt_pos[x] += pid_vx(des_vx − vx)·dt
│    tgt_pos[y] += pid_vy(des_vy − vy)·dt
│    tgt_pos[z] += pid_vz(up    − vz)·dt
│    tgt_yaw    += pid_wz(yaw_cmd − wz)·dt
│
├─ KIRIM ke sim (oneshot, non-blocking)
│    setObjectPosition(target, tgt_pos)
│    setObjectOrientation(target, [0,0,tgt_yaw])
│
├─ print telemetri (\r, satu baris)
│
├─ Esc ditekan? → break
│
└─ sleep sisa waktu agar loop ≈ LOOP_DT
```

**Inti logika**: kontrol berbasis kecepatan (velocity control). Keyboard menentukan *kecepatan yang diinginkan*; PID menghitung koreksi berdasar selisih dengan kecepatan aktual; hasilnya diintegrasi (`× dt`) jadi pergeseran posisi target. Drone mengejar target → bergerak.

### 3.3 Komponen Pendukung

**Kelas `PID`** — PID standar dengan anti-windup: kalau output melebihi `limit`, integral term yang barusan ditambah dikembalikan (baris 28-31) supaya integral tidak menumpuk saat saturasi.

**Kelas `LPF`** — low-pass filter orde satu: `v = α·x + (1−α)·v_prev`. Haluskan pembacaan kecepatan yang berisik dari simulator.

---

## 4. Alur Sensor & Vision (`sensorAndPath.py`)

Modul ganda: (a) dipakai `manual_drone.py` lewat `init` + `get_drone_*`, dan (b) standalone untuk monitor kamera.

### 4.1 Sebagai modul (`init` dipanggil dari luar)

```
init(clientID)
   └─ getObjectHandle untuk: drone, prox_front, prox_right,
                              prox_left, camera
      simpan di _handles (dict global)
```

Setelah itu fungsi getter siap dipanggil:

| Fungsi | Kembalian | Mode API |
|---|---|---|
| `get_drone_position()` | `[x,y,z]` | streaming |
| `get_drone_orientation()` | `[roll,pitch,yaw]` | streaming |
| `get_drone_velocity()` | `([vx,vy,vz],[wx,wy,wz])` | streaming |
| `get_proximity(dir)` | `(detected, point, distance)` | streaming |
| `get_camera_image()` | numpy RGB / None | streaming |

Mode **streaming**: panggilan pertama mendaftarkan data agar terus dialirkan server → panggilan berikutnya cepat (ambil dari buffer lokal), cocok untuk loop real-time.

### 4.2 Mode standalone (`python sensorAndPath.py`)

```
init(connect())
buka window OpenCV
loop:
   baca posisi, orientasi, velocity, 3 proximity
   ambil frame kamera (pakai frame terakhir jika None)
   _draw_hud(...) → overlay telemetri di pojok kanan atas
   imshow
   Esc → keluar
```

`get_camera_image()` mengubah byte mentah jadi gambar: reshape `(res_y, res_x, 3)`, balik vertikal (`flipud`, karena origin sim di kiri-bawah), konversi BGR→RGB.

`_draw_hud()` menghitung lebar kotak dari teks terpanjang, gambar background semi-transparan (`addWeighted`), lalu tulis tiap baris telemetri warna hijau.

---

## 5. Alur Path Planning (Lua, di dalam sim)

Berjalan otomatis saat simulasi start. Dua script Lua bekerja sama lewat **Custom Data Block** pada object `/Path` sebagai papan tukar pesan.

### 5.1 Generator — `path_script.lua`

```
sysCall_init():
   tulis OMPL_XYZ_PATH = ''      (kosong)
   tulis OMPL_READY    = {0}     (belum siap)

sysCall_thread():
   kumpulkan checkpoint dummy:
      /Path/start, /Path/goal1, /Path/goal2, ...  (urut)
      fallback /Path/goal kalau cuma 1 goal
   minimal butuh start + 1 goal, kalau kurang → error log + return

   scale omplRobot ×1.75   (margin keamanan collision)

   UNTUK tiap pasang checkpoint (segmen):
      createTask + StateSpace pose3d, batas [-14,-14,0.25]→[14,14,3]
      algoritma RRTConnect
      collisionPairs = {robot, semua object}
      setStart / setGoal
      compute(timeout=5s, simplify=300)
      gagal? → error log, restore scale, return
      ubah path → list XYZ, sambung ke full_xyz

   restore scale robot ÷1.75
   tulis OMPL_XYZ_PATH = full_xyz (packed float)
   naikkan OMPL_PATH_ID (penanda path baru)
   tulis OMPL_READY = {1}         (siap)
   drawPath() → gambar garis cyan di viewport
```

Multi-segmen: tiap pasang checkpoint berurutan dihitung terpisah lalu disambung jadi satu path kontinu `start→goal1→goal2→...`.

### 5.2 Follower — `pathFollower_script.lua`

```
sysCall_thread():
   cari handle target (/target → /Quadcopter_target → /Quadcopter/target)
   tunggu OMPL_READY == 0          (sinkronisasi mulai bersih)
   tunggu OMPL_READY == 1 & path ada → unpack jadi waypoints [{x,y,z},...]

   yaw awal = arah segmen pertama
   UNTUK tiap pasang waypoint:
      d = jarak, T = d/speed, steps = T/dt
      target_yaw = atan2(dy, dx)
      UNTUK tiap step:
         interpolasi linear posisi (p1→p2)
         smooth yaw: putar current_yaw menuju target_yaw,
                     dibatasi yaw_speed·dt, wrap ke [-π,π] (arah terpendek)
         setObjectPosition(target, pos)
         setObjectOrientation(target, [0,0,current_yaw])
         sim.step()
```

**Handshake** antar dua script lewat `OMPL_READY`:

```
path_script:    READY=0 ──(generate)──► READY=1, tulis path
pathFollower:   tunggu 0 ──► tunggu 1 ──► baca path ──► jalankan
```

### 5.3 Kamera — `visionSensor.lua`

`sysCall_actuation` (tiap frame): hitung vektor drone→target, set orientasi kamera (`pitch`, `yaw`) supaya kamera selalu menghadap target. Kamera onboard mengikuti arah gerak.

---

## 6. Hubungan Antar Komponen (ringkas)

```
connection.connect() ──clientID──► manual_drone / sensorAndPath
                                         │
sensorAndPath.init() ──handles──────────┘
                                         │
manual_drone loop ──setObjectPosition──► Quadcopter_target ──► drone fisik
                  ◄─getVelocity/Orient── (state drone)

[di dalam sim, paralel & mandiri]
path_script ──OMPL_READY/XYZ_PATH──► pathFollower ──► Quadcopter_target
visionSensor ──► arahkan kamera ke target
```

Dua jalur kontrol target (Python manual vs Lua pathFollower) **tidak dipakai bersamaan** — pilih salah satu sesuai mode.

---

## 7. Catatan / Potensi Masalah

Beberapa hal yang perlu diperhatikan saat menjalankan:

1. **Import salah nama** — [manual_drone.py:6](manual_drone.py#L6):
   ```python
   from sensor import init as sensor_init, ...
   ```
   File modulnya bernama `sensorAndPath.py`, bukan `sensor.py`. Import ini akan `ModuleNotFoundError`. Perbaikan: ganti jadi `from sensorAndPath import ...` atau rename file jadi `sensor.py`.

2. **`PID.reset()` tidak ada** — [manual_drone.py:81](manual_drone.py#L81) memanggil `p.reset()`, tetapi kelas `PID` (baris 19-32) tidak mendefinisikan method `reset`. Akan `AttributeError` begitu semua input keyboard dilepas. Perbaikan: tambahkan
   ```python
   def reset(self):
       self._i, self._pe = 0.0, 0.0
   ```

3. **Vision sensor handle** — `get_camera_image()` pakai mode streaming; frame pertama biasanya `None` (wajar), karena itu standalone loop menyimpan `last_frame`.

Dua bug pertama membuat `manual_drone.py` tidak jalan apa adanya. Fix dulu sebelum testing.
