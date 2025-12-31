# Jugglo

This is a hobby IMU-based juggling ball project that streams inertial data over IP/UDP to a computer for visualization and sonification. 

Wi-Fi credentials live in `src/wifi_secret.h` (see below).

### 3d files 

3d files in stl (ball and electronics enclosure) for printing are in folder 3dfiles and were generated using cadquery

### Hardware

- Board: ESP32-C6 DevKitC-1
- IMU: BMI160 (I2C address 0x69)
- Wiring: SCL → GPIO20, SDA → GPIO19 (internal pull-ups enabled)


## Receiving data on the host
- Ensure the host machine is reachable at `TCP_HOST` on `TCP_PORT`.
- View raw sensor data and orientation filter output with:
```bash
python read.py | python plot_raw_vis.py 
```


### Development 

The project is developed with PlatformIO on the Espressif platform (tested on Ubuntu; not yet on Windows).

#### TODO 
 - [ ] Easy Wi-Fi setup (without rebuilding the project / QR code or internal storage)
 - [ ] Recheck hardware design (external antenna and integrated IMU with microcontroller)
 - [ ] Pass the data to external VJ software

#### Configure secrets and target host

`src/wifi_secret.h` is git-ignored. Create it locally before building:
```c
#define WIFI_SSID "your-ssid"
#define WIFI_PASS "your-password"

// Host that receives UDP IMU packets
#define TCP_HOST "host-or-ip"
#define TCP_PORT 50555
```

> Note: `src/main.cpp` expects `TCP_HOST`/`TCP_PORT`; the default fallback defines `MASTER_*` but those macros are unused in the current code, so define the TCP_* values in your secret header.


## Build with PlatformIO
The repository also includes `platformio.ini`:
```bash
pio run -e esp32-c6-devkitc-1
pio run -e esp32-c6-devkitc-1 --target upload --upload-port /dev/ttyACM0
pio device monitor -e esp32-c6-devkitc-1
```
