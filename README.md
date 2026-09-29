# Quest 2 VR Teleoperation for Isaac Sim 4.5

Control a Franka Panda (or UR5) robot arm inside NVIDIA Isaac Sim 4.5 using a Meta Quest 2 headset over ALVR + SteamVR — no ROS2 required, pure Python standalone.

<p align="center">
  <img src="docs/banner.png" alt="Quest 2 → Isaac Sim VR Teleop" width="720"/>
</p>

---

## Table of Contents

- [Overview](#overview)
- [Hardware Requirements](#hardware-requirements)
- [Software Stack](#software-stack)
- [Repository Layout](#repository-layout)
- [Installation](#installation)
  - [1 — NVIDIA Isaac Sim 4.5](#1--nvidia-isaac-sim-45)
  - [2 — SteamVR](#2--steamvr)
  - [3 — ALVR (wireless Quest 2 streaming)](#3--alvr-wireless-quest-2-streaming)
  - [4 — Meta Quest 2 side-load (ALVR client)](#4--meta-quest-2-side-load-alvr-client)
  - [5 — Python environment](#5--python-environment)
- [Quick Start](#quick-start)
- [Detailed Operating Procedure](#detailed-operating-procedure)
  - [Step 1 — Start ALVR streamer on the PC](#step-1--start-alvr-streamer-on-the-pc)
  - [Step 2 — Connect Quest 2](#step-2--connect-quest-2)
  - [Step 3 — Launch SteamVR](#step-3--launch-steamvr)
  - [Step 4 — Run the teleop script](#step-4--run-the-teleop-script)
  - [Step 5 — Start VR in Isaac Sim GUI](#step-5--start-vr-in-isaac-sim-gui)
  - [Step 6 — Begin teleoperation](#step-6--begin-teleoperation)
- [Controller Reference](#controller-reference)
- [CLI Arguments](#cli-arguments)
- [Architecture](#architecture)
- [Troubleshooting](#troubleshooting)
- [Known Limitations](#known-limitations)

---

## Overview

This project streams a live Isaac Sim physics simulation to a Meta Quest 2 over Wi-Fi and lets you drive a robot arm in real time using the Quest 2 Touch controllers. No wires, no ROS bridge — the script talks directly to the OpenVR (SteamVR) IVRInput API via ctypes.

```
Quest 2 (ALVR client)
      │  Wi-Fi (5 GHz)
      ▼
ALVR Streamer (PC)  ←→  SteamVR (PC)
                              │
                     IVRInput_010 (ctypes)
                              │
                    quest2_teleop_v14.py
                              │
                    Isaac Sim 4.5 (Standalone)
                              │
                    Franka / UR5 articulation
```

---

## Hardware Requirements

| Component | Minimum | Notes |
|-----------|---------|-------|
| PC (host) | Ubuntu 22.04, 64-bit | Windows is not tested |
| GPU | NVIDIA RTX 3070 or better | Ampere+ recommended for RTX rendering |
| VRAM | 8 GB | 10 GB+ for full scene with path tracing |
| RAM | 32 GB | 16 GB works, may cause paging |
| CPU | 8-core, 3 GHz+ | Isaac Sim is CPU-intensive during warmup |
| Network | 5 GHz Wi-Fi router, router and PC on same subnet | Quest 2 connects wirelessly |
| Meta Quest 2 | Any firmware | Developer Mode must be enabled |

---

## Software Stack

| Application | Version | Purpose |
|-------------|---------|---------|
| **Ubuntu** | 22.04 LTS | Host OS |
| **NVIDIA Driver** | 535+ | GPU support |
| **CUDA** | 12.x | Isaac Sim runtime |
| **NVIDIA Isaac Sim** | 4.5 (Standalone) | Simulation engine |
| **Steam** | Latest | Required by SteamVR |
| **SteamVR** | Latest | OpenVR runtime, IVRInput API |
| **ALVR** | 20.x (streamer) | PC-side wireless VR streamer |
| **ALVR Client** | matching version | Sideloaded on Quest 2 |
| **SideQuest** (or adb) | Any | Used to sideload ALVR on Quest 2 |
| **Python** | 3.10 (bundled with Isaac Sim) | Script runtime |

---

## Repository Layout

```
quest2-vr-teleop/
├── quest2_teleop_v14.py   # main script
├── activate_teleop.sh     # convenience env-var setter
├── README.md
└── docs/
    └── banner.png
```

`activate_teleop.sh` — create this once:

```bash
#!/usr/bin/env bash
# Edit ISAAC_SIM_PATH to match your install location
export ISAACSIM_PATH="$HOME/isaacsim"   # or wherever Isaac Sim 4.5 unpacked
export OMNI_KIT_ACCEPT_EULA=YES
echo "[OK] ISAACSIM_PATH=$ISAACSIM_PATH"
```

---

## Installation

### 1 — NVIDIA Isaac Sim 4.5

Isaac Sim 4.5 is installed through **NVIDIA Omniverse Launcher** or downloaded as a standalone zip.

**Via Omniverse Launcher:**

1. Download Omniverse Launcher from https://www.nvidia.com/en-us/omniverse/
2. Open Launcher → **Exchange** tab → search **Isaac Sim** → Install version **4.5.0**
3. Default install path: `~/isaacsim/` (you can change it in Launcher settings)
4. After install finishes, note the path shown — you need it as `ISAACSIM_PATH`

**Verify install:**

```bash
ls ~/isaacsim/apps/
# Should show: isaacsim.exp.full.kit  and other .kit files
```

**NVIDIA driver check:**

```bash
nvidia-smi
# Driver version must be ≥ 535
```

If your driver is older:

```bash
sudo apt install nvidia-driver-535
sudo reboot
```

---

### 2 — SteamVR

SteamVR is the OpenVR runtime. Isaac Sim's XR plugin talks to it.

1. Install Steam:
   ```bash
   sudo apt install steam
   steam
   ```
   Log in or create an account (free).

2. Inside Steam: **Library** → search **SteamVR** → **Install**

3. After install, run SteamVR once to let it complete device setup:
   ```bash
   ~/.steam/steam/steamapps/common/SteamVR/bin/vrstartup.sh
   ```
   You will see the SteamVR window open (it will say "No headset detected" — that is fine for now).

4. Close SteamVR. It will be launched automatically by ALVR later.

---

### 3 — ALVR (wireless Quest 2 streaming)

ALVR streams the PC's display and audio to the Quest 2 over Wi-Fi and feeds Quest 2 head and controller tracking back to SteamVR.

**Download ALVR streamer (PC side):**

```bash
# Check latest release at https://github.com/alvr-org/ALVR/releases
# Download the tar.gz for Linux, e.g. alvr_streamer_linux_v20.x.x.tar.gz
mkdir -p ~/alvr
cd ~/alvr
wget https://github.com/alvr-org/ALVR/releases/download/v20.x.x/alvr_streamer_linux_v20.x.x.tar.gz
tar -xzf alvr_streamer_linux_v20.x.x.tar.gz
```

**Run ALVR dashboard:**

```bash
cd ~/alvr
./ALVR
# Opens a browser-based dashboard at http://localhost:8082
```

ALVR will auto-detect your SteamVR install. If it does not:
- Open the ALVR dashboard → **Settings** → **Installation** → point it to `~/.steam/steam/steamapps/common/SteamVR/`

---

### 4 — Meta Quest 2 side-load (ALVR client)

The Quest 2 needs ALVR's client APK sideloaded since it is not on the Meta store.

**Enable Developer Mode on Quest 2:**
1. Open the **Meta Quest mobile app** on your phone
2. Go to **Menu** → **Devices** → select your headset → **Developer Mode** → toggle ON
3. Put the headset on; accept the prompt to enable Developer Mode

**Install SideQuest on your PC** (easiest sideloading tool):

```bash
# Download AppImage from https://sidequestvr.com/setup-howto
chmod +x SideQuest-*.AppImage
./SideQuest-*.AppImage
```

Or use adb directly:

```bash
sudo apt install android-tools-adb
```

**Sideload ALVR client APK:**

From the ALVR releases page, download `alvr_client_android_v20.x.x.apk` (version must match the streamer).

- **Via SideQuest:** Connect Quest 2 to PC via USB-C → accept the USB debugging prompt in the headset → drag-and-drop the APK onto SideQuest → Install
- **Via adb:**
  ```bash
  adb install alvr_client_android_v20.x.x.apk
  ```

After install, the ALVR client app appears in your Quest 2's **App Library** under **Unknown Sources**.

---

### 5 — Python environment

The script uses Isaac Sim's bundled Python (`python.sh`) which has `numpy`, `omni.*`, `carb`, `pxr` etc. already available. No `pip install` is needed for the main script.

```bash
# Confirm Isaac Sim Python works
~/isaacsim/python.sh -c "import omni; print('omni OK')"
```

Clone this repository:

```bash
git clone https://github.com/<your-username>/quest2-vr-teleop.git
cd quest2-vr-teleop
```

Create the activation script:

```bash
cat > activate_teleop.sh << 'ACTIVATE'
#!/usr/bin/env bash
export ISAACSIM_PATH="$HOME/isaacsim"
export OMNI_KIT_ACCEPT_EULA=YES
echo "[OK] ISAACSIM_PATH=$ISAACSIM_PATH"
ACTIVATE
chmod +x activate_teleop.sh
```

---

## Quick Start

Once everything above is installed, the full flow in one glance:

```bash
# Terminal 1 — start ALVR streamer
cd ~/alvr && ./ALVR

# Put Quest 2 on → launch ALVR client on headset → headset connects to PC

# Terminal 1 — ALVR dashboard will auto-launch SteamVR, or manually:
~/.steam/steam/steamapps/common/SteamVR/bin/vrstartup.sh

# Terminal 2 — run the teleop script
source activate_teleop.sh
~/isaacsim/python.sh quest2_teleop_v14.py --run_vr --robot franka

# Isaac Sim window opens → VR tab → SteamVR → Start VR
# Put Quest 2 on → you are in the simulation
# Right Grip = toggle gripper  |  A/B/X/Y = rotate view
```

---

## Detailed Operating Procedure

### Step 1 — Start ALVR streamer on the PC

Open a terminal:

```bash
cd ~/alvr
./ALVR
```

This opens the ALVR dashboard in your browser at `http://localhost:8082`. Leave this terminal and browser tab open throughout the session. ALVR needs to keep running.

Check that ALVR can see SteamVR: in the dashboard under **Connections**, the SteamVR status should say **Running** (if SteamVR is already open) or **Ready** (if not launched yet).

---

### Step 2 — Connect Quest 2

1. Put the Quest 2 on your head.
2. In the Quest 2 home menu, navigate to **App Library** → **Unknown Sources** → launch **ALVR**.
3. The ALVR client will scan for the streamer on the local network. Your PC should appear by hostname or IP address.
4. Tap **Connect** (or **Trust**) on the headset.
5. Back on the PC: the ALVR dashboard shows the headset as **Connected** with signal strength and latency.
6. SteamVR launches automatically (or launch it manually if it doesn't: `~/.steam/steam/steamapps/common/SteamVR/bin/vrstartup.sh`).
7. In the Quest 2 you should now see the SteamVR home environment (a grey grid room). Controllers should show up as floating hands.

**Troubleshooting connection:**
- Make sure PC and Quest 2 are on the **same 5 GHz Wi-Fi network** (not 2.4 GHz — too slow).
- Disable the PC firewall temporarily if the headset cannot find the streamer: `sudo ufw disable`
- Re-enable after confirming: `sudo ufw enable`

---

### Step 3 — Launch SteamVR

SteamVR usually auto-launches when ALVR connects. Confirm it is running:
- Look for the SteamVR status window on the PC (small floating window with green icons for headset and both controllers).
- All three icons (HMD + left controller + right controller) should be green.

If any controller shows as grey/off: in the Quest 2, press any button on that controller to wake it up.

Leave SteamVR running. Do not close it.

---

### Step 4 — Run the teleop script

Open a new terminal (keep ALVR and SteamVR running in the background):

```bash
source activate_teleop.sh
~/isaacsim/python.sh quest2_teleop_v14.py --run_vr --robot franka
```

The terminal will print:

```
[OK] Isaac Sim: /home/arun/isaacsim
[OK] Kit: isaacsim.exp.full.kit
[INIT] Launching SimulationApp …
[INIT] SimulationApp ready.
[VR] XR origin placed at (0, -1.5, 0) with yaw op ready
[VR] Locomotion settings applied at launch.
[MAIN] world.reset() #1 …
[MAIN] Warming up renderer …
  warmup 0/60 …  warmup 10/60 …  (takes 10–15 s)
[MAIN] world.reset() #2 …
[Robot] ArticulationController ready (gripper will use it)
[MAIN] Robot 'franka' ready — 9 DOF
[MAIN] Simulation loop running.
  Isaac Sim GUI → VR tab → Select SteamVR → Click Start VR
```

The Isaac Sim GUI window appears on your monitor. Wait for the warmup to finish before proceeding to Step 5.

---

### Step 5 — Start VR in Isaac Sim GUI

This step connects Isaac Sim's renderer to SteamVR so the headset sees the simulation.

1. In the Isaac Sim GUI window, look at the top menu bar.
2. Click **VR** (or find it under **Window** → **Extensions** → search `XR`).
3. In the VR panel that opens:
   - **Runtime:** select **SteamVR**
   - Click **Start VR** (or **Enable VR**)
4. Wait 5–10 seconds. You should see the simulation scene appear inside the Quest 2 headset.

The terminal will now print VR controller state every second:

```
[VR] IVRInput interface: IVRInput_010
[VR] Action set handle: 0x1234abcd
[VR] handle OK : right_trigger_value = 0xabc1
[VR] handle OK : right_squeeze_value = 0xabc2
...
[VR] ✓ ACTIVE — 9 action handles

[VR] RT=0.00 GRIP=off(raw=0.000) LT=0.00 RX=0.000 RY=0.000 LX=0.000 LY=0.000 A=0 B=0 X=0 Y=0
```

---

### Step 6 — Begin teleoperation

You are now live in the simulation. Use the Quest 2 Touch controllers:

- Look at the robot arm in front of you (default position: origin).
- Use the thumbsticks to move the arm joints.
- Squeeze the right grip button to open/close the gripper.
- Use A/B/X/Y to rotate your viewpoint if the robot is out of frame.
- Left trigger resets the arm to home pose.

The terminal shows live joint state:

```
[000340] joints=[-0.12 -0.78 +0.00 -2.36 +0.00 +1.57] grip=OPN
```

---

## Controller Reference

### Franka Panda (9 DOF)

| Input | Action |
|-------|--------|
| Right thumbstick ← → | Joint 0 — base rotation |
| Right thumbstick ↑ ↓ | Joint 1 — shoulder flex |
| Left thumbstick ← → | Joint 2 — elbow |
| Left thumbstick ↑ ↓ | Joint 3 — wrist 1 |
| Right trigger (hold) + move controller | Fine Cartesian control (joints 0, 1, 3) |
| Right GRIP / squeeze | **Toggle gripper open ↔ closed** |
| Left trigger | Reset arm to home pose |
| **A button** (hold) | Rotate view LEFT |
| **B button** (hold) | Rotate view RIGHT |
| **X button** (hold) | Tilt view UP |
| **Y button** (hold) | Tilt view DOWN |

### Keyboard fallback (no VR / `--no run_vr`)

| Key | Action |
|-----|--------|
| W / S | ± X axis |
| A / D | ± Y axis |
| Q / E | ± Z axis |
| G | Toggle gripper |
| R | Reset to home |
| ESC | Quit |

---

## CLI Arguments

```bash
~/isaacsim/python.sh quest2_teleop_v14.py [OPTIONS]
```

| Argument | Default | Description |
|----------|---------|-------------|
| `--run_vr` | off | Enable VR mode (Quest 2 input + XR rendering). Without this flag the script runs in keyboard-only mode. |
| `--robot` | `franka` | Which robot to load. Choices: `franka`, `ur5` |
| `--isaac_path` | auto | Path to Isaac Sim install directory. Auto-detected from `ISAACSIM_PATH` env var if not set. |
| `--headless` | off | Run without a display window (useful for remote/SSH sessions, but VR requires a display). |
| `--no_locomotion` | **on** | Disable Isaac Sim XR thumbstick locomotion so thumbsticks only drive joints. Keep this on — turning it off lets SteamVR consume thumbstick input for camera movement. |

**Examples:**

```bash
# Franka with VR (most common)
~/isaacsim/python.sh quest2_teleop_v14.py --run_vr --robot franka

# UR5 with VR
~/isaacsim/python.sh quest2_teleop_v14.py --run_vr --robot ur5

# Keyboard-only test (no headset needed)
~/isaacsim/python.sh quest2_teleop_v14.py --robot franka

# Explicit Isaac path
~/isaacsim/python.sh quest2_teleop_v14.py --run_vr \
    --isaac_path /opt/isaacsim-4.5.0
```

---

## Architecture

```
┌─────────────────────────────────────────────────────┐
│                quest2_teleop_v14.py                  │
│                                                      │
│  VRControllerReader                                  │
│  ┌──────────────────────────────────────────────┐   │
│  │ libopenvr_api.so (ctypes, no Python binding) │   │
│  │  IVRInput_010 vtable                         │   │
│  │   slot 4 → UpdateActionState()               │   │
│  │   slot 6 → GetAnalogActionData()             │   │
│  │   slot 7 → GetDigitalActionData()            │   │
│  │  IVRSystem_022 vtable                        │   │
│  │   slot 9 → GetControllerState() [fallback]   │   │
│  │                                              │   │
│  │  _Hysteresis  — debounce grip squeeze        │   │
│  │  digital_dual — try digital + analog paths   │   │
│  └──────────────────────────────────────────────┘   │
│                                                      │
│  JointController                                     │
│  ┌──────────────────────────────────────────────┐   │
│  │ _apply_arm()    → art.set_joint_positions()  │   │
│  │ _apply_gripper_direct()                      │   │
│  │   → ArticulationController.apply_action()    │   │
│  │      ArticulationAction(joint_positions,     │   │
│  │        NaN mask except indices 7 & 8)        │   │
│  └──────────────────────────────────────────────┘   │
│                                                      │
│  rotate_view()                                       │
│  ┌──────────────────────────────────────────────┐   │
│  │ 1. carb settings /xr/world/originYaw+Pitch   │   │
│  │ 2. USD XROrigin RotateZ op (desktop view)    │   │
│  │ 3. TranslateOp nudge (last resort)           │   │
│  └──────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────┘
           │
    Isaac Sim Physics (PhysX 5)
           │
    Franka Panda articulation
    (9 DOF: joints 0–6 arm, 7–8 fingers)
```

**Why ctypes instead of OpenVR Python bindings?**
The `pyopenvr` package is not bundled with Isaac Sim's Python and installing pip packages into the Isaac Sim Python environment can break it. Direct ctypes avoids any dependency.

**Why two separate gripper and arm paths?**
Franka's finger joints (indices 7 and 8) are mimic-constrained in the USD asset. Isaac Sim's physics solver ignores `set_joint_positions()` writes to them silently. The only correct way to drive them in Isaac Sim 4.x is via `ArticulationController.apply_action()` with an `ArticulationAction`.

---

## Troubleshooting

**"Cannot find Isaac Sim 4.5" at startup**

```bash
export ISAACSIM_PATH=/path/to/your/isaacsim
~/isaacsim/python.sh quest2_teleop_v14.py --run_vr
# or use --isaac_path flag
```

**VR reader stuck at "retrying next frame" — never goes ACTIVE**

- Confirm SteamVR is running (check its status window on desktop).
- Confirm you clicked **Start VR** in Isaac Sim's VR panel (Step 5).
- The IVRInput action set `/actions/ov` is registered by `omni.kit.xr.system.steamvr`. Isaac Sim must be in VR mode before handles resolve.
- Try waiting longer — handle registration can take up to 60 seconds on first launch.

**Thumbstick still rotating camera (locomotion not fully disabled)**

Isaac Sim's XR profile can re-enable locomotion after a VR session reconnect. The script re-asserts the lock every 300 frames (~5 s). If it still moves:

```bash
# Add this flag at launch (already default, but make sure it's there)
~/isaacsim/python.sh quest2_teleop_v14.py --run_vr --no_locomotion
```

**A/B/X/Y buttons detected in terminal (A=1) but view is not rotating**

The carb setting path for view rotation varies between Isaac Sim builds. Check which path your build uses:

```python
# Run inside Isaac Sim's Python console (Extensions → Script Editor)
import carb
s = carb.settings.get_settings()
for path in ["/xr/world/originYaw", "/xr/profile/vr/originYaw", "/xr/origin/yaw"]:
    try:
        s.set_float(path, 0.5)
        print(f"WORKS: {path}")
    except:
        print(f"FAIL:  {path}")
```

If all fail, the XROrigin USD RotateZ rotation (desktop viewport) will still work as visual feedback.

**Gripper not moving**

Check terminal output after pressing grip:

```
[Robot] Gripper → CLOSED (0.000m)
```

If the print appears but the fingers don't move in the viewport, the `ArticulationController` may have failed to initialize. Look for this line at startup:

```
[Robot] ArticulationController ready (gripper will use it)
```

If instead you see `ArticulationController unavailable`, it fell back to `set_joint_positions` which does not work for Franka fingers. This means `omni.isaac.core.controllers` was not importable — confirm your Isaac Sim 4.5 install is complete.

**Quest 2 controllers show as grey in SteamVR (not tracked)**

- Press any button on the controller to wake it.
- In ALVR dashboard → **Connections** → verify both controllers show green dots.
- Check battery level on the controllers.

**High latency / judder in headset**

- Use a 5 GHz Wi-Fi band, not 2.4 GHz.
- In ALVR dashboard → **Video** → reduce **Bitrate** to 100 Mbps if your router struggles.
- Close other applications consuming GPU (browsers with hardware acceleration, etc.).
- In Isaac Sim, switch rendering to **RayTracing** instead of Path Tracing for higher FPS.

**"No .kit files in apps/" error**

Your `ISAACSIM_PATH` points to the wrong directory. It should be the root of the Isaac Sim install (the one that contains `apps/`, `python_packages/`, `python.sh`).

---

## Known Limitations

- **Right controller position tracking** (`right_pos()`) is not yet wired — it always returns `None`. The physical movement → joint control path uses the delta tracker but needs pose data from the IVRSystem pose API (planned for v15).
- **UR5 gripper** has no dedicated end-effector in this script — grip toggle is a no-op for UR5.
- **Pitch rotation** (`/xr/world/originPitch` carb setting) availability depends on Isaac Sim 4.x minor version. Confirmed working on 4.5.0; may need key path update on 4.2 or earlier.
- **Left controller input** (buttons, grip) is not exposed — only right controller squeeze/trigger and both thumbsticks are used.
- Tested only on **Ubuntu 22.04**. Windows is not supported because `libopenvr_api.so` is Linux-specific (would need `.dll` path logic).

---

## License

MIT — see `LICENSE`.

---

## Acknowledgements

- NVIDIA Isaac Sim team — articulation and XR documentation
- ALVR project — https://github.com/alvr-org/ALVR
- Valve OpenVR — IVRInput_010 interface specification
