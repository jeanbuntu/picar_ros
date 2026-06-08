# PiCar-X Build, Setup & Systems Documentation
*Comprehensive project log — GrayMatter Robotics / Personal Build*
*Session date: May 30, 2026*

---

## Table of Contents
1. [Project Overview](#project-overview)
2. [Hardware Inventory](#hardware-inventory)
3. [Assembly State](#assembly-state)
4. [OS & Environment Decisions](#os--environment-decisions)
5. [Raspberry Pi OS Setup](#raspberry-pi-os-setup)
6. [Network & SSH Access](#network--ssh-access)
7. [Hardware Debugging — Robot HAT](#hardware-debugging--robot-hat)
8. [I²C Verification](#i2c-verification)
9. [SunFounder Module Installation](#sunfounder-module-installation)
10. [Audio Setup](#audio-setup)
11. [Servo Calibration](#servo-calibration)
12. [Systems Check Results](#systems-check-results)
13. [Development Environment](#development-environment)
14. [OS & Stack Decisions — Research Notes](#os--stack-decisions--research-notes)
15. [Known Issues & Open Items](#known-issues--open-items)
16. [Reference — Key Commands](#reference--key-commands)
17. [Next Steps](#next-steps)

---

## Project Overview

**Goal:** Build, configure, and program a SunFounder PiCar-X AI self-driving robot car kit powered by a Raspberry Pi 4B. The near-term objective is a fully verified, calibrated platform running SunFounder's Python stack. The longer-term objective is custom Python control code, potentially with ROS 2 integration.

**Documentation source:** https://docs.sunfounder.com/projects/picar-x-v20/en/latest/

**Kit:** SunFounder PiCar-X v2.0 — AI Self-Driving Robot Car Kit for Raspberry Pi
**Compute:** Raspberry Pi 4B (4GB RAM)
**OS:** Raspberry Pi OS 64-bit (Debian 13 "Trixie")
**Primary language:** Python 3

---

## Hardware Inventory

| Component | Details |
|-----------|---------|
| Chassis | PiCar-X white plastic/aluminum frame, 4-wheel drive |
| Compute | Raspberry Pi 4B |
| HAT | SunFounder Robot HAT (pre-v5) |
| Camera | CSI camera module on 2-axis pan/tilt servo mount |
| Ultrasonic sensor | Front-mounted distance sensor |
| Grayscale sensor | 3-channel line-tracking board, underside mounted |
| Speaker | Mono speaker wired to Robot HAT I2S amplifier |
| Servos | Steering (direction), camera pan, camera tilt |
| Motors | 2x DC motors (left and right drive) |
| Battery | Robot HAT integrated battery pack |
| Storage | MicroSD card (Raspberry Pi OS boot) |

---

## Assembly State

Physical assembly was completed before this session. Observed in photos:

- Robot HAT stacked on Pi 4B via 40-pin GPIO header
- Camera ribbon cable connected to Pi CSI port
- Camera mounted on pan/tilt servo bracket at front
- All four wheels and motors mounted
- Speaker wired to HAT
- Servo wires routed to HAT ports
- Motor connectors attached to HAT

**Known assembly quirks going in:**
- Servo horns were mounted at arbitrary angles (not zeroed during assembly) — expected and normal
- Right motor was disconnected mid-session during HAT power debugging and needed reconnection before drive tests

---

## OS & Environment Decisions

### Why Raspberry Pi OS (not Ubuntu)

Ubuntu was considered but ruled out for this project for a concrete reason: SunFounder's `robot-hat`, `vilib`, and `picar-x` libraries are written and tested against Raspberry Pi OS. They depend on Raspberry Pi-specific pieces including the libcamera/picamera2 camera stack, GPIO and I²C/SPI setup via `raspi-config`, and audio config helpers like the i2samp script. Running these on Ubuntu would require significant manual porting and debugging of the OS layer before the car would ever move.

**Decision: Raspberry Pi OS 64-bit (Trixie) — correct choice for SunFounder stack.**

### What is Trixie?

Trixie is the codename for Debian 13. Raspberry Pi OS is built on Debian. Debian 13 was released August 9, 2025; the matching Raspberry Pi OS release followed in October 2025, running the Linux 6.12 LTS kernel. It is the current default from Raspberry Pi Imager. The Pi 4B is fully supported.

### Can ROS 2 run on Debian 13 / Trixie?

Yes, with caveats. Official ROS 2 apt packages target specific Ubuntu LTS and Debian versions; Trixie is newer than current official targets, so there are no official OSRF apt packages that install cleanly on Trixie. Options:

1. **Community-built Trixie packages (rospian project):** ROS 2 Jazzy built as native Debian arm64 packages for Trixie/Raspberry Pi OS. Adds a third-party apt repo. Best option if going this route.
2. **Build from source:** Always possible via `colcon`. Slow on Pi 4B, memory-tight.
3. **Docker:** Sidesteps OS mismatch but camera/GPIO/I²C passthrough is fiddly.

**Target distro if pursuing ROS 2: Jazzy Jalisco** (current LTS, released May 2024). Kilted Kaiju (May 2025) exists but is non-LTS with a shorter support window.

**Decision: Deferred. Get the car working on SunFounder stack first, add ROS 2 later on a second SD card to preserve the working setup.**

### Raspberry Pi Connect

Raspberry Pi Connect is the Foundation's browser-based remote access service. It works outbound through NAT/CGNAT with no port forwarding required, using WebRTC peer-to-peer. Free for individuals. Provides screen sharing (requires 64-bit OS + Wayland) and remote shell access.

**Decision: Not needed for this project. SSH is sufficient and simpler. Connect can be added later with `sudo apt install rpi-connect` if browser-based camera preview becomes useful.**

---

## Raspberry Pi OS Setup

### Flashing

- Tool: Raspberry Pi Imager
- OS: Raspberry Pi OS 64-bit (Trixie/Debian 13)
- Target: MicroSD card (initial attempt was USB drive; switched to SD after bootloader clarification)
- Imager customization settings applied:
  - Hostname: `picar-jv`
  - SSH: enabled, password authentication
  - Username: `jvpicar`
  - Wi-Fi SSID, password, and country configured
  - Locale/timezone set

### Boot

Pi 4B boots from SD card. Power delivered through Robot HAT battery (see debugging section below for initial issues). Confirmed boot indicators: red LED solid, green LED blinking on Pi board.

### First-boot setup

```bash
# Update all packages
sudo apt update && sudo apt full-upgrade -y
sudo reboot

# Enable I²C and SPI
sudo raspi-config
# → Interface Options → I²C → Enable
# → Interface Options → SPI → Enable
# → Finish → Reboot
```

### Verify I²C bus is active

```bash
ls /dev/i2c*
# Expected output:
# /dev/i2c-0  /dev/i2c-1  /dev/i2c-10  /dev/i2c-20  /dev/i2c-21  /dev/i2c-22
```

`/dev/i2c-1` is the relevant bus for the Robot HAT.

---

## Network & SSH Access

| Parameter | Value |
|-----------|-------|
| Hostname | `picar-jv` |
| mDNS address | `picar-jv.local` |
| Username | `jvpicar` |
| SSH command | `ssh jvpicar@picar-jv.local` |

**Note:** When `ssh picar-jv.local` is run without a username, it defaults to the host machine's username, which will be rejected. Always specify `jvpicar@`.

**Troubleshooting `.local` resolution:** If `picar-jv.local` doesn't resolve, use the IP address from your router's connected-devices list instead. Mac and Linux resolve `.local` natively; Windows requires Bonjour.

---

## Hardware Debugging — Robot HAT

This was the most significant debugging session of the build. Two symptoms appeared simultaneously:

**Symptom 1:** HAT battery powered the HAT (lights on, a motor twitched) but the Pi showed zero LEDs — no power to the Pi from the HAT.

**Symptom 2:** `i2cdetect -y 1` returned all dashes — Pi could not see the HAT on the I²C bus at all.

### Diagnosis process

1. Powered Pi directly via its own USB-C port → red LED on, green LED blinking → confirmed Pi and SD card are fine
2. Ran `i2cdetect -y 0` and `i2cdetect -y 1` → both all dashes → HAT not on any bus
3. Confirmed I²C was enabled (`ls /dev/i2c*` showed `/dev/i2c-1`)
4. Confirmed HAT switch was ON and battery had charge

**Root cause: The Robot HAT was not fully seated on the Pi's 40-pin GPIO header.** A partial or offset seating means the HAT's own circuitry (which runs from the battery directly) worked fine, but the 5V power pins and I²C signal pins (GPIO pins 3 and 5) were not making proper contact with the Pi. This caused both symptoms simultaneously from one physical cause.

### Fix

Powered everything down. Removed HAT from Pi. Inspected alignment — confirmed offset on GPIO header. Reseated HAT straight down firmly onto all 40 pins, ensuring no row/column shift and no standoff holding it proud. Powered back on.

### Verification

```bash
i2cdetect -y 1
```

```
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:                         -- -- -- -- -- -- -- -- 
10: -- -- -- -- 14 -- -- -- -- -- -- -- -- -- -- -- 
...
```

**0x14 appeared — Robot HAT confirmed alive on I²C bus.** HAT now also powers the Pi correctly from battery alone; direct USB-C no longer needed.

### Key lessons

- The HAT powering accessories (motors) does not prove the GPIO header is correctly seated. Power and I²C pins are distinct from motor driver power.
- Both "HAT won't power Pi" and "nothing on I²C" are explained by one root cause: partial GPIO header seating.
- On PiCar-X specifically, standoffs can hold the HAT proud of the header if the standoff is too tall or screwed down before the header is fully seated. Header seating takes priority over standoff screws.
- `i2cdetect -y 1` finding 0x14 is the definitive confirmation the HAT is correctly installed.

---

## I²C Verification

Expected I²C addresses on a correctly assembled PiCar-X:

| Address | Device |
|---------|--------|
| 0x14 | Robot HAT (confirmed present) |

Run at any time to verify HAT is alive:
```bash
i2cdetect -y 1
```

---

## SunFounder Module Installation

All four steps completed successfully in this session. Run on the Pi in this exact order.

### Step 1 — robot-hat

```bash
cd ~/
git clone -b 2.5.x https://github.com/sunfounder/robot-hat.git --depth 1
cd robot-hat
sudo python3 install.py
```

### Step 2 — vilib (vision library)

```bash
cd ~/
git clone https://github.com/sunfounder/vilib.git --depth 1
cd vilib
sudo python3 install.py
```

This is the slowest step. vilib pulls in OpenCV and the full camera library stack. Be patient. Keep the HAT charger plugged in during this step to avoid a mid-compile brownout.

### Step 3 — picar-x

```bash
cd ~/
git clone -b 2.1.x https://github.com/sunfounder/picar-x.git --depth 1
cd picar-x
sudo pip3 install . --break
```

The `--break` flag is required on current Raspberry Pi OS due to PEP 668 (externally-managed-environment enforcement). The matched branch combination `robot-hat 2.5.x` + `picar-x 2.1.x` is intentional — these versions are tested together.

### Step 4 — i2samp (I2S audio amplifier / speaker)

```bash
cd ~/robot-hat
sudo bash i2samp.sh
```

Follow prompts: answer `y` three times. The script:
- Detects Robot HAT version (pre-v5 in this case — harmless warning)
- Configures `/boot/firmware/config.txt` with `dtoverlay=hifiberry-dac`
- Configures `/etc/asound.conf`
- Sets ALSA speaker volume to 100%
- Configures PulseAudio
- Runs speaker test (speaker-test with stereo L/R sweep)

**Note on "No robothat 5 found":** This warning appeared because the HAT is pre-v5. The script handled it gracefully and configured correctly anyway. Not an error.

**Note on "sink index not found":** PulseAudio couldn't find the soundcard before a reboot. Expected. A reboot activates the dtoverlay.

**Note on "Front Right only" during speaker test:** Expected on a mono speaker. The test plays stereo; the single physical speaker only reproduces one channel. Hardware is fine.

After i2samp, always reboot:
```bash
sudo reboot
```

### Verify audio after reboot

```bash
aplay -l
```

Expected output includes:
```
card 1: sndrpihifiberry [snd_rpi_hifiberry_dac], device 0: HifiBerry DAC HiFi pcm5102a-hifi-0
```

**Confirmed present in this session.** Audio system fully operational.

---

## Servo Calibration

### Tool

```bash
cd ~/picar-x/example
sudo python3 1.cali_servo_motor.py
```

### Calibration Helper controls

| Key | Action |
|-----|--------|
| `1` | Select direction (steering) servo |
| `2` | Select camera pan servo |
| `3` | Select camera tilt servo |
| `4` | Select left motor |
| `5` | Select right motor |
| `W` / `D` | Increase servo angle |
| `S` / `A` | Decrease servo angle |
| `R` | Run servos test |
| `Q` | Change motor direction |
| `E` | Motors run/stop |
| `SPACE` | Confirm/save calibration for selected item |
| `Ctrl+C` | Quit |

### Direction servo issue and fix

**Problem:** Front wheels were turned hard right after assembly. Software nudging (`W`/`S`) produced only a slight flutter with no real movement in either direction — the servo was already at its mechanical limit. The horn had been mounted so far off zero that no software offset could reach straight.

**Important:** Do not force servos. Forcing while powered or past mechanical stops strips the gear train.

**Fix procedure:**
1. Quit calibration tool (`Ctrl+C`)
2. Power HAT off (flip switch)
3. Remove small Phillips screw from center of steering servo horn
4. Lift horn straight off the servo spline
5. Power HAT back on, relaunch calibration, press `1`
6. Servo moves to commanded zero position
7. Press horn down onto spline with wheels pointing dead straight
8. Screw horn back in
9. Fine-tune with `W`/`S` in calibration tool
10. Press `SPACE` to save

**Why this works:** The servo's commanded zero and the horn's physical position are independent. The fix aligns them so "zero degrees commanded" equals "wheels straight" in the physical world. The software offset then handles the last few degrees of fine-tuning.

### Calibration results — all five items

| Item | Status |
|------|--------|
| Direction servo (steering) | ✅ Calibrated — physical horn reseat required |
| Camera pan servo | ✅ Calibrated — software adjustment only |
| Camera tilt servo | ✅ Calibrated — software adjustment only |
| Left motor direction | ✅ Calibrated |
| Right motor direction | ✅ Calibrated |

---

## Systems Check Results

### move.py — motors, steering, camera servos

```bash
cd ~/picar-x/example
sudo python3 2.move.py
```

**Result: ✅ PASS**

Car performed a quick automated systems check sequence: drove forward, steered left/right, panned camera, tilted camera. All five actuators confirmed working in one run.

**Important:** Always prop wheels off ground for first move.py run in case motor direction is wrong.

### computer_vision.py — camera and vision

```bash
sudo python3 7.computer_vision.py
```

**Result: ✅ PASS**

Output:
```
vilib 0.3.18 launching ...
picamera2 0.3.36
Local display failed, because there is no gui.
Web display on: http://192.168.1.241:9000/mjpg
Starting web streaming ...
```

"Local display failed" is expected — no GUI on headless Pi. Web stream at `http://192.168.1.241:9000/mjpg` opens in browser on any device on the same network. Camera live and streaming confirmed.

Available in-session controls:

| Key | Function |
|-----|----------|
| `q` | Take photo |
| `1`-`6` | Color detect (red/orange/yellow/green/blue/purple) |
| `0` | Switch off color detect |
| `r` | Scan QR code |
| `f` | Toggle face detection |
| `s` | Display detected object info |

### avoiding_obstacles.py — ultrasonic sensor

```bash
sudo python3 4.avoiding_obstacles.py
```

**Result: ⚠️ FUNCTIONAL but needs environment**

Car reversed continuously — ultrasonic sensor was detecting something within its threshold (default ~30-40cm) immediately in front. This is correct behavior; the car was sitting on its box in a confined space. Not a fault. Test in open floor space with at least 1 meter clear in front.

To verify raw sensor readings:
```bash
python3 -c "
from picarx import Picarx
import time
px = Picarx()
for i in range(10):
    print(px.ultrasonic.read())
    time.sleep(0.5)
"
```

### line_tracking.py — grayscale sensor

```bash
sudo python3 6.line_tracking.py
```

**Status: ⏳ Not yet tested this session**

Requires dark line on light surface (black tape on white paper works). Run car over it and confirm three grayscale sensor values respond.

### Audio

**Result: ✅ PASS** — confirmed via i2samp speaker test and `aplay -l` post-reboot.

---

## Development Environment

### On the Pi

Files live at:
```
~/picar-x/          # main library and config
~/picar-x/example/  # all example scripts
~/robot-hat/        # robot-hat library
~/vilib/            # vision library
```

### On the Mac (host development machine)

**Recommended workflow:** Write and edit code on Mac, deploy to Pi via `scp` or `rsync`, run on Pi via SSH. The Pi is the runtime; the Mac is the development environment. This avoids ARM-specific tooling headaches and is the standard embedded development pattern.

**Copy a file to Pi:**
```bash
scp yourfile.py jvpicar@picar-jv.local:~/picar-x/example/
```

**Sync a whole folder:**
```bash
rsync -avz ./myproject/ jvpicar@picar-jv.local:~/picarx-dev/
```

### Claude Code (Mac)

Installed successfully: Claude Code v2.1.158.

**PATH fix required** (one-time, run on Mac):
```bash
echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.zshrc && source ~/.zshrc
```

Then `claude` launches Claude Code in any directory.

**Why Mac, not Pi:** Claude Code is officially ARM64-supported but has known installer quirks on Raspberry Pi (aarch64 architecture detection bugs, headless OAuth flow issues). Mac installation is clean and avoids all of these. The code Claude Code writes on Mac is deployed to the Pi; no compute happens locally regardless.

### VS Code (optional)

If VS Code is installed, install the **Remote - SSH** extension. Connect to `jvpicar@picar-jv.local` to browse and edit the Pi's filesystem directly from a full editor without scp.

**Install `code` CLI command** (if VS Code installed but command not found):
VS Code → `Cmd+Shift+P` → "Shell Command: Install 'code' command in PATH"

---

## OS & Stack Decisions — Research Notes

### Raspberry Pi Connect vs SSH

Raspberry Pi Connect is the Foundation's browser-based remote access (outbound WebRTC, no port forwarding). Free for individuals. Useful for desktop/camera preview access from outside the home network. For this project, SSH is sufficient. Connect can be added later: `sudo apt install rpi-connect`.

### ROS 2 on this platform

If pursuing ROS 2:
- **Target distro:** Jazzy Jalisco (LTS, May 2024)
- **Installation path on Trixie:** rospian community packages (third-party apt repo for arm64 Trixie)
- **Alternative:** Build from source via colcon (slow on Pi 4B)
- **Recommendation:** Establish working SunFounder baseline first, then add ROS 2 on a second SD card

### Claude Code on Pi (for reference)

Possible but involves workarounds:
- ARM64 OAuth flow has known truncated scope bug
- Native installer has aarch64 detection issues in some versions
- Workaround: authenticate on x86 Mac, copy `~/.claude/.credentials.json` to Pi via scp
- Requires Node.js 20 via NodeSource (not apt default)
- Not recommended as primary dev environment for this project

---

## Known Issues & Open Items

| Issue | Status | Notes |
|-------|--------|-------|
| Right motor disconnected during debugging | ⚠️ Reconnect before drive tests | Was unplugged when it spun on HAT power-up |
| Obstacle avoidance reverses continuously | ⚠️ Environment issue | Test in open space, 1m+ clear in front |
| Line tracking not yet tested | ⏳ Pending | Needs dark line on light surface |
| Claude Code `claude` command not found on Mac | ⚠️ PATH not updated | Run one-line fix in terminal |
| `code .` command not found | ⚠️ VS Code CLI not installed | Install via VS Code command palette |

---

## Reference — Key Commands

### SSH into Pi
```bash
ssh jvpicar@picar-jv.local
```

### Check HAT is on I²C bus
```bash
i2cdetect -y 1
# Look for 0x14
```

### Check audio devices
```bash
aplay -l
# Look for sndrpihifiberry
```

### Run examples
```bash
cd ~/picar-x/example
sudo python3 1.cali_servo_motor.py   # Servo calibration
sudo python3 2.move.py               # Motor/servo systems check
sudo python3 4.avoiding_obstacles.py # Ultrasonic obstacle avoidance
sudo python3 5.cliff_detection.py    # Cliff detection
sudo python3 6.line_tracking.py      # Grayscale line tracking
sudo python3 7.computer_vision.py    # Camera + vision (web stream)
sudo python3 3.keyboard_control.py   # Manual keyboard drive
```

### Raw ultrasonic distance check
```bash
python3 -c "
from picarx import Picarx
import time
px = Picarx()
for i in range(10):
    print(px.ultrasonic.read())
    time.sleep(0.5)
"
```

### Copy file from Mac to Pi
```bash
scp myfile.py jvpicar@picar-jv.local:~/picar-x/example/
```

### Fix Claude Code PATH on Mac
```bash
echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.zshrc && source ~/.zshrc
```

---

## Next Steps

### Immediate
- [ ] Fix Claude Code PATH on Mac (`echo 'export PATH...'`)
- [ ] Reconnect right motor
- [ ] Test obstacle avoidance in open floor space
- [ ] Test line tracking (`6.line_tracking.py`) with black tape on white paper
- [ ] Test keyboard control (`3.keyboard_control.py`) for manual driving

### Short term
- [ ] Explore `keyboard_control.py` for manual driving
- [ ] Try `7.computer_vision.py` face and color detection features
- [ ] Define first custom behavior to build (autonomous patrol, follow a line, object tracking, etc.)
- [ ] Set up Claude Code on Mac for assisted Python development

### Longer term
- [ ] Custom Python control scripts
- [ ] Consider ROS 2 Jazzy on second SD card (rospian packages for Trixie arm64)
- [ ] Explore GPT/LLM integration examples (`gpt_examples/` folder in picar-x repo)

---

*End of session documentation. Car is calibrated and verified operational on motors, steering, camera, and audio. Platform is ready for custom development.*
