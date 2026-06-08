# PiCar-X Reference Guide

Version: 2.1.0a1 | Branch: 2.1.x

---

## Directory Structure

```
picar-x/
├── picarx/                  # Main package
│   ├── picarx.py            # Core robot class (Picarx)
│   ├── preset_actions.py    # ActionFlow + choreographed actions
│   ├── voice_assistant.py   # VoiceAssistant wrapper (from robot_hat)
│   ├── llm.py               # LLM interfaces (OpenAI, Ollama)
│   ├── tts.py               # Text-to-speech (Espeak, Pico2Wave, Piper)
│   ├── stt.py               # Speech-to-text (Vosk)
│   ├── music.py             # Music/sound (robot_hat.Music)
│   ├── led.py               # LED control
│   └── utils.py             # Utilities
├── example/                 # 20+ numbered example scripts
├── gpt_examples/            # OpenAI integration examples
├── sounds/                  # .wav sound effect files
└── musics/                  # Background music files
```

Key external dependencies: `robot_hat` (hardware HAL), `vilib` (computer vision).

---

## Core Class: `Picarx` (`picarx/picarx.py`)

```python
from picarx import Picarx
px = Picarx()
```

### Hardware Pin Defaults

| Role | Pins |
|---|---|
| Camera pan servo | P0 |
| Camera tilt servo | P1 |
| Direction steering servo | P2 |
| Left motor direction | D4 |
| Right motor direction | D5 |
| Left motor PWM (speed) | P13 |
| Right motor PWM (speed) | P12 |
| Grayscale sensors | A0, A1, A2 |
| Ultrasonic trigger / echo | D2 / D3 |

### Constructor

```python
Picarx(
    servo_pins=['P0', 'P1', 'P2'],           # cam_pan, cam_tilt, direction
    motor_pins=['D4', 'D5', 'P13', 'P12'],   # left_dir, right_dir, left_pwm, right_pwm
    grayscale_pins=['A0', 'A1', 'A2'],
    ultrasonic_pins=['D2', 'D3'],
    config='/opt/picar-x/picar-x.conf',
)
```

### Angle / Speed Limits

| Parameter | Range |
|---|---|
| `set_dir_servo_angle` | -30 to 30 deg |
| `set_cam_pan_angle` | -90 to 90 deg |
| `set_cam_tilt_angle` | -35 to 65 deg |
| Motor speed | -100 to 100 (negative = reverse) |
| Grayscale ADC values | 0 to 4096 |

---

### Movement Methods

```python
px.forward(speed)               # speed 0-100
px.backward(speed)              # speed 0-100
px.stop()                       # cut both motors
px.set_power(speed)             # same speed to both motors
px.set_motor_speed(motor, speed)# motor=1 (left) or 2 (right), speed -100..100
```

### Servo Methods

```python
px.set_dir_servo_angle(value)   # steering angle, -30..30
px.set_cam_pan_angle(value)     # camera left/right, -90..90
px.set_cam_tilt_angle(value)    # camera up/down, -35..65
```

### Sensor Methods

```python
distance = px.get_distance()            # float, cm (ultrasonic HC-SR04)
gs_data  = px.get_grayscale_data()      # list[3], raw ADC 0-4096

# After setting references:
line_status  = px.get_line_status(gs_data)   # list[3]: 0=line detected, 1=background
cliff_status = px.get_cliff_status(gs_data)  # bool: True if cliff detected
```

### Calibration Methods

```python
px.dir_servo_calibrate(offset)          # float offset stored in conf
px.cam_pan_servo_calibrate(offset)
px.cam_tilt_servo_calibrate(offset)
px.motor_direction_calibrate(motor, value)  # motor 1|2, value 1|-1
px.motor_speed_calibration(value)           # list [left_offset, right_offset]
px.set_line_reference([v0, v1, v2])         # grayscale threshold for line
px.set_cliff_reference([v0, v1, v2])        # grayscale threshold for cliff
```

### System Methods

```python
px.reset()   # stop + center all servos
px.close()   # clean shutdown (stop, reset, close ultrasonic)
```

---

## Action System: `ActionFlow` (`picarx/preset_actions.py`)

Thread-based action queue with optional music integration.

```python
from picarx.preset_actions import ActionFlow
af = ActionFlow(car=px)
af.start()                      # start handler thread
af.add_action("shake head")     # queue one or more actions
af.add_action("honking")
af.wait_actions_done()          # block until queue empties
af.stop()                       # shut down thread
```

### Status Enum

```python
ActionFlow.ActionStatus.STANDBY       # idle
ActionFlow.ActionStatus.THINK         # LLM is thinking
ActionFlow.ActionStatus.ACTIONS       # executing actions
ActionFlow.ActionStatus.ACTIONS_DONE  # queue finished
```

### Built-in Actions

| Key | Description |
|---|---|
| `"forward"` | Move forward ~1 s at speed 5 |
| `"backward"` | Move backward ~1 s at speed 5 |
| `"shake head"` | Pan servo sweep |
| `"nod"` | Tilt servo nod |
| `"wave hands"` | Direction servo wave + camera tilt |
| `"resist"` | Defensive servo gesture |
| `"act cute"` | Vibrating movement |
| `"rub hands"` | Direction servo back-and-forth |
| `"think"` | Complex pan/tilt/direction sweep |
| `"twist body"` | Coordinated motor + servo twist |
| `"celebrate"` | Victory direction + pan gesture |
| `"depressed"` | Sad drooping gesture |
| `"honking"` | Play horn sound |
| `"start engine"` | Play engine-start sound |

---

## AI / Voice Stack

### LLM (`picarx/llm.py`)

Thin wrappers re-exported from `robot_hat.llm`:

```python
from picarx.llm import OpenAI    # cloud inference
from picarx.llm import Ollama    # local Ollama inference
```

### TTS (`picarx/tts.py`)

```python
from picarx.tts import Espeak      # lightest, robotic voice
from picarx.tts import Pico2Wave   # natural voice, offline
from picarx.tts import Piper       # highest quality, offline
```

### STT (`picarx/stt.py`)

```python
from picarx.stt import Vosk        # offline speech recognition
```

### Music (`picarx/music.py`)

```python
from picarx.music import Music
m = Music()
m.music_play('musics/startup.mp3')
m.sound_play('sounds/honk.wav')
m.music_stop()
```

---

## Vision (`vilib`)

Not in the picarx package directly — imported from `vilib`:

```python
from vilib import Vilib
Vilib.camera_start(vflip=False, hflip=False)
Vilib.face_detect_switch(True)       # enable face detection
Vilib.color_detect("red")            # color blob detection
Vilib.qrcode_detect_switch(True)     # QR code detection
Vilib.take_photo("name", "/path/")   # capture still
Vilib.rec_video_run("name", "/path/")# start recording
Vilib.rec_video_stop()
Vilib.camera_close()

# Detection results (updated in-place):
Vilib.detect_obj_parameter['human_n']    # face count
Vilib.detect_obj_parameter['human_x']   # face center x (0-640)
Vilib.detect_obj_parameter['human_y']   # face center y (0-480)
Vilib.detect_obj_parameter['color_x']   # color blob center x
Vilib.detect_obj_parameter['color_y']   # color blob center y
Vilib.detect_obj_parameter['qr_data']   # decoded QR string
```

---

## VoiceAssistant Integration

`VoiceAssistant` comes from `robot_hat`; `picarx/voice_assistant.py` re-exports it.

### `VoiceActiveCar` pattern (from `example/voice_active_car.py`)

Subclass `VoiceAssistant`, override lifecycle hooks:

```python
class VoiceActiveCar(VoiceAssistant):
    def on_start(self): ...         # initialization
    def on_wake(self): ...          # wake word detected
    def before_listen(self): ...    # before mic opens
    def on_heard(self, text): ...   # speech recognized
    def before_think(self, text): ...  # before LLM call
    def before_say(self, text): ...    # before TTS
    def after_say(self, text): ...     # after TTS finishes
    def on_finish_a_round(self): ...   # end of conversation turn
    def on_stop(self): ...          # shutdown

    def parse_response(self, text) -> str:
        # Extract ACTIONS: from LLM response
        # Expected format:
        #   <spoken reply text>
        #   ACTIONS: shake head, nod, celebrate
        ...
```

---

## Example Scripts Index

| Script | What it demonstrates |
|---|---|
| `1.cali_servo_motor.py` | Interactive servo calibration (keyboard) |
| `1.cali_grayscale.py` | Auto-calibrate line/cliff thresholds |
| `2.move.py` | Basic forward/backward + servo sweeps |
| `3.keyboard_control.py` | WASD live control |
| `4.avoiding_obstacles.py` | Ultrasonic obstacle avoidance loop |
| `5.cliff_detection.py` | Grayscale cliff detection |
| `6.line_tracking.py` | Line following algorithm |
| `7.computer_vision.py` | Face/color/QR detection via vilib |
| `8.stare_at_you.py` | Gimbal tracks detected face |
| `9.record_video.py` | Record to USB storage |
| `13.sound_background_music.py` | Play music + sound effects |
| `14.voice_promt_car.py` | TTS output (Espeak / Pico2Wave) |
| `16.voice_controlled_car.py` | Wake word + Vosk command recognition |
| `17.text_vision_talk.py` | Local vision LLM (Ollama + LLaVA) |
| `18.online_llm_test.py` | OpenAI GPT chat integration |
| `19.local_voice_chatbot.py` | Full offline voice chatbot (Vosk + Ollama + Piper) |
| `20.treasure_hunt.py` | Color detection game with TTS |
| `21.voice_active_car_gpt.py` | Full OpenAI-powered intelligent robot |

---

## Configuration File

Persisted at `/opt/picar-x/picar-x.conf` via `robot_hat.fileDB`:

| Key | Default | Description |
|---|---|---|
| `picarx_dir_servo` | 0 | Direction servo calibration offset |
| `picarx_cam_pan_servo` | 0 | Pan servo calibration offset |
| `picarx_cam_tilt_servo` | 0 | Tilt servo calibration offset |
| `picarx_dir_motor` | [1, 1] | Motor direction polarity |
| `line_reference` | [1000,1000,1000] | Grayscale line threshold |
| `cliff_reference` | [500,500,500] | Grayscale cliff threshold |

---

## Quick-Start Snippets

### Obstacle avoidance skeleton

```python
from picarx import Picarx
import time

px = Picarx()
px.set_dir_servo_angle(0)

while True:
    dist = px.get_distance()
    if dist < 20:
        px.stop()
        px.backward(30)
        time.sleep(0.5)
        px.set_dir_servo_angle(30)
        time.sleep(0.5)
    else:
        px.forward(30)
    time.sleep(0.05)
```

### Line following skeleton

```python
from picarx import Picarx

px = Picarx()
while True:
    gs = px.get_grayscale_data()
    status = px.get_line_status(gs)   # [left, mid, right], 0=line
    if status == [0, 1, 0]:           # only center sees line
        px.set_dir_servo_angle(0)
    elif status[0] == 0:              # line on left
        px.set_dir_servo_angle(-15)
    elif status[2] == 0:              # line on right
        px.set_dir_servo_angle(15)
    px.forward(20)
```

### Face tracking skeleton

```python
from picarx import Picarx
from vilib import Vilib

px = Picarx()
Vilib.camera_start()
Vilib.face_detect_switch(True)

while True:
    if Vilib.detect_obj_parameter['human_n'] > 0:
        x = Vilib.detect_obj_parameter['human_x']  # 0-640
        y = Vilib.detect_obj_parameter['human_y']  # 0-480
        pan = (x - 320) / 320 * -90
        tilt = (y - 240) / 240 * -35
        px.set_cam_pan_angle(pan)
        px.set_cam_tilt_angle(tilt)
```
