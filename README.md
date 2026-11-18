# ESP32 micro-ROS Mechanical Claw Control

C++ firmware using ROS 2 and micro-ROS to control an ESP32-S3-driven
servo that rotates a mechanical claw mechanism to grab and release objects.

The firmware accepts pulse-width commands in microseconds, supports relative
adjustments, and gradually reduces command increments near the target.

## Hardware and software

- Freenove ESP32-S3 WROOM
- REEFS 1100 Smart servo
- External servo power supply
- Windows with WSL2 and Ubuntu 24.04
- ROS 2 Jazzy and micro-ROS Agent
- PlatformIO with the Arduino framework

The firmware uses GPIO14 for the servo signal and serial communication
at 115200 baud through the board's USB-to-UART port.

Connect the servo ground, external supply ground, and ESP32 ground together.
Power the servo from a supply suitable for its rated voltage and current.

## Features

- Absolute pulse-width commands
- Relative pulse-width adjustments
- Gradual movement toward the commanded target
- Smaller movement increments near the target
- Publication of the current commanded pulse width

## ROS 2 interface

Node: `/esp32_servo_node`

All topics use `std_msgs/msg/Int32`.

| Topic | Direction relative to ESP32 | Purpose |
| --- | --- | --- |
| `/servo_pwm_us` | Input | Set the target pulse width in microseconds |
| `/servo_pwm_step_us` | Input | Add a signed increment to the existing target |
| `/servo_pwm_current_us` | Output | Report the current commanded pulse width |

A relative command of `0` requests the current commanded value without
changing the target.

Reported values describe the software command. They are not measurements
of servo position, speed, or the physical PWM waveform.

## Build and upload

Run these commands from the repository directory with PlatformIO installed:

```bash
pio run
pio run --target upload --upload-port /dev/ttyACM0
```

If using the existing PlatformIO virtual environment:

```bash
~/.platformio/penv/bin/pio run
~/.platformio/penv/bin/pio run --target upload --upload-port /dev/ttyACM0
```

Replace `/dev/ttyACM0` with the actual serial device.

This configuration uses the Arduino ESP32 2.x LEDC API supplied by the pinned
PlatformIO platform. Arduino ESP32 3.x uses a different LEDC API.

## Start the micro-ROS Agent

These commands assume ROS 2 Jazzy and a built micro-ROS Agent workspace
at `~/microros_ws`.

```bash
source /opt/ros/jazzy/setup.bash
source ~/microros_ws/install/local_setup.bash

ros2 run micro_ros_agent micro_ros_agent serial \
  --dev /dev/ttyACM0 -b 115200 -v6
```

Keep this terminal running. If necessary, press the ESP32 EN/RST button
after the Agent starts.

When using WSL2, attach the USB-to-UART device to WSL before starting
the Agent. Close any other application using the serial port.

## Send commands

Open another Ubuntu terminal:

```bash
source /opt/ros/jazzy/setup.bash
```

Monitor the current commanded pulse width:

```bash
ros2 topic echo /servo_pwm_current_us
```

In a separate terminal, set a target:

```bash
ros2 topic pub --once /servo_pwm_us \
  std_msgs/msg/Int32 "{data: 1500}"
```

Increase the target by 10 microseconds:

```bash
ros2 topic pub --once /servo_pwm_step_us \
  std_msgs/msg/Int32 "{data: 10}"
```

Decrease the target by 10 microseconds:

```bash
ros2 topic pub --once /servo_pwm_step_us \
  std_msgs/msg/Int32 "{data: -10}"
```

Request the current commanded value:

```bash
ros2 topic pub --once /servo_pwm_step_us \
  std_msgs/msg/Int32 "{data: 0}"
```

Use pulse widths verified for the installed servo and claw mechanism.

## Motion settings

Edit these constants in `src/main.cpp` and upload again:

| Setting | Default | Meaning |
| --- | --- | --- |
| `PWM_INITIAL_US` | 1500 | Pulse width applied immediately at startup |
| `PWM_HZ` | 50 | PWM repetition frequency |
| `PWM_BITS` | 14 | PWM duty resolution |
| `MOTION_INTERVAL_MS` | 20 | Minimum interval between motion updates |
| `MAX_STEP_US` | 10 | Maximum pulse-width change per update |
| `SLOWDOWN_DIVISOR` | 20 | Controls how early command increments decrease |

Increase `SLOWDOWN_DIVISOR` to begin slowing the command ramp earlier.
Decrease `MAX_STEP_US` or increase `MOTION_INTERVAL_MS` for a slower ramp.

This controls the rate of change of the pulse-width command. Actual servo
velocity depends on the servo, load, and mechanism.

## Operating behavior and limitations

- The servo receives `PWM_INITIAL_US` before the Agent connects.
- The software accepts integer pulse widths from 1 to 19999 microseconds.
  This is a signal validity check, not a safe mechanical travel range.
- Calibrate the usable range for the installed mechanism before testing endpoints.
- PWM resolution means that not every 1-microsecond command produces a distinct
  physical output.
- There is no position feedback, grasp detection, or force control.
- The firmware does not implement automatic Agent reconnection or a
  communication-loss stop.
- The PWM output continues at the last commanded value if communication stops.
- This version contains the claw servo controller only.

## Project files

- `src/main.cpp`: micro-ROS node, command callbacks, PWM output, and motion ramp
- `platformio.ini`: board, framework, and micro-ROS build configuration
- `.gitignore`: generated files excluded from Git
