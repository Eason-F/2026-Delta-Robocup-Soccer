# RoboCup Junior Soccer — Team Tangent

Firmware for Team Tangent's 2026 RoboCup Junior Lightweight Soccer robot. The
project targets a Teensy 4.1 and is built with PlatformIO using the Arduino
framework.

## What the firmware does

The main control loop combines ball tracking, field-edge detection, heading
control, optical odometry, four-wheel holonomic drive control, and telemetry.
`Robot` owns the hardware modules and coordinates them through four high-level
ball-handling states:

- `SEARCH`: return toward a known field position while no ball is visible.
- `APPROACH`: drive toward the detected ball.
- `ORBIT`: move around the ball until it is aligned with the target heading.
- `CAPTURED`: drive forward while keeping the ball centred.

Some strategy calls in `Robot::run()` are currently commented out for hardware
testing. The checked-in loop updates sensors, holds the robot's heading, sends
telemetry, and logs odometry.

## Hardware and data flow

| Module | Interface | Responsibility |
| --- | --- | --- |
| Teensy 4.1 | Main controller | Runs the control loop and motor PID controllers |
| Four drive motors | PWM + quadrature encoders | Holonomic translation and rotation |
| BNO055 IMU | `Wire2` / I2C | Absolute and relative yaw |
| SparkFun Qwiic OTOS | `Wire` / I2C | Robot position and heading |
| Four colour modules | Digital inputs | White boundary-line detection |
| IR/communications controller | `Serial4` / UART | Ball measurements and robot-to-robot packets |
| USB serial | `Serial` at 115200 baud | Logs and Teensy crash reports |

UART frames use two marker bytes (`A5 5A`), followed by packet type, payload
length, sequence number, payload, and a little-endian CRC-16/CCITT. Payloads are
limited to 64 bytes. See `UartPacketTransport` for the framing implementation
and `RobotPacket` for the 14-byte robot telemetry format.

## Repository layout

```text
src/
├── main.cpp                 Arduino entry points
├── Robot.*                  Hardware ownership and match strategy
├── colour/                  Boundary sensor aggregation and debounce
├── communication/           Robot packets and framed UART transport
├── drive/                   Holonomic drive and individual motor control
├── imu/                     BNO055 yaw wrapper
├── ir/                      UART and legacy QikEasy ball sensors
├── odometry/                Qwiic OTOS position tracking
└── util/                    PID, vectors, field geometry, and logging
```

## Build, upload, and monitor

Install [PlatformIO](https://platformio.org/), connect the Teensy 4.1, and run:

```sh
pio run
pio run --target upload
pio device monitor
```

The serial monitor is configured for 115200 baud. Project dependencies are
declared in `platformio.ini` and are downloaded by PlatformIO during the first
build.

## Coordinate and unit conventions

- Field positions are in millimetres, with the origin at field centre.
- Positive X points right; positive Y points toward the opponent goal.
- Public drive directions and headings are in degrees unless named otherwise.
- `Vector` angle/magnitude construction uses radians.
- Time deltas passed to PID and motor methods are in seconds.
- Motor commands use RPM; raw duty-cycle commands use a percentage from -100
  to 100.

## Configuration and tuning

Pin assignments and controller gains live beside the module that uses them.
The main strategy speeds, tolerances, and debounce times are private constants
in `Robot.hpp`; motor and position gains are in `Drive.hpp`. Field geometry is
centralised in `FieldConstants.hpp`.

When changing hardware, check the constructor wiring in `Robot.cpp`, the motor
pin groups in `Drive.hpp`, and the I2C/UART ports listed above. Tune on a raised
or otherwise secured robot first, because uploading firmware may immediately
enable the normal control loop when the active-low start button is pressed.

## Current limitations

- `UartIRSensor::strengthToDistance()` and `QikEasy::strengthToDistance()` are
  placeholders and currently return zero.
- `OpticalOdometry::boundaryAlignOdometry()` is reserved for boundary-based
  position correction and currently performs no correction.
- Sensor setup failures are mostly reported over serial; there is no unified
  fault state yet.
