# RoboCup Junior Soccer — Team Delta

Firmware for Team Delta's 2026 RoboCup Junior Lightweight Soccer robot. It
runs on a Teensy 4.1 and is built with PlatformIO using the Arduino framework.

## Overview

The robot uses a four-wheel holonomic drive, infrared ball tracking, colour
sensors for boundary detection, a BNO055 IMU for heading, and a SparkFun Qwiic
OTOS for optical odometry. A framed UART connection carries IR measurements and
robot-to-robot telemetry through an external communications controller.

The main strategy in `Robot` is organised into four ball-handling states:

- `SEARCH`: move to a search position when the ball is not visible.
- `APPROACH`: drive toward the ball.
- `ORBIT`: move around the ball to align with the target heading.
- `CAPTURED`: drive forward while keeping the ball centred.

## Repository structure

```text
src/
├── Robot.*          Main controller and strategy
├── colour/          Boundary sensors
├── communication/   UART framing and robot telemetry
├── drive/           Holonomic drive and motor control
├── imu/             Heading measurements
├── ir/              Ball sensors
├── odometry/        Optical position tracking
└── util/            PID, vectors, logging, and field geometry
```

The serial monitor runs at 115200 baud. Dependencies and the Teensy 4.1 build
configuration are declared in `platformio.ini`.

## Conventions

- Field positions are in millimetres.
- The field centre is the origin; positive X points right and positive Y points
  toward the opponent goal.
- Drive headings are in degrees, while `Vector` polar angles are in radians.
- Control-loop time deltas are in seconds and motor targets are in RPM.
