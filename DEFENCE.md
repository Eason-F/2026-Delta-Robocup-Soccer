# Defence strategy

`defend()` calls `checkDefenceStage()` on every movement update:

- `RETURN`: proportional position control toward the nearest point inside the
  inset goal box.
- `SHUFFLE`: track the ball laterally while correcting inward near any box edge.
  Heading correction keeps the robot facing field forward.
- `PASSIVE`: zero translation when ball data is missing (250 ms) and the robot
  is safely inside the inset box.

The full box is X = [-450, 450], Y = [-915, -615] mm. The default 50 mm inset
keeps tracking inside X = [-400, 400], Y = [-865, -665]. Inside the full box but
outside the inset, inward correction is combined with lateral ball tracking.
Inside the safe Y band, no forward/backward motion is commanded: there is no
fixed Y target. Outside the full goal box, return takes priority at up to 270 RPM,
slowing as the robot approaches. Clearance is measured from
the odometry reference point; tune it for robot size, momentum and sensor error.
Field boundary escape retains priority over strategy motion.

White-line localization maps colour-sensor angles (0 forward, +90 right) into
field coordinates using the current yaw. Front/rear contact corrects only Y;
left/right contact corrects only X. The other coordinate is preserved. Ambiguous
diagonal or cancelling detections still trigger escape but do not relocate the
odometry pose. At heading zero a rear-line hit gives Y = -815, keeping the robot
inside its defence area so lateral tracking can resume after escape.

## Starting roles and handoffs

Choose a starting position with the start switch off. Preset buttons on pins
12, 13 and 14 are active-low (`INPUT_PULLUP`), debounced for 25 ms.

| Pin | Single press | Double press within 500 ms |
| --- | --- | --- |
| 12 | (0, -150) | Both robots attack for the run; keeps any selected position |
| 13 | (0, -615) | Friendly goal-box top left: (-450, -615) |
| 14 | (0, -815) | Friendly goal-box top right: (450, -615) |

Singles register after the 500 ms window; doubles register on the second debounced
press. A held button does not repeat. The most recent button wins, including over
an older pending single. A later position selection clears the explicit both-attack
choice. Simultaneous presses are processed in pin order, with the higher pin last.

Inputs during gameplay are ignored. If the start switch turns on with a click
pending, motion waits for its original 500 ms window to finish; further button
presses are ignored. Turning the start switch off clears the position to null and
clears the local run override. Select a new preset during reset to give the next
run a known position. A button held from gameplay must be released and pressed
again. Presets are not saved through a power cycle.

Positions are applied to odometry when selected and again at gameplay start, using
field units converted to the sensor's calibrated scale. The old repeated idle
odometry reset has been removed. Logs show `preset` (position validity) and
`bothAttack` (explicit or latched run override), alongside odometry X/Y.

Larger starting Y becomes attacker; smaller Y becomes defender. Two equal valid Y
positions wait for a distinguishable assignment. Roles are not locked in while
idle because the presets can still change. Both robots must face their usual
forward reference when starting; the IMU yaw origin is reset while stopped.

If either robot starts with a null preset, or with the pin-12 double-press override,
both robots remain attackers for that run. The override is sent in teammate
packets and latched by a running receiver, so it survives subsequent communication
loss. Stop both robots and choose valid positions to restore normal roles for a
new run. A null preset is tracked separately from a numeric (0,0) sensor origin;
it never qualifies a robot to defend.

Only a defender inside all four full-box boundaries can initiate a swap:

1. Its fresh ball strength is at least 30 and bearing is within +/-60 degrees.
2. Or the attacker reports a fresh signal below 45 with a robot-relative ball bearing more than
   130 degrees from its forward direction. This approximates a distant ball behind
   the attacker; strength-to-distance calibration is not available yet.

There is no role-switch debounce. The defender changes immediately, and the
teammate adopts the opposite role on packet receipt. The new defender must return
to the box before another coordinated swap. If no packet has ever arrived, or
the latest packet is older than `ScoreConfigs::COMMUNICATION_TIMEOUT_MS` (500 ms),
the robot defaults to attack even if it was returning to the goal box. Attack
state is not restarted on each offline update. When communication returns, the
robots re-establish complementary roles; two offline attackers use field Y again.

Both robots need this firmware. The 14-byte packet size is unchanged; flags now
contain role-initialized (bit 0), fresh-ball (bit 1), preset-valid (bit 2),
both-attack (bit 3), gameplay-active (bit 4), and a wrapping three-bit handoff
counter (bits 5-7). The radio bridge must preserve flags. Ordinary packets
are sent at 50 Hz; flag changes bypass that limit. Old handoff counters are ignored.

## Tuning

Edit `DefenceConfig` in `src/strategy/strategy.hpp`:

| Constant | Default | Meaning |
| --- | --- | --- |
| `RESPONSE_HALF_ANGLE_DEG` | 60 | Response cone half-width |
| `RESPONSE_MIN_STRENGTH` | 30 | Minimum response strength |
| `ALIGNMENT_DEADBAND_DEG` | 20 | Centred-ball tolerance |
| `BOX_INSET_MM` | 50 | Clearance from goal-box edges |
| `EDGE_SLOWDOWN_MM` | 100 | Lateral slowdown distance |
| `RETURN_GAIN` | 3.0 | Return gain in RPM/mm |
| `RETURN_MIN_SPD` / `RETURN_MAX_SPD` | 100 / 270 | Return speed limits |
| `BOX_CORRECTION_GAIN` | 1.2 | Inset correction gain in RPM/mm |
| `BOX_CORRECTION_MIN_SPD` | 35 | Minimum inward correction speed |
| `SHUFFLE_GAIN` | 3 | RPM per degree outside deadband |
| `SHUFFLE_MAX_SPD` | 100 | Lateral speed limit |
| `ATTACKER_BEHIND_ANGLE_DEG` | 130 | Robot-relative overshoot bearing threshold |
| `ATTACKER_FAR_STRENGTH` | 45 | Overshoot strength upper limit |

The UART IR input uses degrees, not receiver IDs. The implementation follows
the drive convention: 0 forward, positive right, negative left. Verify the IR
bridge and IMU signs physically. Host tests check commanded behaviour rather
than physical stopping distance or sensor calibration.

## Return-speed diagnostics

Defender return commands up to 270 RPM with a 3 RPM/mm gain and 100 RPM minimum.
The attacker SEARCH state also uses 270 RPM when returning toward the friendly
goal-box centre (0, -765) without a fresh ball reading (`AttackConfig::SEARCH_SPD`).
It no longer targets the rear white line at Y = -915. Normal in-box
tracking and edge correction retain their separate lower limits.

Serial logs include `role`, numeric `stage`, `driveRPM` (translation command),
`m1RPM` (last measured motor-1 speed), `edgeEscape`, `moveDeg`, `ballDeg`, `ballStr`,
and `ballAgeMs`. Attack stages are
0 SEARCH, 1 APPROACH, 2 ORBIT, 3 TRANSITION, 4 CAPTURED; defence stages are
0 PASSIVE, 1 RETURN, 2 SHUFFLE. Boundary escape overrides strategy at 80 RPM.
Translation RPM is projected onto each wheel, so straight backward at 270 gives
about 191 RPM per wheel before heading correction; it is not a measured ground
speed. Logs continue during boundary escape to expose that override.
