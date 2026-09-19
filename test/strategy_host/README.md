# Strategy host tests

Run `sh test/strategy_host/run.sh` from the repository root. Requires a C++17
compiler with address and undefined-behaviour sanitizer support.

Compiles the production strategy, PID and vector code against simulated hardware.
Checks return directions, four box boundaries, shuffling, edge slowdown, yaw
compensation, stale data, startup assignment, response thresholds, return lockout,
delayed packets, handoff counter wrap, solo attack and communication recovery.
Physical dynamics and radio delivery
require testing on the robots.

Also tests preset input debounce, all single/double mappings, the 500 ms boundary,
held buttons, latest-selection precedence, gameplay input lock, reset to null,
unsigned timer wrap, both-attack propagation/latching, and recovery for a new run.
Odometry tests exercise actual wrapper code with a simulated OTOS and verify
field-coordinate conversion, readback, boundary alignment and reset.

The boundary-tracking regression uses the production colour sensor, odometry,
and strategy code: a rear white-line hit must preserve X, remain inside the goal
box, and allow subsequent left/right tracking. Cardinal/rotated boundary tests
check which coordinate is corrected and reject ambiguous diagonal observations.
