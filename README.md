# Motor Protection Package

## Description
This Package is designed as a niche replacement for the comms heartbeat package. It is entirely run rover side, and sends a zero command to motors if a joystick or cmd_vel command has not been registered in the last half a second. This prevents the motors from continuing to turn in the event of a partial or complete comms loss.

## Known Limitations
There are no known limitations of this package, it was used at competition in conjunction with the drivetrain and manipulator control packages and no adverse behavior was seen.

Adjustments could be made to allow for this package to begin small routines in the event of a total or partial comms loss, and this could be paired with an updated comms heartbeat package to allow for more informative control. (Reflex vs High Level Control)