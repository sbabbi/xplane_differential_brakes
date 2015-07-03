# xplane_differential_brakes
An X-plane plugin that emulates differential brakes using the joystick yaw input.

Bind a key to the command **sim/flight_controls/both_brakes** to control the brakes. 
While braking, if the yaw command from the joystick is full left or full right, only the corresponding side will break.
Otherwise if the joystick is centered both brakes will be used.

Unlike the other xplane brake commands, this command will not be considered as parking brakes.
