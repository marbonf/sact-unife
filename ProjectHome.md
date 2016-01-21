# Firmware source code for SaBot (ScAlable roBOT) actuator control board #
A complete control system for a differential-drive mobile robot, based on dsPIC microcontroller.

## Implements ##
  * current/speed/position control loops for two DC motors with incremental encoders.
  * odometry localization
  * cartesian trajectory generation based on nonlinear smoothing filters
  * dynamic feedback linearization