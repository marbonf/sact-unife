# Supported Control Modes #

The firmware on SACT board has been designed with the aim to ease flexible configuration of the complete robot control system.
In fact, most motion control boards specifically designed for commercial and academic mobile robots accept ONLY
velocity commands, either defined as independent wheel velocities or as coupled translational and rotational velocities.
Motor current or equivalently force/torque commands are commonly not supported, even though they may be useful for the
implementation of control laws based on the robot dynamics. Moreover, in some low-cost applications it would be desirable
to drop the use of an onboard PC and to make the motion control board capable of autonomous robot navigation.

Therefore, the current SACT board can be configured at runtime, by means of specific user commands, as:
  * a dual motor current controller, with independent set-point commands (**TORQUE MODE INDEPENDENT**, _TMI_);
  * a driving force / steering torque controller (**TORQUE MODE DRIVING**, _TMD_);
  * a dual motor speed controller, with independent set-point commands (**VELOCITY MODE INDEPENDENT**, _VMI_);
  * a driving / steering velocity controller (**VELOCITY MODE DRIVING**, _VMD_);
  * an autonomous navigation system, accepting set-points in cartesian coordinates (**CARTESIAN MODE**, _CM_).

## TORQUE MODE (INDEPENDENT) ##

In this control mode, the SACT board accepts set-point commands for both motor currents and regulates them with a closed loop
based on a PID algorithm, whose output is the duty cycle for the PWM command applied to the H-Bridge power converter. In order
to avoid abrupt changes in the PID set-points, the commanded values are smoothed by a moving average digital filter.
The control loop block diagram is therefore the following:

![http://sact-unife.googlecode.com/svn/wiki/images/current-loop.jpg](http://sact-unife.googlecode.com/svn/wiki/images/current-loop.jpg)

## TORQUE MODE (DRIVING) ##

In this control mode (currently not implemented), the board accepts set-points interpreted as a force to apply for translational
robot motion and a torque to apply for rotational motion, converted by the firmware into motor current set-points according to the
kinematic parameters of the robot.

## VELOCITY MODE (INDEPENDENT) ##

In this control mode, the board accepts set-point commands for both motor velocities. These commands are applied to a position
trajectory generator whose output is a ramp motion profile with limited acceleration and velocity equal to the set-point command.
The position profile is then tracked by cascaded control loops for position, velocity and current. In this way, smooth motion
and robust tracking the velocity reference are achieved. The control loop block diagram is therefore the following:

![http://sact-unife.googlecode.com/svn/wiki/images/pos-loop.jpg](http://sact-unife.googlecode.com/svn/wiki/images/pos-loop.jpg)

## VELOCITY MODE (DRIVING) ##

In this control mode (currently not implemented), the board accepts set-points interpreted as the velocity for translational
robot motion and the velocity for rotational motion, converted by the firmware into wheel velocity set-points according to the
kinematic parameters of the robot.

## CARTESIAN MODE ##

In this control mode the SACT board is able to control the robot to track smooth trajectories, interpolating a sequence of
way-points in the cartesian space.
This motion control strategy is based on a nonlinear filtering technique that allows online smoothing of two-dimensional
straight-line paths, like those composed by the lines connecting way-points. The output of the nonlinear smoothing filter is
compatible with both the kinematic constraints of a differential-drive platform (i.e. the unicycle-like "rolling without slipping"),
and the robot dynamics, which limits linear and radial accelerations. The limitation on the latter implies also a bound on the
curvature of admissible trajectories for the mobile robot.

In order to provide such smooth motion the filter generates linear velocity and orientation references, such that the
resulting trajectory approaches the current target way-point by reducing progressively the distance from the target and the
relative orientation between target and trajectory velocity vectors. The two quantities can be calculated according to the so-called
planar pursuit-evasion equations, commonly used in aerospace guidance algorithms. The geometrical and mathematical details of
planar pursuit-evasion are given by the following pictures:

![http://sact-unife.googlecode.com/svn/wiki/images/target-geometry.jpg](http://sact-unife.googlecode.com/svn/wiki/images/target-geometry.jpg)

![http://sact-unife.googlecode.com/svn/wiki/images/pursuit-equations.jpg](http://sact-unife.googlecode.com/svn/wiki/images/pursuit-equations.jpg)

The contraints on linear/angular velocity and accelerations can be satisfied if the inputs of the filter, that are the reference
velocity and orientation, are switched according to a proper target approaching logic. For example, if the current linear velocity
of the generated trajectory is moving away from the target way-point, it is necessary to slow-down (to limit radial
acceleration along a curve) and change smoothly the orientation of the trajectory. If the trajectory is instead oriented towards
the target way-point, so that the motion path is rectilinear, the linear velocity can be set to the maximum value admissible for
the robot. Full details on the nonlinear filtering technique can be found in
[(BONFE-SECCHI, IROS2010)](http://dx.doi.org/10.1109/IROS.2010.5650306), for the application to mobile robotics, and in
[(ZANASI et al., AUTO2000)](http://dx.doi.org/10.1016/S0005-1098(99)00164-8), for the basic one-dimensional case.

The full block diagram of the trajectory generator can be described as follows:
![http://sact-unife.googlecode.com/svn/wiki/images/nlfilter.jpg](http://sact-unife.googlecode.com/svn/wiki/images/nlfilter.jpg)

The trajectory tracking control method implemented on the SACT board is based on feedback linearization of the dynamic model of
the robot. This control strategy derives from the one described in
[(ORIOLO et al., IEEE-CST2002)](http://dx.doi.org/10.1109/TCST.2002.804116), but extended to allow the definition of robot
control inputs at force/torque level. More precisely, the control law is based on the inverse dynamic model, as shown as follows:

![http://sact-unife.googlecode.com/svn/wiki/images/dfl.jpg](http://sact-unife.googlecode.com/svn/wiki/images/dfl.jpg)

and a linear feedback law that exploits as a feedforward action the third-order derivative of the generated trajectory (which is
guaranteed to be bounded).

![http://sact-unife.googlecode.com/svn/wiki/images/lin-loop.jpg](http://sact-unife.googlecode.com/svn/wiki/images/lin-loop.jpg)

The complete control loop for this cartesian mode can be described by the following block diagram:

![http://sact-unife.googlecode.com/svn/wiki/images/cart-loop.jpg](http://sact-unife.googlecode.com/svn/wiki/images/cart-loop.jpg)