# SaBot and its ACTuator control board #

SaBot (ScAlable RoBOT) is an experimental mobile platform developed from the Laboratory of Intelligent Robotics and Automation (LIRA)
of the University of Ferrara (Italy).

![http://sact-unife.googlecode.com/svn/wiki/images/sabot.jpg](http://sact-unife.googlecode.com/svn/wiki/images/sabot.jpg)

The design of the robot started from scratch with the aim to gain practical experience on mobile robotics, by addressing
all the issues and learn all the lessons related to a full "mechatronic design".

Therefore, most of the mechanics and electronics of SaBot has been realized in the LIRA lab, including in particular the
motor controller called SACT (SaBot ACTuator control board). The SACT board can control two DC motors equipped with quadrature
(incremental) encoders, since SaBot has been designed as a [differential-drive](http://planning.cs.uiuc.edu/node659.html)
robot.

Moreover, since the main features of SaBot are expected to flexibility and "scalability", the SACT board can be used:
  * as a stand-alone control system, programmed with self-localization and navigation algorithms;
  * in conjuction with a PC (that we can call SaBot System Board, SSB) either onboard or remotely connected with a wireless link;
  * in conjuction with other SACT boards connected with a fieldbus (e.g. CAN bus)

A pictorial view of the SaBot architecture can be given as follows:

![http://sact-unife.googlecode.com/svn/wiki/images/sabot-arch.jpg](http://sact-unife.googlecode.com/svn/wiki/images/sabot-arch.jpg)