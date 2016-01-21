# Tasks and scheduling #

There are four cyclic tasks running in the SACT firmware.

  * **current control** loop: this task is the most frequent and the one with the highest priority. It is associated to the ADC Interrupt Service Routine, because the A/D conversion is periodic and is triggered by the PWM timer and the end of its period. Thanks to the configuration of PWM pins with inverted polarity, this instant corresponds also to the peak value of the current feedback signal. This trick allows also to filter out the noise imposed by PWM switching of the H-Bridge power converter.
  * **velocity/position/cartesian** loops: this task is associated to the Timer5 Interrupt Service Routine.
  * **medium event handler**: this task is executed in a while(1) loop of the main() function, when a counter incremented by the PWM Interrupt Service Routine reaches a given value (corresponding to a period of 20 ms). This execution method makes the task "soft" real-time (it can be interrupted by any Interrupt Service Routine), therefore it is related to non-critical operations, like commands management or board diagnostics.
  * **slow event hanlder**: this task is similar to the previous one, but it is executed with a period of 100 ms. It manages control mode switching (e.g. from OFF to TORQUE MODE) and the trasmission of diagnostic and sensor information packets on active communication channels.

The following paragraphs show the full call graphs of the tasks, together with the parsing of commands received from a master
system.

## Main function and soft real-time tasks ##

![http://sact-unife.googlecode.com/svn/wiki/images/Main_callgraph_files.png](http://sact-unife.googlecode.com/svn/wiki/images/Main_callgraph_files.png)

## ADC Interrupt Service Routine and current control loop ##

![http://sact-unife.googlecode.com/svn/wiki/images/ADC_callgraph_files.png](http://sact-unife.googlecode.com/svn/wiki/images/ADC_callgraph_files.png)

## Timer5 Interrupt Service Routine and higher-level control loops ##

![http://sact-unife.googlecode.com/svn/wiki/images/T5_callgraph_files.png](http://sact-unife.googlecode.com/svn/wiki/images/T5_callgraph_files.png)

## SACT Protocol commands reception management ##

![http://sact-unife.googlecode.com/svn/wiki/images/RX_callgraph_files.png](http://sact-unife.googlecode.com/svn/wiki/images/RX_callgraph_files.png)