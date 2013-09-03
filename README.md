Orocos-RTT / Gazebo Integration
===============================

This repository contains tools for prototyping robot control systems designed
with the [Orocos Toolchain](orocos.org) in the [Gazebo
Simulator](gazebosim.org).

***NOTE:*** *The Orocos Toolchain packages (RTT amd OCL) need to be built
with the following CMake arguments to enable use of the remote console:*

```
-DENABLE_CORBA=ON -DCORBA_IMPLEMENTATION=OMNIORB
```

## Packages

* [rtt\_gazebo\_plugin](rtt_gazebo_plugin) A Gazebo plugin and RTT interface
  for interfacing RTT components with Gazebo
* [rtt\_gazebo\_console](rtt_gazebo_console) A console for connecting to an
  Orocos TaskBrowswer running in the Gazebo Simulator.

