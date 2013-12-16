Orocos-RTT / Gazebo Integration
===============================

This repository contains tools for prototyping robot control systems designed
with the [Orocos Toolchain](http://orocos.org) in the [Gazebo
Simulator](http://gazebosim.org).

***NOTE:*** *The Orocos Toolchain packages (RTT amd OCL) need to be built with
the following CMake arguments to enable use of the remote console:*

```
-DENABLE_CORBA=ON -DCORBA_IMPLEMENTATION=OMNIORB
```

## Packages

* [rtt\_gazebo\_plugin](rtt_gazebo_plugin) A *Gazebo* plugin for running an RTT
  deployment component inside a Gazebo process.
* [rtt\_gazebo\_activity](rtt_gazebo_activity) An *Orocos RTT* plugin for
  executing RTT components based on Gazebo's simulation time whether they are
  inside or outside the Gazebo process.
* [rtt\_gazebo\_console](rtt_gazebo_console) A CORBA-based console for
  connecting to an Orocos TaskBrowswer running in the Gazebo Simulator.

## Usage


