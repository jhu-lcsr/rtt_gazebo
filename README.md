Orocos-RTT / Gazebo Integration
===============================

This repository contains tools for prototyping robot control systems designed
with the [Orocos Toolchain](http://orocos.org) Real-Time Toolkit (RTT) in the
[Gazebo Simulator](http://gazebosim.org). The goal is to make it easy to run
the exact same collections of RTT components in simulation as are run on real
hardware.

## Building

***NOTE:*** *The Orocos Toolchain packages (RTT amd OCL) need to be built with
the following CMake arguments to enable use of the remote console
(`rtt_gazebo_console`):*

```
-DENABLE_CORBA=ON -DCORBA_IMPLEMENTATION=OMNIORB
```

## Packages

* [rtt\_gazebo\_deployer](rtt_gazebo_deployer) A Gazebo `ModelPlugin` for
  running an RTT deployment component associated with a given Robot Model. This
  deployment component can then be used to load other RTT components. This will
  also instantiate an RTT TaskBrowser associated with each Gazebo Model.
* [rtt\_gazebo\_console](rtt_gazebo_console) A CORBA-based console for
  connecting to an Orocos TaskBrowswer running in the Gazebo Simulator (via
  `rtt_gazebo_deployer`).
* [rtt\_gazebo\_examples](rtt_gazebo_examples) Example RTT components which can
  interact with Gazebo.

See the README.md files in each package for more details.

## Theory of Operation

In order to simulate a piece of hardware, you will write a custom end-point
component (like a driver) which will expose the same RTT interfaces as your
actual hardware. Instead of calling a lower-level API for reading the device,
this component will define a pair of RTT operations which will be called each
time Gazebo's simulation engine updates.

In this callback, the compoents can query any information about the world, and
store it as necessary. Note that this execution path is _different_ than the
way in which the component interacts with Orocos/RTT. 

This component will be associated with a Gazebo SDF model, and when the SDF is
loaded, the `rtt_gazebo_deployer` Gazebo model plugin will create an RTT
`DeploymentComponent` in which the simulation component is instantiated. This
will allow you to load numerous other components as its peers. 

These deployers can be accessed via the `rtt_gazebo_consol` over the standard
RTT CORBA deployer interfaces. This enables quick and easy introspection into
components which are technically instantiated _in_ the Gazebo process.

In order for your periodic tasks to run at the right rates, you can use the
`SimClockActivity` in the `rtt_rosclock` package. When loading a component
in the `rtt_gazebo_deployer`, RTT's system clock will be driven by the Gazebo 
clock and `SimClockActivity` activities will run at rates which observe this
simulated time. 

Note that if you want to run your components out-of-process and communicate
with Gazebo asynchronously over ROS topics, you can still use `rtt_rosclock`
and communicate with Gazebo

For more usage details, see the README.md files in each package in this
repository.

## Performance (thorough analysis forthcoming)

Obviously, when using the tools in this package to run RTT components
synchronously in the Gazebo process, you are able to force the system to enable
you to read and write as fast as is desirable. This means that a collection of
RTT components can _slow_ the Gazebo simulation to the point where they have
enough time to compute each control update. This can be useful in cases where
the simulation machine is computationally underpowered compared to the real
system.

The expectation is that running RTT components in-process enables lower
latencies and higher closed-loop control frequencies. A close analysis of
such performance has yet to be performed.
