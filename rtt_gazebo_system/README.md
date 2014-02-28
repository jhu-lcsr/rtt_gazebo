RTT Gazebo System
=================

This package provides a Gazebo system plugin which initializes RTT in the 
Gazebo server process and overrides RTT's normal Time service using
`rtt_rosclock` with Gazebo as a manual time source. This means that not
only will `RTT::os::TimeService` report the simulated time, but also any
`TaskContext` which uses an `rtt_rosclock::SimClockActivity` will be triggered
at the appropriate period.

## Design

Since systems in a simulated world are neither guaranteed nor likely to run in
real-time, we override the normal RTT TimeService and update it at each Gazebo
simulation step so that it is synchronized with Gazebo's simulation clock.
This is done with the `rtt_rosclock` manual clock source.

*However*, without using a custom RTT Activity to run a component (i.e. running a
component with the normal RTT::PeriodicActivity), that component will be
scheduled based on the *wall time*. 

Instead, components that are meant to run periodically and which should run in
accordance with simulation time should be executed by an
`rtt_rosclock::SimClockActivity`.  Note that this requirement is different than
the RTT::SimulationActivity, which runs as fast as possible. 

## Usage

Since this is a Gazebo _system_ plugin, it needs to be passed by name to the
invocation of the Gazebo server. When using `gazebo_ros_pkgs` this can be done
with the following roslaunch markup (where `$(arg world)` is the path to your
Gazebo world file):

```xml
<node name="gazebo" pkg="gazebo_ros" type="gzserver" respawn="false" output="screen"
      args="$(arg world) -s librtt_gazebo_system.so" />
``

See the example in `rtt_gazebo_examples` for more details.

