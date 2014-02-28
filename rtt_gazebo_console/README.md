RTT Gazebo Console
==================

This package builds a single executable which connects to the RTT
TaskContextServer running in the RTT Gazebo plugin. The task browser
communicates with the RTT components inside of Gazebo over CORBA.

## Usage

The console can be run easily with rosrun:

```
rosrun rtt_gazebo_console console
```

Once the console is up, it operates like the normal Orocos CORBA deployer.
