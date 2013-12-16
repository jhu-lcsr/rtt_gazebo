RTT Gazebo Activity
-------------------

This package provides an Orocos RTT plugin, `rtt_gazebo_service` which provides
a mechanism for executing RTT componetns based on Gazebo's simulation clock for
components loaded inside or outside the Gazebo process. In the case where a
component is out-of-process, the synchronization is done via the ROS `/clock`
topic and enabled if the `/use_sim_time` ROS parameter is set to true.

## Usage
