^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rtt_gazebo_examples
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.1 (2014-12-08)
------------------

0.1.0 (2014-12-08)
------------------
* moved cmake_modules dependency lines to catkin components list
* attempting indigo build.  fixed Eigen dependency by adding cmake_modules package.
* Updated first instruction of example readme
  test.launch is in rtt_gazebo_examples
* rtt_gazebo_deployer: disabling gravity on model while loading scripts, then restoring after scripts are done loading
* deployer plugin: updates and bugfixes to scripting
  - fixed "inline" script parsing
  - added support for Lua scripts
  - added example using Lua scripts
* updating doc and example
* updating example
* Cleaning up cmakelists
* Updating documentation
* Updating gazebo deployer behavior and the test
* Updating system plugin name
* Functional RTT time-faking using rtt_rosclock
* wip: splitting up plugin into world and model plugin
* Large-scale refactor to use the new rtt_rosclock, reduce complexity.
* Contributors: John Choi, Jonathan Bohren, svozar
