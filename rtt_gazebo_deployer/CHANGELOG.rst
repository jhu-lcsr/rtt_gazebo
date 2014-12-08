^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rtt_gazebo_deployer
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* moved cmake_modules dependency lines to catkin components list
* attempting indigo build.  fixed Eigen dependency by adding cmake_modules package.
* rtt_gazebo_deployer: disabling gravity on model while loading scripts, then restoring after scripts are done loading
* Update README.md
* Update README.md
* Update README.md
* deployer plugin: updates and bugfixes to scripting
  - fixed "inline" script parsing
  - added support for Lua scripts
  - added example using Lua scripts
* deployer plugin: minor reorg
* Merge branch 'master' of github.com:jhu-lcsr/rtt_gazebo
* rtt_rosclock api changes, adding rosdeployment
* deployer plugin: Switching to only use one mutex
* STATIC SHIZNAT YO
* Fixing stability issues when loading multiple models with rtt_gazebo plugins
* Moving potentially conflicting things into deferred load thread and adding mutex to prevent race conditions creating rtt structures
* updating doc and example
* deployer: Changing xml schema to support multiple ops scripts, fixing inline script parsing
* rtt_gazebo_deoployer: Adding support for multiple RTT components with gazebo hooks in a single model
* rtt_gazebo_deployer: Fixing issue when building in isolation
* Set up orocos logging to go out through gzlog and use runscript instead of eval
* Doc updates
* updatng doc
* updatng doc
* updating example
* Cleaning up cmakelists
* Updating documentation
* Updating gazebo deployer behavior and the test
* Functional RTT time-faking using rtt_rosclock
* wip: splitting up plugin into world and model plugin
* Update README.md
* updates to readme
* updates to readme
* Large-scale refactor to use the new rtt_rosclock, reduce complexity.
* Renaming rtt_gazebo_plugin to rtt_gazebo_deployer (fixes `#11 <https://github.com/jhu-lcsr/rtt_gazebo/issues/11>`_)
* Contributors: John Choi, Jonathan Bohren
