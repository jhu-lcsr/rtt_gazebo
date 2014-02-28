RTT Gazebo Deployer
===================

This package provides a model plugin for the Gazebo simulator which enables the
construction and execution of Orocos RTT components inside the Gazebo process so
that they can have synchronous API-level access to Gazebo. 

## Design

This plugin aims to provide an easy and efficient way to simulate RTT-based
state estimation and control systems for use on real hardware. This means that
the RTT components used with this plugin should be identical to the ones used
on the real system. 

### Gazebo RTT Component

The standard procedure for simulation in component-based frameworks is for
everything including and above the hardware abstraction layer to be the same,
but with a different underlying implementation of the hardware response. In a
real system, this would be some lower-level API to talk to real hardware, and in
a simulated system, this is the Gazebo API to the virtual model and world. 

In the simplest cases, Gazebo's effort-controlled mechanism model is sufficient,
and for this case, we provide a default RTT component which has input ports for
effort commands and output ports for joint-level state. Sometimes, simulating
more complex physics is required, and in those cases, we expose the Gazebo C++
API to a user-supplied RTT component that provides two special RTT operations
which are called by the Gazebo plugin.

These operations are defined on a "gazebo" sub-service of a bottom-level RTT
component meant to interface with Gazebo and have the following signatures:

```cpp
// Gazebo configure hook, called once when the model is loaded
// my_component.gazebo.configure()
virtual bool gazeboConfigureHook(gazebo::physics::ModelPtr model);

// Gazebo update hook, called once per simulation step
// my_component.gazebo.update()
virtual void gazeboUpdateHook(gazebo::physics::ModelPtr model);
```

In order for these member functions to be called by the RTT Gazebo plugin, they
need to be provided as operations:

```cpp
this->provides("gazebo")->addOperation("configure",&DefaultGazeboComponent::gazeboConfigureHook,this,RTT::ClientThread);
this->provides("gazebo")->addOperation("update",&DefaultGazeboComponent::gazeboUpdateHook,this,RTT::ClientThread);
```

For example implementations of these operations, see the [default gazebo
component](../rtt_gazebo_examples/src/default_gazebo_component.cpp). In the
future, these operations will be automatically created by an RTT Service which
can be loaded on any TaskContext.

### Gazebo Deployer

The desired RTT component is named after the Gazebo model whem it is loaded
into an Orocos OCL DeploymentComponent. By default, all models are loaded into
the same DeploymentComponent, called "gazebo", but they can specify that they
need to be isolated, in which case they are loaded in an alternative
DeploymentComponent named "MODEL\_NAME\_\_deployer\_\_". This allows components
associated with different models to either be isolated or interact with
each-other.
 
### Time

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

### CORBA-Based Console Interation

The RTT Gazebo component also support CORBA-based remote task browsing. For more
informationa about connecting a task browser to a set of orocos components
running inside of Gazebo, see [rtt\_gazebo\_console](../rtt_gazebo_console).

## Usage

The RTT plugin is a model plugin, so it gets attached to a given model in SDF or
URDF. 

The following tags are supported:
* `<opsScript>...</opsScript>` In-line Oroocs Ops script to be executed when the
  plugin is loaded. ***NOTE: only `/* ... */` comments can be used because
  Gazebo removes all line breaks from parsed XML!*** 
* `<opsScriptFile>/path/to/file.ops</opsScriptFile>` The path to an Orocos Ops
  script file to be executed when the plugin is loaded.
* `<isolated/>` Load the RTT components in a DeploymentComponent specific to
  this model named "MODEL\_NAME\_\_deployer\_\_". Without this tag, all
  components are created in the "gazebo" DeploymentComponent.
* `<component>COMPONENT_NAME</component>` Use a user-defined component loaded in
  either the `<opsScript>` or `<opsScriptFile>` tags as the Gazebo component
  instead of the DefaultGazeboComponent.

For example, the following XML would load the RTT Gazebo plugin, create a
DefaultGazeboComponent with the same name as the mode in an isolated
DeploymentComponent, and then execute some Orocos Ops script, which in this case
is given in-line in the XML file:

***NOTE: only "/* ... \*/" comments can be used because Gazebo removes all line
breaks from parsed XML! Single-line comments starting with `//` or `#` will
comment out the rest of the script in the `<opsScript>` block, and it will not
run!***

```xml
<gazebo>
  <plugin name="rtt_gazebo" filename="librtt_gazebo_deployer.so">
    <isolated/>
    <component>sevenbot</component>
    <opsScript>
      /* A comment! */
      import("rtt_ros");
      ros.import("rtt_std_msgs");

      ros.import("rtt_gazebo_examples");

      loadComponent("sevenbot", "DefaultGazeboComponent");
    </opsScript>
  </plugin>
</gazebo>
```

## Demo

This package also includes a simple demonstration of the plugin which also uses
ROS for launching gazebo.

First, launch gazebo with the demo model:
```shell
roslaunch rtt_gazebo_deployer test.launch
```

Then, in another shell, you can launch the rtt\_gazebo\_console:
```shell
rosrun rtt_gazebo_console console-$OROCOS_TARGET
```

This will display something similar to this:
```shell
   Switched to : console_deployer

  This console reader allows you to browse and manipulate TaskContexts.
  You can type in an operation, expression, create or change variables.
  (type 'help' for instructions and 'ls' for context info)

    TAB completion and HISTORY is available ('bash' like)

    Use 'Ctrl-D' or type 'quit' to exit this program.

console_deployer [S]> 
```

Once you've loaded up the console, you can `cd` into the demo component:
```shell
cd gazebo
cd sevenbot
```

Then you can list some of the debug attributes:
```
ls debug
```

This will display something simiar to:
```shell
 Listing Service debug[R] :

 Configuration Properties: (none)

 Provided Interface:
  Attributes   : 
     double time_rtt       = 6.938               
     double time_gz        = 6.938               
        int steps_rtt      = 6891                
        int steps_gz       = 6771                
        int n_joints       = 8                   
      array joint_pos      = { [-2.78448e-06, 7.2723e-06, 6.19356e-06, -5.0639e-06, 3.9167e-06, -2.79475e-06, 1.73491e-06, 8.32204e-07 ], size = 8, capacity = 8 }
      array joint_command  = { [3.06293e-06, 8.01095e-06, -6.80253e-06, 5.57588e-06, -4.30829e-06, 3.07115e-06, -1.90659e-06, -9.20536e-07 ], size = 8, capacity = 8 }

  Operations      : (none)

 Data Flow Ports: (none)

 Services: 
(none)
```

You can also change the gains of the joint-level PD effort control:
```
kp = 0.5
kd = 10
```
