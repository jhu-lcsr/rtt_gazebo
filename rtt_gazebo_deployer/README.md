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

### Gazebo RTT Component Interface

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
this->provides("gazebo")->addOperation("configure",&MyComponent::gazeboConfigureHook,this,RTT::ClientThread);
this->provides("gazebo")->addOperation("update",&MyComponent::gazeboUpdateHook,this,RTT::ClientThread);
```

For example implementations of these operations, see the [default gazebo
component](../rtt_gazebo_examples/src/default_gazebo_component.cpp).

### Gazebo Deployer

The desired RTT component is named after the Gazebo model whem it is loaded
into an Orocos OCL DeploymentComponent. By default, all models are loaded into
the same DeploymentComponent, called "gazebo", but they can specify that they
need to be isolated, in which case they are loaded in an alternative
DeploymentComponent named "MODEL\_NAME\_\_deployer\_\_". This allows components
associated with different models to either be isolated or interact with
each-other.

The RTT Gazebo component also support CORBA-based remote task browsing. For more
informationa about connecting a task browser to a set of orocos components
running inside of Gazebo, see [rtt\_gazebo\_console](../rtt_gazebo_console).

## Usage

The RTT Gazebo deployer plugin is a model plugin, so it gets attached to a
given model in SDF or URDF. 

The following tags are required:
* `<component>...</component>` One or more user-defined RTT components (one per
  `<component>` tag) loaded in the plugin as the Gazebo component. This is the
  component which will be connected to the Gazebo update hook.  Underneath,
  this imports the desired package and then loads a component with the given
  type and desired name into this model's deployer.  This component is
  specified by the three following tags:

  * `<package>...</package>` The (ROS) package in which the component is defined.
  * `<type>...</type>` The type of the component.
  * `<name>...</name>` The name of the component.

The following tags are supported:
* `<opsScript>...</opsScript>` In-line Oroocs Ops script to be executed after the
  plugin is loaded and the RTT component has been loaded. ***NOTE: only `/* ...
  */` comments can be used because Gazebo removes all line breaks from parsed
  XML!*** 
* `<opsScriptFile>/path/to/file.ops</opsScriptFile>` The path to an Orocos Ops
  script file to be executed when the plugin is loaded.
* `<isolated/>` Load the RTT components in a DeploymentComponent specific to
  this model named "MODEL\_NAME\_\_deployer\_\_". Without this tag, all
  components are created in the "gazebo" DeploymentComponent.

For example, the following XML would load the RTT Gazebo plugin, create a
DefaultGazeboComponent with the same name as the mode in an isolated
DeploymentComponent, and then execute some Orocos Ops script, which in this case
is given in-line in the XML file:

**NOTE:** only `/* ... */` comments can be used because Gazebo removes all line
breaks from parsed XML! Single-line comments starting with `//` or `#` will
comment out the rest of the script in the `<opsScript>` block, and it will not
run!

```xml
<gazebo>
  <plugin name="rtt_gazebo" filename="librtt_gazebo_deployer.so">
    <isolated/>
    <component>
      <package>rtt_gazebo_examples</package>
      <type>DefaultGazeboComponent</type>
      <name>sevenbot</name>
    </component>
    <orocosScript>
      <filename>/path/to/script.ops</filename>
    </orocosScript>
    <orocosScript>
      <inline>
        /* Run arbitrary ops script here */
      </inline>
    </orocosScript>
  </plugin>
</gazebo>
```
