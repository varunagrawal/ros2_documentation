====
Lesson 2: Introduction to ROS API and Build Tools
====

* This lesson is intended to be an hour long crash course in the ROS 2 Dashing API and build tools. * After this lesson you should be able to code and build a rudimentary ROS 2 application.
* It should be noted, that since this is a "crash course" we are giving you a pre-configured Docker environment call ADE.
  
  * We will not cover setting up the ROS environment.
  * This will be left as an exercise to the reader.
  * As much as possible we have used the tooling availabe *ROS 2 Dashing desktop full* installation.


* This crash course was written for ROS 2 Dashing.
  
  * ROS 2 Foxy is about to be released.
  * This relase is the first major, stable, release of ROS 2. 

* The next lesson, lesson 3, will show you how to use the ROS 2 command line interface. 

----

====
Before We Begin...
====

.. image:: ./images/jfk.jpg
	   :align: center
	   :width: 500


		   
*"We choose to go to the moon, not because it is easy, but because it is hard."* 		   


**ROS is hard. There are many topics you must learn to become a skilled robotocist. We try to make it easy, but be aware the path ahead is difficult. You will get stuck. You will get frustrated. You will need help**


*The good news is, all of this is ok and completely normal; there are resources out there to help!* 

----

====
Getting Help!
====

**ROS has a ton of resources to help you be succeed. You should be aware of these before you begin.**


* We have our own QA website like Stack Overflow called ROS Answers.

  * `http://answers.ros.org <https://answers.ros.org/questions/>`_
  * ROS answers actually predates the rise of Stack Overflow!

* Check out ROS Discourse. It is the community hub for news and discussion.

  * `https://discourse.ros.org/ <https://discourse.ros.org/>`_
  * *DO NOT* ask questions on discourse.

* We have a very large ROS wiki. It is mostly ROS 1.

  * `http://wiki.ros.org/ <http://wiki.ros.org/>`_
  * Most of the content is still highly useful.

* Most of this talk comes from the ROS 2 documentation.
  
  * `https://index.ros.org/doc/ros2/ <https://index.ros.org/doc/ros2/>`_
  * This is probably where you should look. 

----

====
Other and/or Un-Official Resources
====

* The ROS / Robotics Sub Reddits are Great!
* There is an "un-official" `ROS Discord <https://discord.com/invite/HnVcz5a`>_.

  * Please try using ROS Answers first.
  
* We have a yearly ROS developers conference `ROSCon. <https://roscon.ros.org/2020/>`_

  * Most of old talks are free on the web.

* We're not big on social media but we're busy on the twitter.

  * `@OpenRoboticsOrg <https://twitter.com/openroboticsorg>`_ is a bit more active.
  * `@ROSOrg <https://twitter.com/rosorg>`_ "Offical" ROS announcements.

* `Open Robotics <https://www.openrobotics.org/>`_ is the non-profit that administers ROS and Ignition Gazebo.

  * We take donations and take contract work from time to time. 

----

====
To Understand ROS let's talk about its History
====

*PR2 Image*

* Let's go back to the early 2000's.
  
* Open Source is growing, but Windows dominates. 

* What about Robots?:

  * Robots are expensive and mainly for mass manufacturing and R&D.
  * Mostly is "real-time" control systems. Just make arms move the same way over and over.
  * Not a lot of Open Source.

* ~2006, Former Google VPs decide to work on Robots.

  * Create a company called `Willow Garage. <https://en.wikipedia.org/wiki/Willow_Garage>`_
  * From this org we get OpenCV, PCL, ROS, PR2 Robot, and many spin outs. 

* ~2012 Willow Garage Folds, Open Robotics emerges.

  * 2017 ROS 2 Begins to move ROS out of the lab (it was already out of the lab).
  * Address security and robustness concerns.
  * Add RTOS support and support other OS's. 
----

====
Let's Talk about Some Concepts that Motivate the Design of ROS
====

ROS's design was informed by *design patterns* that were successfully used in prior robotic systems. We can't cover each of these in detail, but reading about them will help you better understand ROS.

* **Processes / Threads ==> ROS Nodes** -- A ROS Node is a self contained execution process, like a program. ROS is really a lot of tooling for running a bunch of programs in parallel. 
* **Buses / PubSub ==> ROS Topics** -- The backbone of ROS is a `publish/subscribe bus <https://en.wikipedia.org/wiki/Publish%E2%80%93subscribe_pattern>`_. If you have ever used ZeroMQ, RabbitMQ, or ModBus, ROS topics are very similar. 
* **Serialization ==> ROS Messages / ROS Bags** -- ROS uses a pre-defined messages to move data over topics. This allows data to be `serialized <https://en.wikipedia.org/wiki/Serialization>`_ between nodes in different programming languages. An analog would be Google Protocol Buffers. ROS can be written to file, called a bag. A good analogy is a python pickle file.   
* **Black Board Pattern  ==> ROS Params** -- A `blackboard <https://en.wikipedia.org/wiki/Blackboard_(design_pattern)>`_ is a way to create global variables between nodes/programs. A good analogy would be Redis. 
* **Synchronus Remote Procedure Call (RPC)  ==> ROS Services** -- A ROS service is a program that can be called by another program. The caller is blocked until the callee returns.  This is formerly called a `remote procedure call <https://en.wikipedia.org/wiki/Remote_procedure_call>`_. 
* **Asynchronus Remote Procedure Call (RPC)  ==> ROS Actions** -- A ROS action is a program that can be called by another program. The caller is **not** blocked until the callee returns.  
* **State Machines ==> ROS Lifecycles** -- `State machines <https://en.wikipedia.org/wiki/Finite-state_machine>`_ are a tool to move between states, or modes. State machines are a useful way to model machine behavior. 
* **Matrix Math for 3D Operations ==> URDF and TF** -- TF, short for transform, and URDF (universal robot description format) are tools for automatically `calculating robot geometry using matrix math <https://en.wikipedia.org/wiki/Matrix_(mathematics)>`_ .
  
----

====
Jumping in the Deep End: Start ADE and Install/Update Deps
====

* First things first, let's make sure everything is ready to go.
* Now is a good time to hit pause on the video make sure you have intalled the requirements.
* Install ADE as per Autoware Instructions.
* Now were going to update the system, install ROS dashing, and a couple tools.

::

   ade start
   ade enter 
   source /opt/ros/dashing/setup.bash 
   sudo apt update
   sudo apt install ros-dashing-turtlesim
   sudo apt install ros-dashing-rqt-*
   sudo apt-install byobu

You should now be ready for the class!

----

====
Some Nomenclature as we Begin
====

* **Package** -- A collection of code. 
* **Workspace** -- A workspace is a collection of source code / ROS packages that will run on a robot. It has a uniform directory structure.  A good analogy is python virtual env, or "project" in most IDEs. 
* **Overlay** -- A second workspace with more/different/new packages. If there are multiple versions of a package/code then the one at the bottom is used.  
* **Underlay** -- The workspace, underneath an overlay, we're aware this is confusing. 
* **Colcon** -- The ROS 2 build tool. Think of it as a layer above CMake/Make/SetupTools that helps these tools work together smoothly.  

This is a bit confusing. You may ask yourself why we have our own build tool.  The short of it is that the ROS ecosystem consists of tens of thousands of developers, working on thousands of packages, across a handful of platforms, using multiple languages. We needed a flexible system to build code and one didn't exist at the time, and still doesn't exist.

----

====
Let's Get Started 
====

As we're diving headfirst into ROS our first job is to checkout a repository of examples and build it. Roughly the steps to do this are as follows. 

* Source the ROS setup.bash file so we have the right version of ROS in our path.
* Make a workspace called `ros2_example_ws`. We usually use `_ws` to indicate a workspace.
* Clone an example repository and change to the dashing branch.

  * Generally ROS repos have a branch per release. 

* Use Colcon to build the source. 
::
   
   source /opt/ros/dashing/setup.bash
   mkdir -p ~/ros2_example_ws/src
   cd ~/ros2_example_ws
   git clone https://github.com/ros2/examples src/examples
   cd ~/ros2_example_ws/src/examples/
   git checkout dashing 
   cd ~/ros2_example_ws
   colcon build --symlink-install

----

#. What is a robot?
#. History of ROS
#. ROS 2: Production Ready Boogaloo
    #. Maturity
    #. Security
    #. RTOS Support
    #. Portability
#. 30 Minutes of Robotics Concepts
    #. Concurrency (not your problem)
    #. Message Passing / Buses
    #. Black Boards
    #. State Machines
    #. Real Time (not really)
#. Pillars of ROS
    #. Nodes
    #. Messages and Topics
    #. Services
    #. Actions
    #. Parameters
#. Playing Well With Others
    #. Packages
    #. Workspaces
    #. Run-Time Tools

====
ROS 2 Tooling: Introduction to CLI and friends
====

#. Overview and Motivating Concepts
    #. The Command Line
    #. Environment Variables

----

====
The Command Line
====

* ROS is effectively Linux for Robots.
* From Last Time:

  * ROS has your cross platform build tools.
  * ROS has ployglot builds.

* Now lets talk about runtime.

  * ROS presents command line interface (CLI) for robot execution.
  * Most of these commands follow a regular format.
  * These commands have auto-tab complete (yay!)
  * Most blank commands will spit out error or info.
  * Most commands will behave nicely with `--help`
    
----
