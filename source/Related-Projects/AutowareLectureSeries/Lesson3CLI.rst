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

====
*Example of --help* 
====

Here is ros2 --help::
  
  kscottz@ade:~$ ros2 --help
  usage: ros2 [-h] Call `ros2 <command> -h` for more detailed usage. ...

  ros2 is an extensible command-line tool for ROS 2.

  optional arguments:
  -h, --help            show this help message and exit

  Commands:
  action     Various action related sub-commands
  bag        Various rosbag related sub-commands
  component  Various component related sub-commands
  daemon     Various daemon related sub-commands
  launch     Run a launch file
  lifecycle  Various lifecycle related sub-commands
  msg        Various msg related sub-commands
  multicast  Various multicast related sub-commands
  node       Various node related sub-commands
  param      Various param related sub-commands
  pkg        Various package related sub-commands
  run        Run a package specific executable
  security   Various security related sub-commands
  service    Various service related sub-commands
  srv        Various srv related sub-commands
  test       Run a ROS2 launch test
  topic      Various topic related sub-commands

  Call `ros2 <command> -h` for more detailed usage.

That's all your commands!  

----

=========
ROS 2 Run
=========

`ros2 run` is used to execute ROS nodes.

General format is `ros2 run <package_name> <executable_name> <flags>`

* *TAB COMPLETIONS!* are baked in.
* Generally if tabbing works, you are good to go.
* Don't know a full package name? Try tabbing.
* Don't know the executables in a package?
  * **TRY TABBING!!!**
* Why don't we try starting this `turtlesim node`.
* In your terminal type `ros2 run turtlesim turtlesim_node`

----

=====
TA-DA! A wild ROS turtle appears
=====

* When you run this `ros2 run turtlesim turtlesim_node`, this should happen:

.. image:: ./images/turtlesim_start.png
	   :width: 200
	   
* This is cool, this is our simple virtual turtle. Don't worry if the turtle looks different.  Let's make the turtle move. 

  * In byobu press `F2` to create a new terminal.
  * source `source /opt/ros/dashing/setup.bash`
  * We're going to run another node, let's check out this `draw_square`.
  * `ros2 run turtlesim draw_square`

----

====
Moving your Turtle
====

If everything is setup correctly your turtle should move. 

.. image:: ./images/turtlesim_square.png
	   :width: 800

You can stop the simulation using *`CTRL-C`*
		   

----

====
Let's Explore What is Happening 
====

* We have two terminals open running two "programs" running.

  * We have the `turtlesim` "program" running in the first terminal.
  * The `draw_square` "program" is running in a second terminal.
  * The two are communicating over ros topics.
  
* _What if we didn't know what was going on?_
* What if we worked with a large team and a lot of programs, or nodes, were created by our team mates.

**How can we figure out what _nodes_ are running on our simulated robot?**

----

====
Inspecting Nodes 
====

* Open a new terminal by press `F2`
* Source your bash file `source /opt/ros/dashing/setup.bash`

Let's try inspecting our running nodes::

  kscottz@ade:~$ source /opt/ros/dashing/setup.bash
  
  kscottz@ade:~$ ros2 node --help
    Commands:
      info  Output information about a node
      list  Output a list of available nodes

      Call `ros2 node <command> -h` for more detailed usage.
      
  kscottz@ade:~$ ros2 node list --help
    usage: ros2 node list [-h] [--spin-time SPIN_TIME] [-a] [-c]
    Output a list of available nodes
    optional arguments:
    -h, --help            show this help message and exit
    -a, --all             Display all nodes even hidden ones
    -c, --count-nodes     Only display the number of nodes discovered

----

====
Let's Try Node List
====

Let's Try ros2 node list.::

  kscottz@ade:~$ ros2 node list
  /draw_square  <== This is the node moving the turtle.
  /turtlesim    <== This is the node rendering the turtle. 

We can see the two nodes we started.

Can we dig down deeper into each of these nodes?

----

====
Let's try Node Info
====

Let's try this ros2 node info command!


.. image:: ./images/node_info.png
	   :width: 400

*WOW, THAT'S A LOT OF INFO!!!*

* What's there?
  * Subscribers and message types. 
  * Publishers and message types.
  * Services
  * Actions 

----

====
What about non CLI Options?
====

* Understanding complex graphs as a list of node and topic names in our shell is really hard.
* Good news: we have GUI tool!
* Type `rqt_graph` in the terminal.
* The little double arrow in the top left will load nodes. 

.. image:: ./images/rqt_graph.png
	   :width: 400

----
