=====
Autoware Course
=====

Lesson 1: Building ROS workspace
-----



Lesson 2: ROS 2 101 - Just the basics
-----

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


Lesson 3: ROS 2 Tooling - Develop Like a Pro
----
#. Overview and Motivating Concepts
    #. The Command Line
    #. Environment Variables
#. Setuping up a Workspace.
    #. Say it 10 times: source setup.bash
    #. Checking your environment.
#. Setting up our Toy Environment:
    #. Review Lesson 1?
    #. Workspaces again?
    #. Building again?
    #. Launching again?
#. Command Line Tooling in ROS 2:
    #. ros2 run: execute a program
    #. rqt_graph - inspecting nodes
    #. ros2 node: inspect a node
       #. ros2 node list
       #. ros2 node info
#. "Sniffing the Bus": Examining Topics
    #. ros2 topic list
    #. ros2 topic echo
    #. ros2 topic info
    #. ros2 interface show
    #. ros2 topic pub
    #. ros2 topic hz
    #. GUI Tools: rqt_plot / rqt_graph
#. Services: Making things Happen
    #. ros2 service list
    #. ros2 service type
    #. ros2 service find
    #. ros2 interface show
    #. ros2 service call
    #. Seeing your results rviz2
#. Parameters
    #. ros2 param list
    #. ros2 param get
    #. ros2 param set
    #. ros2 param dump
    #. Parameter Serialization
#. Logging Data: Secure the Bag
    #. What's a bag?
    #. ros2 bag record
    #. ros2 bag record -- selecting topics
    #. ros2 bag info
    #. ros2 bag play
    #. Python tools for bag introspection
    #. Bags as a collaborative tool
