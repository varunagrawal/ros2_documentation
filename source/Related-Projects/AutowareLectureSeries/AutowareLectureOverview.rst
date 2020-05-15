=====
Autoware Course
=====

Lesson 1: Building ROS workspace
-----



Lesson 2: ROS 2 101 - Just the basics
-----

#. Intro
#. Getting Help
#. Unofficial Resources
#. ROS Intro
   #. Brief Intro to ROS
   #. Core Concepts of ROS
   #. Environment Setup
   #. Colcon Nomenclature
#. Nodes and Publishers
   #. Overview of Topics
   #. Building and Running a Node
   #. Simple Publisher Build and Run
   #. Modify the Publisher
   #. Building a Subscriber
   #. Pub/Sub Working Together
#. Services
   #. Concept Overview
   #. Review Basic Service
   #. Running Basic Services
   #. Calling Services from Command Line
   #. Building a Service Client
   #. Executing Service Server/Client.
#. Actions
   #. Action Overview
   #. Action File Review
   #. Basic Action Review
   #. Running / Calling an Action
   #. Action Client Review
   #. Running Action Server with Client. 


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
#. Logging Data: Secure the Bag
    #. What's a bag?
    #. ros2 bag record
    #. ros2 bag record -- selecting topics
    #. ros2 bag info
    #. ros2 bag play
    #. Python tools for bag introspection
    #. Bags as a collaborative tool
