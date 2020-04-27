=====
Autoware Course
=====

Lesson 1: Building ROS workspace
-----



Lesson 2: ROS 2 101 - Just the basics
-----

1. What is a robot?
2. History of ROS
   3. ROS 2: Production Ready Boogaloo
   1. Maturity
   2. Security
   3. RTOS Support
   4. Portability
4. 30 Minutes of Robotics Concepts
   1. Concurrency (not your problem)
   2. Message Passing / Buses
   3. Black Boards
   4. State Machines
   5. Real Time (not really)
5. Pillars of ROS
   1. Nodes
   2. Messages and Topics
   3. Services
   4. Actions
   5. Parameters
6. Playing Well With Others
   1. Packages
   2. Workspaces
   3. Run-Time Tools


Lesson 3: ROS 2 Tooling - Develop Like a Pro
----
1. Overview and Motivating Concepts
   1. The Command Line
   2. Environment Variables
2. Setuping up a Workspace.
   1. Say it 10 times: source setup.bash
   2. Checking your environment.
3. Setting up our Toy Environment:
   1. Review Lesson 1?
   2. Workspaces again?
   3. Building again?
   4. Launching again?
3. Command Line Tooling in ROS 2:
   1. ros2 run: execute a program
   2. rqt_graph - inspecting nodes
   3. ros2 node: inspect a node
      1. ros2 node list
      2. ros2 node info
4. "Sniffing the Bus": Examining Topics
   1. ros2 topic list
   2. ros2 topic echo
   3. ros2 topic info
   4. ros2 interface show
   5. ros2 topic pub
   6. ros2 topic hz
   7. GUI Tools: rqt_plot / rqt_graph
5. Services: Making things Happen
   1. ros2 service list
   2. ros2 service type
   3. ros2 service find
   4. ros2 interface show
   5. ros2 service call
   6. Seeing your results rviz2
6. Parameters
   1. ros2 param list
   2. ros2 param get
   3. ros2 param set
   4. ros2 param dump
   5. Parameter Serialization
7. Logging Data: Secure the Bag
   1. What's a bag?
   2. ros2 bag record
   3. ros2 bag record -- selecting topics
   4. ros2 bag info
   5. ros2 bag play
   6. Python tools for bag introspection
   7. Bags as a collaborative tool
