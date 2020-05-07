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

* Fire up a terminal manager inside the container. I use byobu. You can use whatever you want. You can also fire up 3 real terminals and call `ade enter` on them. 
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

====
Let's Learn about Nodes and Publishers together!
====

* The core of ROS is the ROS pub/sub bus. In ROS parlance this is called `topic`.

  * A topic has a `message type` that is published on the bus. These messages are defined in a yaml file and define the serialization/deserialization format for ROS messages.
  * ROS has a lot of built in message types. There are lots of pre-defined messages for controlling a robot, distributing sensor data, and understanding the geometry of your robot.
  * ROS publishers produce messages and slowly or as quickly as they need to.
  * A ROS subscriber, `subscribes` to a `topic` and then does things with the information.

* ROS has lots of built-in tools for managing topics. You can list them, echo (watch) them, rename them (called remap), and store them to file (called bagging). 

* ROS `Nodes` are basically programs, or processes that run concurrently on ROS.

  * A ROS node can publish to one or more topics.
  * That same node can subscribe to other topics.
  * Many nodes subscribe to topics, process the data, and publish the results.
  * ROS has tooling to start and stop multiple nodes at the same time. 

----


====
Preparing to Run a ROS Node: setup.bash files 
====

* Open a new terminal, in Byobu you can do this by pressing `F2`.
* First we need to source the `setup.bash` file for our `workspace.` This will help ROS find the programs we built.

  * `source ./ros2_example_ws/install/setup.bash`
  * Protip: you can find any file using `find ./ -name <file name>`  
  
* **ROS Best Practice** _ALWAYS_ build and execute in different terminals.
  
  * The build terminal should source the global ROS setup.bash file (i.e. /opt/ros/dashing/setup.bash).
  * The execution terminal should source the `setup.bash` of your workspace  
  * This is a common failure mode for new users. If something seems weird or funky. Create a new terminal and source the correct bash file.

----

====
Let's Run a Simple C++ Publisher Node. 
====

* ROS has an advanced, and fairly complex CLI interface. We'll cover it in depth in our next lesson.
* We are going to ask ros to run the EXECUTABLE `publisher_lambda` in our WORKSPACE named `examples_rclcpp_minimal_publisher`.
* The syntax for doing this is `ros2 run <WORKSPACE> <EXECUTABLE>`
* To run our publishing node, let's run the following command in our execution terminal: `ros2 run examples_rclcpp_minimal_publisher publisher_lambda`
* If everything works you should see something like this:

::
   
   kscottz@ade:~$ ros2 run examples_rclcpp_minimal_publisher publisher_lambda 
   [INFO] [minimal_publisher]: Publishing: 'Hello, world! 0'
   [INFO] [minimal_publisher]: Publishing: 'Hello, world! 1'
   [INFO] [minimal_publisher]: Publishing: 'Hello, world! 2'
   [INFO] [minimal_publisher]: Publishing: 'Hello, world! 3'
   ...

* To exit the program press `CTRL-C`

----

====
What just happened?
====

* We just executed a ROS node that publishes a simple string message to a topic called `/topic` twice a second.
* I'll show you how I know this with some tools. We'll cover these tools in detail next time.

::

   kscottz@ade:~$ ros2 topic list
   /parameter_events
   /rosout
   /topic
   kscottz@ade:~$ ros2 topic echo /topic
   data: Hello, lambda world! 63
   ---
   data: Hello, lambda world! 64
   ---
   data: Hello, lambda world! 65
   ---
   kscottz@ade:~$ ros2 topic hz /topic 
   average rate: 2.000
   min: 0.500s max: 0.500s std dev: 0.00011s window: 4
   kscottz@ade:~$ 
   
----

====
Digging into the Code
====

* Let's take a look at the code. Like a lot of software there is more than one way to skin a cat. Let's look at the member function approach.
* Using your favorite editor open the folloing source file, `./ros2_example_ws/src/examples/rclcpp/minimal_publisher/member_function.cpp`
* **rclcpp** is an abbreviation of "ROS Client Library C++", its the ROS C++ API
  
.. code-block:: c++
   :linenos:
      
   #include <chrono>
   #include <memory>

   #include "rclcpp/rclcpp.hpp" // THIS the header file for the ROS 2 C++ API
   #include "std_msgs/msg/string.hpp" // This is header for the messages we want to user
                                      // These are usually auto generated. 

   using namespace std::chrono_literals;

   /* This example creates a subclass of Node and uses std::bind() to register a
   * member function as a callback from the timer. */
                                                   // Make a class called Minimal Publisher
     class MinimalPublisher : public rclcpp::Node  // Have it inherit from the ROS Node Class
     
----

====
Let's Build our Node's Constructor
====

* The `MinimalPublisher` constructor inherits fomr the RCLCPP Base Class, gives the name a node, and sets our counter.
* The next line creates a publisher object that publishes `std_msgs::msg`.
* The constructor then creates a callback to the function `timer_callback` that gets called every 500ms. 


.. code-block:: c++
   :linenos:

      class MinimalPublisher : public rclcpp::Node // Inherit from ROS Node
      {
      public:
        MinimalPublisher()
        : Node("minimal_publisher"), count_(0) // Set the node name
        {   //Create a publisher that pushes std_msgs::msg to the topic "topic" 
	    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10); 
            timer_ = this->create_wall_timer( // Call timer_callback every 500ms
              500ms, std::bind(&MinimalPublisher::timer_callback, this));
        }

----
	
====
Now to Handle the Callback
====

* In the callback function we do the following:

  * Create the ROS `std_msgs::msg::String()` to send to our topic.  
  * Construct the message that will be pushed to the ROS Topic
  * Log the results.
  * Actually publish the newly constructed message.  

.. code-block:: c++
   :linenos:

      private:
       void timer_callback()
       {
         auto message = std_msgs::msg::String(); // create message
         message.data = "Hello, world! " + std::to_string(count_++); // Fill it up
         RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str()); // Log it
         publisher_->publish(message); // Publish
       }
       // Create our private member variables. 
       rclcpp::TimerBase::SharedPtr timer_;
       rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
       size_t count_;

----

====
Finally, Let's Create the Main for our Node
====

* This last little bit creates the main node entry point.
* Initializes `rcpcpp` with the values from the command line.
* Run's the MinimalPublisher, until a terminate is given
* Finally the node cleans up everything and exits. 

.. code-block:: c++
   :linenos:
      
     int main (int argc, char * argv[])
     {
       rclcpp::init(argc, argv); # Init RCL
       rclcpp::spin(std::make_shared<MinimalPublisher>()); # Run the minimal publish
       rclcpp::shutdown(); # Cleanup on shut down.
       return 0;
     }

----

====
Exercise:  Modify and Build this Node
====
* Let's try to make a few modification to our node for practice.

  * Make it run at 10Hz (100ms) insteand of 500.
  * Change the topic name from "topic" to "greetings."
  * Change the message "Hello Open Road."
  * Change the node name from `minimal_publisher`, `revenge_of_minimal_publisher`

* Once you make these changes
  
  * Save the file.
  * Toggle over to your execution window run
  * Run `colcon build`
  * In your execution window run `ros2 run examples_rclcpp_minimal_publisher publisher_member_function`
  

----

====
Let's Try Subscribing. 
====

* The pattern here is similar to publishing.
* We basically inherit from the Node class, and define the topic and message we want.
* Whenever that topic is published we hit a callback.
* If everything is correctly configured the file is at

  * /ros2_example_ws/src/examples/rclcpp/minimal_subscriber/member_function.cpp


.. code-block:: c++
   :linenos:
      
      #include <memory>

      #include "rclcpp/rclcpp.hpp"
      #include "std_msgs/msg/string.hpp"
      using std::placeholders::_1;

      // Again we inherit the public interface of a ROS node. 
      class MinimalSubscriber : public rclcpp::Node
      {
        public:
        MinimalSubscriber() // Construct our node, calling it minimal_subscriber
        : Node("minimal_subscriber")
        { // Create a subscription, to messages of the format stdmsg:msg:String
          subscription_ = this->create_subscription<std_msgs::msg::String>(
	  // Subscribe to the topic, "topic" and set a callback for when things are pub'd 
	  "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
	}
      ...

----

====
More Subscriber
====

* The subscriber node looks fairly similar to our publisher but instead of publishing on a regular callback, we get a callback when a new messae hits our topic. 

.. code-block:: c++
   :linenos:
      
   private:
     // Whenever we get a new messaged published on our topic
     // this callback will be executed.    
     void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
     {
       // Log the message that we are subscribed to 
       RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
     }
     rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
   };

   // This is effectively the same boiler plate from last time. 
   int main(int argc, char * argv[])
   {
     rclcpp::init(argc, argv);
     rclcpp::spin(std::make_shared<MinimalSubscriber>());
     rclcpp::shutdown();
     return 0;
   }

----

====
Let's Modify the Subscriber
====

* In the publisher we changed the name of our publisher topic to `greetings.`
* Let's change the subscribed topic to `greetings`.
* Note that there are a lot of ways to change topic names, modifying source is just one approach. Offten we just `remap` topics instead of changing source.


* Once you have modified the subscriber run `colocon build ` (it will build everything)
* Open another terminal, source the bash file, and start the publisher.

  * `ros2 run examples_rclcpp_minimal_publisher publisher_member_function`
  
* Now run our subscriber.
  
  * `ros2 run examples_rclcpp_minimal_subscriber subscriber_member_function`


----

====
The Result
====

If everything went well you should have two screens. The first screen with the publisher should be spitting out the following

.. code-block:: bash

   [INFO] [revenge_of_minimal_publisher]: Publishing: 'Hello, Open Road! 1000'
   [INFO] [revenge_of_minimal_publisher]: Publishing: 'Hello, Open Road! 1001'
   [INFO] [revenge_of_minimal_publisher]: Publishing: 'Hello, Open Road! 1002'
   [INFO] [revenge_of_minimal_publisher]: Publishing: 'Hello, Open Road! 1003'
   [INFO] [revenge_of_minimal_publisher]: Publishing: 'Hello, Open Road! 1004'

The subscriber screen should be pushing out:

.. code-block:: bash

   [INFO] [minimal_subscriber]: I heard: 'Hello, Open Road! 1000'
   [INFO] [minimal_subscriber]: I heard: 'Hello, Open Road! 1001'
   [INFO] [minimal_subscriber]: I heard: 'Hello, Open Road! 1002'
   [INFO] [minimal_subscriber]: I heard: 'Hello, Open Road! 1003'
   [INFO] [minimal_subscriber]: I heard: 'Hello, Open Road! 1004'

**You can terminate both of these programs with `CTRL-C`**

*Congratulations, you now know the three most important ROS components, nodes, publishers, and subscribers. 
   
----

====
Making Things Happen with Services
====

* Publishing and subscribing nodes are the bread and butter of ROS. This pattern is great for moving around a lot of data, and processing it quickly.
* However, we often want our robots to respond to data. To construct simple behaviors in ROS we use `services.
* A service is a robotic task that can be performed _synchronusly_, which is just afancy word for `while you wait.
* A good analogy for services would be a regular old function call. In most programs when you call a function, the code making the call waits for the function to return before proceeding.
* A few toy examples of services for autonomous driving would be:
  
  * Turning Lights Off/On.
  * Checking a sensor and returning the results.
  * Lock / Unlock a door or window.
  * Beeping a horn.
    
* Services can be called via the command line or through an API call within another node. 
* In ROS services are hosted within a ROS Node, and they can co-exist with other services as well as publishers and subscribers.

----

====
C++ Service Example
====

* As a toy example of a ROS service we are going to make a node that offers an "AddTwoInts" service.
* What will happen is the service has two inputs, and returns a single output.

In your terminal navigate to the following file and open it in your favorite editor or use the `less` command to peek inside.

`ros2_example_ws/src/examples/rclcpp/minimal_service/main.cpp`

The source should look like this:





----

====
====

.. code-block:: c++
   :linenos:
      
      #include <inttypes.h>
      #include <memory>
      #include "example_interfaces/srv/add_two_ints.hpp"
      #include "rclcpp/rclcpp.hpp"

      using AddTwoInts = example_interfaces::srv::AddTwoInts;
      rclcpp::Node::SharedPtr g_node = nullptr;

      void handle_service(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<AddTwoInts::Request> request,
        const std::shared_ptr<AddTwoInts::Response> response)
      {
        (void)request_header;
        RCLCPP_INFO( g_node->get_logger(),
	"request: %" PRId64 " + %" PRId64, request->a, request->b);
	response->sum = request->a + request->b;
      }
