# ROS2 Cheatsheet

Cheatsheet for all basic ROS2 Humble commands.

[ROS2 Humble Documentation](https://docs.ros.org/en/humble/index.html)

<details>
<summary>Table of contents</summary>

-   [Creating a workspace](#creating-a-workspace)
-   [Creating a package](#packages)
-   [Adding new nodes](#new-nodes)
-   [Topics](#topics)
-   [Publishers and Subscribers](#pubsub)
-   [Services](#srvcli)
-   [Actions](#actions)
</details>

<br>

<div id='creating-a-workspace'>

# 1. Creating a workspace

[Docs](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html)

</div>

To work with ros2, you first needed to create a "workspace" which is just a directory suffixed by "\_ws" for good practice (such as "ros2_ws", "turtlesim_ws"). To begin, navigate to the location wherein you want your ros2 workspace to reside and run the following commands.

```bash
mkdir ros2_ws
cd ros2_ws
mkdir src
cd ..
```

Now we created a workspace directory with an src directory where all of our packages will be. It is time to build it. Building your workspace should always be done after creating packages, adding new files or editing files in existing packages, etc.

```bash
colcon build
```

Every time we build our workspace, we need to source the `setup.bash` file found in `ros2_ws/install`. So instead of running this command every time, we can add it to our `.bashrc` file.

```bash
gedit ~/.bashrc
```

Add this command at the end of the file

```bash
source ~/ros2_ws/src/install/local_setup.bash
```

Now you have a workspace and it'll be sourced every time you open a terminal!

<br>
<br>

<div id='packages'>

# 2. Creating a package

[Docs](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)

</div>
In order to start working in your workspace, you need to create a package which will hold all of your code and files relating to a certain project(s).

All packages should be created in the `src` directory in your workspace so first change directory to it then we'll create the package

```bash
cd ~/ros2_ws/src
```

```bash
ros2 pkg create --build-type ament_python <package_name>
```

If you know some dependencies you'll be needing beforehand, you can use the `--dependencies` flag and list your dependencies.

```bash
ros2 pkg create --build-type ament_python <package_name> --dependencies rclpy geometry_msgs
```

**Build the package** - Be sure to go back to the workspace directory building running the build command.

```bash
cd ~/ros2_ws
colcon build
```

Source the setup file by sourcing `.bashrc` edited before

```bash
source ~/.bashrc
```

This process of building and sourcing will be done everytime you add new files, nodes, code, etc to your package(s) so don't forget to do it.

**Running your package** - if you have a node inside your package you can now run it using.

```bash
ros2 run <package_name> <node_name>
```

<br>
<br>

<div id='new-nodes'>

# 3. Adding new nodes

[Docs](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html#id2)

After creating a packages, we want to write code for nodes and execute them. First, create your python file `example_node.py` inside the `/package_name/package_name` directory and start writing code.

```python
from rclpy.node import Node

class MyNode(Node):
  def __init__(self):
    super().__init__('my_node')
  # .....

def main(args=None):
    rclpy.init(args=args)

    # ....
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Now, to make `my_node` an executable, open the `/package_name/setup.py` and add this line to the `console_scripts` array.

```python
  'my_new_node = package_name.file_name:main',
```

Now build and source your packages then you can run this node using

```bash
ros2 run <package_name> my_new_node
```

`my_new_node` will be the executable name used in the command line however `my_node` will be the name of your node

<br>
<br>

<div id='topics'>

# 4. Topics

When running ROS2 nodes, you might want to continuously receive (subscribe) or transmit (publish) information between nodes. This is done through topics. They store information of a certain type. They act as a bus between nodes.

To see all topics

```bash
ros2 topic list
```

```bash
ros2 topics list -t  // flag for getting the data types of each topic
```

To print out the messages (if any) being published on a certain topic

```bash
ros2 topic echo <topic_name>
```

To know how many Subscribers and Publishers subscribe and publish to a certain topic

```bash
ros2 topic info <topic_name>
```

### Manually publishing to a topic

We will take the `/turtle1/cmd_vel` topic in the turtlesim package as an example and publish a message to it directly from the command line

```bash
ros2 topic pub <topic_name> <msg_type> '<args>'
```

```bash
ros2 topic pub --rate 1 /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
```

`--rate` specifies a steady publish stream of 1 Hz. You can use `--once` for publishing once.

<br>
<br>

<div id='pubsub'>

# 5. Publishers and Subscribers

[Docs](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html#)

Publishers publish messages onto topics while Subcribers subscribe to the topic to receive these messages.

I won't cover all the code required to create publisher and subscribers, just the key methods to be used when dealing with publishers and subscribers.

### Publishers

**Creating a publisher** - inside your node's constructor

```python
self.msgPublisher = self.create_publisher(<msg_type>, 'topic_name', 10)
```

Usually, you'd want your publisher to constantly publish messages at a certain interval. So we'll use the `create_timer` method which runs the `publisher_callback` defined by us after every time interval (0.5 seconds here)

```python
self.timer = self.create_timer(0.5, self.publisher_callback)
```

To publish messages, use the `publish` method of your publisher.

```python
self.msgPublisher.publish(msg)
```

### Subscribers

**Creating a subscriber** - inside your node's constructor

```python
self.subscription = self.create_subscription(<msg_type>,'topic_name',self.listener_callback,10)
```

The `listener_callback` is the method that runs whenever our subscriber detect data present in the topic

```python
def listener_callback(self,data):
  # data argument is the message incoming from the topic
```

<br>
<br>

<div id='srvcli'>

# 6. Service and Client

[Docs](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html)

A client is a node that sends a request for data. A service is the node that responds to these requests. Firstly, the structure of a Request and Response is defined in a `.srv` file.

```
int64 a
int64 b
---
int64 sum
```

Request will contain 2 integers `a`, `b` while the Response will contain 1 integer `sum`

In order to write code for services and client, we need to first import that structure. Read more about [creating custom .srv files](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html)

```python
from example_interfaces.srv import AddTwoInts
```

### Services

**Creating a service** - in your node's constructor

```python
self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
```

Inorder to deal with the request and return a response, we have to create a callback function which runs whenever a request is received.

```python
def add_two_ints_callback(self, request, response):
  # you now have access to request.a, request.b, and response.sum as specified in the .srv file
  return response
```

### Clients

**Creating a client** - in your node's constructor

```python
self.client = self.create_client(AddTwoInts, 'add_two_ints')
```

Some important client methods

```python
self.client.wait_for_service(timeout_sec = 1)   # returns boolean on whether it detects the service or not
```

```python
self.future = self.client.call_async(self.req) # makes an async request and returns a future that completes when the request is done
```

You can then use `rclpy.spin_until_future_complete(self, self.future)` so that the node keeps spinning as long as the request didn't return a response yet.

<br>
<br>

<div id="actions">

# Actions

[Docs](https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html)

Actions are similar to a service and client wherein there's a request and a response however actions involve a little more. Actions can have feedbacks and goals.

Actions deal with `.action` files that look like this

```python
# Request
---
# Result
---
# Feedback
```

And so, to begin, we need to import our specific action files.

```python
from action_tutorials_interfaces.action import Fibonacci
```

### Action Servers

**Creating a client** - in your node's constructor

```python
from rclpy.action import ActionServer

self._action_server = ActionServer(self, Fibonacci, 'action_name', self.execute_callback)
```

Similar to before, the action server will take an `execute_callback` which is called to process accepted goals. This callback function can look like this

```python
def execute_callback(self, goal_handle):
    self.get_logger().info('Executing goal...')

    feedback_msg = Fibonacci.Feedback()
    feedback_msg.partial_sequence = [0, 1]

    for i in range(1, goal_handle.request.order):
        feedback_msg.partial_sequence.append(....)
        goal_handle.publish_feedback(feedback_msg)
        time.sleep(1)

    goal_handle.succeed()

    result = Fibonacci.Result()
    result.sequence = feedback_msg.partial_sequence
    return result
```

As you can see, we can get form our feedback and publish it throughout the running of our request execution.

### Action Clients

**Creating a client** - in your node's constructor

```python
from rclpy.action import ActionClient

self._action_client = ActionClient(self, Fibonacci, 'action_name')
```

How an action client works is a little complicated so I'll try to simplify it.

-   In order to send our goal, first, we'll make sure there's a server by using `self._action_client.wait_for_server()` which runs until it return true
-   Send the goal using `self._action_client.send_goal_async(goal_msg)` which returns a future object
-   Add a callback to this future object using `future.add_done_callback()`
-   In this callback we will check if the goal was accepted or rejected, when accepted, we will get a future object using `self._get_result_future = goal_handle.get_result_async()` which when done will return our result message.
-   We can add a callback to run when this future is done `self._get_result_future.add_done_callback(self.get_result_callback)` which just prints out our `future.result().result` information

I didn't want to fit all of the code here so [read the docs](https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html#getting-a-result), it will make more sense.

One more thing, we still didn't get our feedback. To do this, we will simply add a feedback callback function to our `send_goal_async()`

```python
self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
```

```python
def feedback_callback(self,feedback_msg):
  # feedback = feedback_msg.feedback
```
