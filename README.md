# ROS2 Cheatsheet

Cheatsheet for all basic ROS2 Humble commands.

[ROS2 Humble Documentation](https://docs.ros.org/en/humble/index.html)

Table of contents

-   Creating a workspace
-   Creating a package
-   Topics
-   Publishers and Subscribers
-   Services
-   Action Server and Client

# 1. Creating a workspace

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

# 2. Creating a package

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

**Build the package**  - Be sure to go back to the workspace directory building running the build command.
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


# 4. Adding new nodes
After creating a packages, we want to write code for nodes and execute them. First, create your python file  `example_node.py` inside the `/package_name/package_name` directory and start writing code.

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

# 5. Publishers and Subscribers
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




















