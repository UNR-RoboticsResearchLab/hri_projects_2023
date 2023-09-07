# Week1 Tutorial

Major deliverables this week:

- Use a message from a prior package
- Get familiar with the stage simulator
- Issue some robot commands for stage
- Practice a ROS connection from one computer to another
- Obstacle avoidance

## How this package was created

You don't need to run this command, but just so that you know how this package was created, here it is:

```
catkin_create_pkg week2 week1 rospy geometry_msgs std_msgs stage_ros
```

## Get familiar with the stage simulator

First off, make sure that stage is installed:

```
sudo apt install ros-noetic-stage-ros
```

I've added a world file. you can run this using a launch file:

```
roslaunch week1 basic_world.launch
```

If you don't already have a roscore node started, then launching the launch file will start one of those (which will close when you close the launch file), it also sets a parameter that you'll need when using simulators, telling ROS to get the time from the simulator.

The simulator window should pop up. The simulator is a ROS node that has some publishers and subscribers. Here are some ways of figuring out what's going on with a node:

```
rostopic list
rosnode info stage
```

## Issue some robot commands for stage


### Sensors

Let's start by looking at some of the robot's sensor output. You can see what these topics are publishing using rostopic:

```
rostopic type /base_pose_ground_truth
```

We can read from the robot's sensors and its location using the command rostopic:

```
rostopic echo /base_pose_ground_truth
rostopic echo /base_scan
```

- Goal 1: Create a new node (you can copy the listener node from last week) that listens to the base_scan topic. Listen on the /base_scan topic and print the closest reading (you'll have to loop through all the sensor readings).

### Actuators

Next up, let's look at how we can make the robot move. We can command the robot to move using the /cmd_vel topic. You can see how that message gets contructed:

```
rosmsg show geometry_msgs/Twist
```

If we change linear.x, then we can make the robot move forward and backward. If you change angular.z, you can make the robot turn.

- Goal 2: Write some nodes to make the robot move:
	- In a square
	- In a figure-eight
	- In a triangle


## Practice a ROS connection from one computer to another

Now let's play with different computers and ROS's network capabilities. First off, connect to the HRI_class link (password: schooliscool)

Check the IP address for your computer on the wifi:

```
ip a
```

You should get something like this:

```
2: wlp58s0: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 1500 qdisc noqueue state UP group default qlen 1000
    link/ether 3c:e9:f7:de:7b:00 brd ff:ff:ff:ff:ff:ff
    inet 10.0.0.20/24 brd 10.0.0.255 scope global dynamic noprefixroute wlp58s0
       valid_lft 83998sec preferred_lft 83998sec
    inet6 fe80::8079:c175:8a97:52a8/64 scope link noprefixroute 
       valid_lft forever preferred_lft forever

```

In this case, 10.0.0.20 is my cmputer's IP. find yours and share it with your neighbor. For each terminal, you can set your environment to look for a ROS core node on another machine using:

```
export ROS_MASTER_URI=http://<IP>:11311
```

Do this and run a node that'll move the robot in a simulator on another machine. Each of you can be running a simulator on your machine and run nodes pointed at the core node on the other machine. Lots of combinations will work, give things a try.

## Obstacle avoidance

Make one node that both reads from the laser message as well as publishes the cmd_vel topic as well. Now we want to make a robot that can move using the sensor data.

- Goal 3: Make a robot that can move forward, unless there's an obstacle close by, then it would stop.

- Goal 4: Make a robot that can move forward, then turn if there is an obstacle in front of the robot. It should turn in the direction with fewer obstacles.
