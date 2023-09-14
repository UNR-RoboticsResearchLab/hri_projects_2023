# Week 2 Tutorial

This week we're going to move the focus from basic robot actions to actions around people. Major deliverables this week:

- Detecting a person in simulation with a laser
- Using the transform library and ROS visualizer (rviz)
- Moving toward a goal with obstacle avoidance
- Creating a data recording using rosbag
- Using a rosbag recording

## detecting a person in sim with a laser

First, let's open up our simulator again:

```
roslaunch week1 basic_world.launch
```

In this world, we just have a robot and a box. Close that down, now and I'm making a new world file to include a 'person', let's take a look:

```
roslaunch week2 person_world.launch
```

We now have a 'person' in this world, which is just two circles. On a laser, those will look like legs. Now we can play around with a new package, leg_detector:

```
sudo apt install ros-noetic-leg-detector
roslaunch week2 leg_detector.launch
```

Now, you can run that in addition to the simulation in order to spot the legs. One good way to see where the person is is to use the ROS visualizer (rviz)

## Using the transform library and ROS visualizer (rviz)

rviz should already be installed, but just in case, you can install its package:

```
sudo apt install ros-noetic-rviz
```

and run it with:

```
rosrun rviz rviz -d ~/hri2023/src/hri_projects_2023/week2/people.rviz
```

This config file includes a number of visualizers for the robot's laser, the robot's odometry, pose, and the detected legs. Play around with it a bit to see how it works.

Now, take a break to look at the TF library: http://wiki.ros.org/tf2 there are some tutorials that will help you out.

Write a node to use TF2Broadcaster to publish the positions of the detected legs/people as TF frames.

Note lines like this that help you convert Euler angles to quaternions:

```
q = tf_conversions.transformations.quaternion_from_euler(0, 0, msg.theta)
```

If this works, you should see a frame show up in Rviz in the proper place.

## Moving toward a goal with obstacle avoidance

Now look back at your obstacle avoidance node from week1. Re-write this node to do more than just go straight, it should head toward the detected person without hitting any obstacles that might get in its way.

*** This will be the first project turn-in, due next Friday, 9/22, AOE ***

## Creating a data recording using rosbag

Read through the tuturials on rosbag: http://wiki.ros.org/rosbag

While we won't have a specific turn-in this week, next week's programming exercises will use this.

## Using a rosbag recording

Dave's going to record a rosbag with some laser-based people detection on a real robot, you'll view it using `rosbag play` and rviz.