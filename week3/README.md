# Week 3 Tutorial

This week we're going to consider programming social navigation. Major deliverables this week:

- Identifying a group from observing multiple people
- Recording data from the simulator into a rosbag
- Playing back data from a rosbag
- Identifying coalitions
- Moving to join a group


## Identifying a group from observing multiple people

There's a launch file `person_world.launch` that will load a simulator with 3 simulated people in it standing in a group. These people are also 'robots', so if you run the:

```
rostopic list
```

then you will see the main robot's topics show up as robot_0, and the other 'people' show up as robot_1, robot_2, and robot_3. You will need to adjust code you're writing, since the topic names have changed.

I've also put a script in the week3 directory, wiggler.py that will move your fake folks around so that they show up better to the leg detector. You could write nodes to make the fake people move, so that you can study still motion and movement.

Start coming up with some metrics for deciding if two 'people' that are detected are part of a group. Try writing code that uses these metrics to identify pairs and/or groups of people using these fake data.

## Recording data from the simulator into a rosbag

Review how rosbag works, and record some data with these fake people. You want to make sure to record the topics that the leg detector is publishing, the laser scan, and the tf topic.

It is also good to record several different data sets, some that show groups and some that don't (label them as such) with the people arranged in different places and different configurations. You can use these for testing for true/false positives and true/false negatives.

## Playing back data from the rosbag and viewing it in rviz

Use `rosbag play` to playback the data that you've recorded. If this works, you should be able to view the recorded people in rviz.

## Identifying a group from observing multiple people

Write code to identify if people are standing in a line and if people are standing in a circle. This should be a standalone node that takes in a People message (like from the leg_detector node) and updates the name string to prepend the person name with the group name (i.e., person_1 would become circle_1_person_1 or line_1_person_1). This new topic should be published on a topic: `/robot_0/detected_groups`.

## Moving to join a group

Write a new node that waits for your group detector to detect and group and then, move to the center of a circle (bad, I know!), or to the end of a line. This node should read in a People message and output a cmd_vel message.