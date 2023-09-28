# Week 4 Tutorial

This week we're going to consider classifying a group of people. Major deliverables this week:

- Identifying a group from observing multiple people (from last week)
- Classifying coalitions
- Avoiding a crowd


## Identifying a group from observing multiple people (revisiting last week)

Write code to identify if people are standing in a line and if people are standing in a circle. This should be a standalone node that takes in a People message (like from the leg_detector node) and updates the name string to prepend the person name with the group name (i.e., person_1 would become circle_1_person_1 or line_1_person_1). This new topic should be published on a topic: `/robot_0/detected_groups`.

It seems like last week's stuff might have been left over so I wanted to carry this into this week.

## Classifying coalitions

Let's now publish a new message type that you'll create, Group. A Group will have an array of People messages, plus a header saying when the data was most recently received, and a Pose representing the center of the group. You should have a float named `size`. For a line, this should be the length of the line the people are standing in, for a circle it should be the radius of the circle.

Your node should also publish a [visualization message](http://wiki.ros.org/visualization_msgs). If this is done right, and you add that to rviz, you'll be able to visualize where the group is. For a line, draw a box around the people in the line (oriented correctly along the line). For a circle, draw a circle for the o-space and another for the p-space.

Your node should also be publishing a frame with the center of the line or the circle.

## Avoiding a crowd

We will assume that crowds that are in a line move from 0-1 m/s, and that circles stay put. We want to create some social navigation scenarios and play with them.

*Hint: using TF2 here might help you out with the navigation goals*

### Scenario 1: Circle

Arrange the simlulated people in a circle. Have the robot move to a goal that would have it cross where the circle-group is standing. Write a motion controller that pays attention to the group's location and avoids it.

### Scenario 2: Line

Arrange the simulated people into a line. Write a node to move that line straight in a direction. Have the robot move to a goal that would have it cross where the line-group is moving. Write a motion controller that pays attention to the group's location and avoids it.