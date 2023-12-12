# Week 5 Tutorial

This week we're going to record a perception dataset for different forms of groups/coalitions for comparison.

## Week4 Results

Once you've completed your week4 tasks, we're going to start off by recording some data of the group behavior for 3 different conditions:

- no group interaction;
- line group interaction; and
- circle group interaction.

## Make 5 recordings for each category

Use `rosbag record`. Here's the requirements for a good recording:

- complete: all data required to recreate the group observation from a recording are present (this means recording all the correct topics)
- compact: if every file is 1G large, then it won't be easy to move around (files should definitely be less than 100M)
- distinct: each data recording should be different from others, the same data over and over are bad for training
- documented: each dataset should have some notes about the dataset, what date/time recorded, a clear text description of the data that are included. There is a `.list` file in each directory where you can record this information

If you make a good recording, you should be able to see the data visualized well in rviz if you play it back (`rosbag play`).

## Share with others, and see if your week4 classifier works

Just as the title says, test your classifier against other datasets. How did it do? Would you change things to make it work better?

