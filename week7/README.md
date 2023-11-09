# Week 7 Tutorial

This week we're going to play with a simulated nao.

## Install nao packages

So, they stopped packaging the nao

```
sudo apt install ros-noetic-naoqi-driver ros-noetic-move-base-msgs ros-noetic-nao-meshes
cd ~/hri2023/src
git clone git@github.com:ros-naoqi/nao_robot.git
cd ..
catkin_make
```

This will add the nao packages to your workspace and compile them.

## Running the nao simulation


- Rather than running a full nao simulator, we're going to just play with a model nao in rviz. Let's start with loading a description of the nao into ROS:

```
roslaunch nao_description robot_state_publisher.launch
```

- In another terminal, start up an instance of `rviz`, add a RobotModel.

- Finally in a third terminal, let's run a gui publisher:

```
rosrun joint_state_publisher_gui joint_state_publisher_gui
```

This will load up a gui that can move each joint individually. If you want to see what the gui is publishing, you can use rostopic echo to see:

```
rostopic echo /joint_states
```

## Keyframe animation with the Nao

Now, we'll play with some animation. Instead of using the gui, we'll write a node to publish to publish to that `/joint_states` topic. For an action:

- prototype some key frames in the gui
- save the joint values for those keyframes
- put them into a ROS node that publishes those frames in sequence, interpolating between them


