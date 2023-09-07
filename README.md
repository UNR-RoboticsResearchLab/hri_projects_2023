# hri_projects_2023
Course content for UNR CS4/691 - Human-Robot Interaction

## set up ROS environment

If you have not yet installed ROS, you can do so in just a few lines (and just a few minutes) like so:

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-noetic-desktop-full
```

Note: this assumes that you're running 20.04 and installing the noetic distribution


You should be able to now add ROS to your environment variables like so (note you have to do this once per terminal:

```
source /opt/ros/noetic/setup.bash
```

This will install the packages for a base ROS distribution. Just for funzies, let's take a look at what that setup.bash file did:

```
export | grep ROS
```

Will show us all the environment variables needed for the ROS configure. The most relevant variables will be ROS_DISTRO, ROS_ROOT, ROS_PACKAGE_PATH, and ROS_MASTER_URI. Look up at Dave, now.

## set up your ROS workspace

We're not done, yet. Now, we need to create a place for the ROS packages that we (and others) write to go. This is called a workspace. This is easy to do and can help you keep track of multple versions of ROS installations or package configurations. 

To start we need to create a directory:

```
cd ~
mkdir -p hri2023/src
cd ~/hri2023/src
cd ~/hri2023
catkin_make
```

You'll see some compile nonsense that hasn't really done anything other than set up the workspace. Let's look at it now.

If you type

```
ls
```

You'll see there's now a build and devel directory. Let's concentrate on devel for now. if you look in that directory you'll see a lot of setup files, these help ROS find the developed files in this workspace. Let's use one of them:

```
source ~/hri2023/devel/setup.bash
```

This will install the packages for a base ROS distribution. Just for funzies, let's take a look at what that setup.bash file changed:

```
export | grep ROS
```

ROS_PACKAGE_PATH= now points to your workspace directory (along with the default ROS package directory). Look up at Dave, now.


## fork and clone repository

First, navigate to the hri_projects_2023 page: https://github.com/UNR-RoboticsResearchLab/hri_projects_2023

You should create a version under your github id. Then clone your repository:

```
cd ~/hri2023/src
git clone https://github.com/<YOUR GITHUB ID HERE>/hri_projects_2023
```

this will clone the repo into your ROS package source directory.

Next we want to test the compile of the system:

```
cd ~/hri2023
catkin_make
```

assuming all goes well, then you should be able to get to the week 0 package using the roscd command:

```
roscd week0
```

if you've gotten to the directory: ~/hri2023/src/hri_projects_2023/week0

## Navigate to the week 0 project readme

Each project week will have a readme file with instructions for the week, navigate to the week1 directory on github to see the instruction for the week.

If you get lost, the direct link is: https://github.com/UNR-RoboticsResearchLab/hri_projects_2023/tree/main/week0

## committing a change to your repository

when you're done try commiting to the repo:

you want to add any new files with:

```
git add <filename>
```


```
git commit -m 'INSERT MEMO HERE'
git push
```
