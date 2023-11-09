# Week A Tutorial

This week we're going to play with speech recognition and speech generation.

## Setup

We're going to install a package called ros_vosk that's pretty useful for this:

```
cd ~/hri2023/src
git clone git@github.com:alphacep/ros-vosk.git
cd ..
catkin_make
```

Now you want to follow the instructions 2- on the project's README page: https://github.com/alphacep/ros-vosk

Now we're going to go back to our weekA directory to develop our nodes.

## Speech Recognition

Play around a bit with the speech recognition node:

```roslaunch ros_vosk ros_vosk.launch```

*(note: this also loads a speech generator node)*

Listen to each of the topics published by the `speech_recognition` node and how parsing of whole and partial utterances work.

## Speech Generation

Vosk comes with a *very* basic speech generator node. You can publish strings to the `tts/phrase` topic to speak them.

* Write a node to listen for speech and then repeat that speech back
* Write a node that listens for speech commands from a user and then moves the robot (use actions from weeks 7 and 8) in response
* Write a node that asks some yes/no questions, waits for a speech response from a user, then changes its response based on the answers to those questions