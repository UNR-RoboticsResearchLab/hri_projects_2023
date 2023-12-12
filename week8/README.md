# Week 8 Tutorial

This week we're going to play some more with the simulated nao.

## Look at hand / look at where the hand is pointing

Let's play around with TF2 and our robot model to make some more complex gestures:

- Start by making a node to look at the hand. You can do this by figuring out which angle a frane on the robot (you could use one of the camera frames or the Head frame has to the hand). Once you have the angles, then you can change HeadYaw and HeadPitch to point at those angles.
- Next let's make a node that publishes a new frame, 1 meter from one of the hand frames (make sure it looks right in rviz). Instead of looking at the hand, write another node to look at the place the hand is pointing to (your new frame).

## project turn-in

Your next project turn-in will include:

- The two nodes above
- The keyframe animator we made last week
- A node to smoothly move the head to the target angle, rather than snapping it directly to the angle in question


