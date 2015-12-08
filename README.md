Baxter Pick and Place
=============================================

Rikki Irwin, Matt Cruz, Nathan Corwin, Ayush Sharma
---------------------------------------------


#### Table of Contents ####
[Project overview](#Project Overview)

[Requirements](#Requirements)

[Detecting the block with AR tracking](#Vision)

[Moving above the block](#Movement)

[Adjusting the height](#fine)

[Dropoff](#drop)

[Instructions for running the files](#Instructions)


#### Project Overview  <a name="Project Overview"></a>
The main goal of this project was to use the Baxter robot to autonomously pick objects off of a work surface and place them in a container. This was the final project for ME495: Embedded Systems in Robotics at Northwestern University. The task is split into four main states which are implemented through a state machine:
* Locate a block with tags using ar_track_alvar
* Move to above the block position using a joint trajectory
* Adjust the height using a Cartesian trajectory
* Grab the block and drop it in a specified position

#### Dependencies <a name="Requirements"></a>

  *  Baxter SDK - follow the [Workstation Setup](http://sdk.rethinkrobotics.com/wiki/Workstation_Setup) Instructions
  * pykdl_utils package - clone [this](https://github.com/gt-ros-pkg/hrl-kdl.git ) repository
  * ar_track_alvar package with apt-get installa ar-track-alvar
  * urdf package with apt-get install ros-version-urdfdom-py
  * kdl package with apt-get install ros-version-kdl-conversions


#### Detecting the block with AR tracking  <a name="Vision"></a>
In this state, ar_track_alvar is used to find the block positions and orientations. Each block has a unique tag; if multiple blocks are found, the block that is closest to the current position will be used. 

#### Moving above the block  <a name="Movement"></a>
In this state, a joint trajectory is calculated to move between from the current position to 10 mm above the block. The joint angles for the gripper position at the block is calculated through inverse kinematics and transformed to the robot's base frame. The joints are then linearly interpolated and the joint positions are sent sequentially to move the gripper to the desired position.

#### Adjusting the height  <a name="fine"></a>
In this state, a Cartesian trajectory is used to hold the orientation of the gripper constant and just change the height as the gripper moves closer to the block. The laser range data from Baxter's hand is used in a feedback loop to determine when the gripper has reached the block.


#### Dropoff <a name="drop"></a>
In this state, the gripper grabs the block and moves to drop it off in a specified location. The blocks are sorted into two groups using metadata from the AR tags to specify which dropoff location to use.

#### Instructions for running files  <a name="Instructions"></a>

To run the files, the workspace must be connected to Baxter and properly sourced. Then use the following command: `roslaunch baxter_pick_and_place baxter_vis_launch.launch`
