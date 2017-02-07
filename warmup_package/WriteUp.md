#Table of Contents
1. [Project Overview](#project-overview)
2. [Running the Code](#running-the-code)
3. [Understanding the Code](#understanding-the-code)
4. [Process, Takeaways, and Improvements](#process-takeaways-and-improvements)


#Project Overview
This was a two-person "warmup project" for the Introduction to Computational Robotics Class at Olin College of Engineering, taught by [Paul Ruvolo](https://github.com/paulruvolo) in Spring 2017. The project was aimed to be an introduction to using ROS for mobile robotics, processing data from sensors, programming robot behaviors using reactive control strategies, and using finite-state control. 

To meet these learning goals, we were expected to use Python scripts and ROS to:
* Use RViz for visualization and debugging: 
  * Visualize incoming sensor data
  * Create markers to visualize important points in code (e.g. a representation of the point the robot is following)
  * replay rosbag data 
* Create a teleoperation mode for the robot
* Make the robot drive in a 1 meter by 1 meter square
* Drive until it encounters a wall, at which point it turns to follow it (parallel to the wall)
* Dynamically follow a person or other object
* Avoid obstacles
* Combining multiple behaviors using finite-state control

Full project description and expectations can be found on the course website, [here](https://sites.google.com/site/comprobo17/projects/warmup-project).


#Running the Code
##Neato Requirements
If you are in CompRobo or are an instructor of CompRobo, skip to the next subsection.
First, you will need to have a Neato equipped with the RasPi/LIDAR/etc. setup ready to go. Make sure you're connected to the OLIN-ROBOTICS wifi network.
In order to connect to the neato, clone Paul's [CompRobo repo](https://github.com/paulruvolo/comprobo17) into your catkin workspace (often ~/catkin_ws), and run `$ catkin_make`in the directory. This contains the rosnodes necessary for connecting to the neato. 
To connect to a neato, run: 

``` 
$ roscore
```

And in a separate terminal window, run:

```
$ roslaunch neato_node bringup.launch host:=IP.ADDRESS.OF.ROBOT
```

For more details, follow the Neato connection guide written by Paul [here](https://sites.google.com/site/comprobo17/how-to/neato-etiquette).

##Running Our Code
To be able to run our code, clone this repo into your catkin workspace (often ~/catkin_ws), and run `$ catkin_make`in the directory. You should now have warmup_package as a ROS Package.
To run any of the scripts in the warmup_project_2017/warmup_package/scripts/ directory, run:

```
$ rosrun warmup_package SCRIPTNAME.py
```

Scripts meant to be run include: 
* __teleop.py__: Use teleoperation controls to drive the robot remotely.
* __drive_square.py__: Autonomously drive the robot in a 1x1m square. (It will drive forward 1m then left 1m so make sure you have space.) 
* __wall_follow.py__: Autonomously drive the robot straight, until it sees a wall, at which point it will turn parallel to the wall to follow it.
* __person_follow.py__: The robot will attempt to follow any object in front of it. This includes moving people, or like... walls. (Don't worry, it has an e-stop bumper.)
* __obstacle_avoid.py__: The robot will avoid obstacles, and try to return its original angle of travel.

(Read more about what each of these scripts does in the [next section](#understanding-the-code)!)

If the scripts don't run, make sure the python files are executable (e.g. try running `chmod u+x FILENAME`). The package and script names should tab complete after the `rosrun`.

#Understanding the Code
Each script does a different task, although there are some that build off of previous functionalities. We'll explain the architecture of our code here so they build off of each other in order.

##Teleoperational Command - `teleop.py`
Teleop Command uses nine keyboard keys (u i o / j k l / m , .) to provide remote control steering for the Neato. In order to do this, we initialize a rosnode that publishes to the topic `/cmd_vel`, which is a Twist geometry message. Inside the main loop, we first print instructions to the terminal for the driver to use, then pass key inputs through a function of our creating that assigns velocities depending on what key is pressed. After publishing these velocities as a Twist message, we then tell rospy to sleep so the loop executes at 10Hz (so it doesn't eat your CPU with this while loop).

We wrote this script by creating a teleop class, which is particularly nice for our purposes because we don't have to be passing variables back and forth and creating lots of new ones -- it's all captured under self.linear and self.angular vectors. This also sets us up nicely to create more complicated scripts later that pass in other classes. But we'll get to that. Yay for object oriented programming!

##Drive in a Square - `drive_square.py`
We chose to execute our drive square function by subscribing to the Neato's internal odometry to sense where it was at any given point, under the `/odom` rostopic. We carried over the keyboard input from the Teleop script in order to let the person chose to run as many or as few squares as they like -- at the beginning of the program and after every square, it gives you the option to press the space bar to start a square or Ctrl-C to exit. It also similarly publishes to the `/cmd_vel` topic, sending linear and angular velocities. 

We wrote three functions that control the robot at different times, which allow the robot to turn, drive forwards, or stop. This script is also object oriented, with only one function outside the main Class (it was written by Paul and supplied as a helper function to convert the odometry quaternions to an (x,y,theta) position). The main loop of our function calls the first time the robot drives forwards, and from there, the robot alternates between driving forwards and turning 90deg until it has completed the square, at which point the program essentially restarts and asks the user whether they'd like to start another square. If at any point the user hits Ctrl-C or the bump sensor on the front of the robot is activated, the program ends and the node shuts down. 

##Wall Following - `wall_follow.py`
For wall following, we decided to not use odometry for robot movements, but rather base our movements purely based on subscribing to data from the onboard LIDAR, via the rostopic `/scan`. We also subscribed to the `/bump` topic in order to implement an e-stop if the robot doesn't correctly follow the wall and instead drives into something. As laser scan data is published to `/scan`, our script processes it immediately to see if and where any walls or other objects are, and sets a flag in our code if there is a wall present. In order to tell which way to turn to avoid the wall, we find the angle that the closest point to the robot is at from the robot's straight ahead, and turn away from it using proportional control. As the wall is no longer detected in front of the robot, we continue travelling in this new forwards direction, leading us to move parallel to the wall, or "follow" it.
We publish a marker in the '/visualization_marker' topic to illustrate what our neato is sensing. The closest point is identified as part of the processing the laser scan, and this is passed to the 'make_marker' function, also within the wall follower class. There is another class, however, called Sphero, that simply holds all of the attributes needed to publish a marker. This class is used by the make_marker function, and the marker is published during the run loop.

##Dynamic Person Following - `person_follow.py`
Person following uses much of the same sensing mechanics as wall following, except that instead of finding the closest point in a scan, the neato identifies the the center of what it is sensing, presumably a person, and moves toward that center. This is accomplished through the "find_target" function in the Person Follower class. Find_target takes all the identified points within our range averages all the angles and all the distances using numpy to find the approximate center in polar coordinates, which is then passed to the marker class like the wall follower.
Another key difference between the wall follower and the person follower is how they utilize proportional control in their movements. The wall follower turns more and more as it approaches the wall, so in the code the turning speed is divided by the distance. In the person follower, the neato must accelerate to close larger distances faster, so it multiplies a set speed by the its distance from the person center.
[Video Demo!](https://www.youtube.com/watch?v=5vwHkopVCrM)

##Obstacle Avoidance and Finite-State Control - `obstacle_avoid.py`
If you'd like to see a video demo in addition to the rosbags in the /warmup_project_2017/bags directory, you can view our obstacle avoidance working [here!](https://www.youtube.com/watch?v=7-8pAAtLgBg)

Obstacle avoidance is where we decided to implement finite state control. We have two states: travel in the original direction, and avoid obstacle, which you can see in the diagram below.
![Finite State Diagram](https://github.com/laurengulland/warmup_project_2017/blob/master/warmup_package/finite_state_diagram.png "Finite State Diagram")
The triggers to switch between states are based on laser scan data -- specifically, whether there is an object in the robot's view or not. When the robot is swtiching back after succesfully avoiding an object, it attempts to return to the original angle of travel, rather than just continuing in the direction the obstacle redirected it into.

#Process, Takeaways, and Improvements
