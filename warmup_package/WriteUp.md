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
* __person_follow.py__: The robot will follow any object in front of it. 
* __obstacle_avoid.py__: The robot will avoid obstacles, and try to return its original angle of travel.
More on each of these scripts in the next section!

If the scripts don't run, make sure the python files are executable (e.g. try running `chmod u+x FILENAME`). The package and script names should tab complete after the `rosrun`.

#Understanding the Code

#Process, Takeaways, and Improvements
