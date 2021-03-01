# RRT Planner
Author: Kevin Ma

## Intro
This package includes:
- a map server that reads jpg/png/pgm files and publishes nav_msgs/OccupancyGrid msgs.
- a planner based on Rapid-exploring Random Trees algorithm

## How to use
> Make sure you have installed SDL libraries, if not, install with the commands:
> ``` bash
> sudo apt-get install libsdl-image1.2-dev
> sudo apt-get install libsdl-dev
> ```

Copy the `rrt_planner` folder to `<your_workspace>/src/`, and in `<your_workspace>`, run:
``` bash
source /opt/ros/melodic/setup.bash
catkin_make
source devel/setup.bash
```

To launch everything run:
``` bash
roslaunch rrt_planner planner.launch.xml [map:=<path/to/map> res:=<resolution>]
```
You should be able to see rviz being launched with the map being displayed at the centre.

In rviz, Use "2D Pose Estimate" to send initial position, and "2D Nav Goal" to send goal. And enjoy the planning animation!

After the planning is done, you can pick another set of init and goal directly.

To change map, you can either add the `map` argument in commandline (as shown above), or modify the `launch/planner.launch.xml`

## Implementation details

### map server
This map server is modified from the [map_server package](https://github.com/ros-planning/navigation/tree/noetic-devel/map_server) in ROS navigation stack. I used the `image_loader` library for loading images from various formats using SDL. The map_server node is modified so that no yaml file is needed and image files are directly loaded.

To run the map server alone:
```
rosrun rrt_planner map_server <path/to/map> [resolution]
```

The map is published to the topic `/map` as `nav_msgs/OccupancyGrid`

### RRT planner
The RRT planner has two parts: the RRT data structure library and the planner node. The RRT implemented is the minimal vanilla RRT with some trivial heuristics biasing the goal.

The planner accepts map from `/map` topic, initial pose and goal from rviz. It publishes visualization markers during the planning and publishes a `nav_msgs/Path` msg to `/path` topic after it's done.

To run the planner alone:
```
rosrun rrt_planner planner [goal_bias] [delta]
```

Credit to [this repo](https://github.com/donrv/rrt-ros) as reference for the code structure.
