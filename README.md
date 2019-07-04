# Visibility Graph Path Planning

## Introduction
Implementation of visibility graph (vgraph) path planning algorithm using the ArbotiX turtlebot simulator and visualization of the graph and robot path following in RViz.

## Usage
*separate terminals*
roslaunch vgraph launch.launch
python main.py

### Files
- `data/world_obstacles.txt`: Text file indicating obstacle locations (in **cm**).
  - First line is the total number of obstacles in the world.
  - Each obstacle starts with a number specifying total number of vertices, followed by these vertices' location in counter-clockwise order.
- `data/goal.txt`: Location of the goal (in **cm**).
- `src/create_map.py`: Creates or updates the `map.png` under `maps/` folder, using obatacles and goal defined in the above files.
- `launch/launch.launch`: Launch file to start everything you need.
- `maps/map.png`: Image of the map.
- `maps/map.yaml`: Map metadata.
- `package.xml`: Package metadata.
- `world.rviz`: RViz configuration file.
[This Base Package from](https://github.com/jingxixu/vgraph)

### Program
This project builds off of my Previous Project ["Bug2PathPlanning"](https://github.com/s-abdullah/Bug2PathPlanning). The methods in class OutAndBack have already
been explained there.

**readObs**, **growPoints** and **showGrow** take in the corners of each object and grow
them by adding a 36x36cm square to each vertex. The reference point is taken to be
the center, so this expands the vertices out 18cm out. It outputs the new
obstacles to Marker Array.

The line class handles connecting the graph at each vertex of each visible obstacle.
The functions assess intersection points and edge cases such as:
collinear and overlapping, collinear but not overlapping, parallel and
not intersecting.

Lastly, the main function handles djikstra's path finding algorithm.
This is denoted with a comment.

### Output
2. Grown the obstacles using reflection algorithm (fliped the robot around the reference point, placed at each vertex of the obstacle, and created a convex hull). Assumed the robot to be a 36cm-by-36cm square. 

3. Created the vgraph by first fully connecting all obstacle vertices + start + goal and then implementing a collision checker to remove edges that collide with obstacles (except endpoints).

4. Dijsktra's Algorithm implemented to get the shortest path.


5. Robot made to follow the shortest path.

![alt text](https://github.com/s-abdullah/VGraph/blob/master/gifs/vg.gif)