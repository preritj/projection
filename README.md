## Bounding box visualization for DiDi 

### Instructions

* Download the project to the `src` of your catkin workspace. For example,
```
cd ~/catkin_ws/src/
git clone https://github.com/preritj/projection.git

```

* Make sure all python files in source directory (`~/catkin_ws/src/projection/scripts/*.py`) are executable. 
Use the command `chmod +x` for this step.

* Build the catkin package followed by `source ~/catkin_ws/devel/setup.bash`. 
This last step has to be performed for every new terminal.

* To play the rosbag with bounding box visulaization on camera, 
```
roslaunch launch/projection.launch bag:=<absolute_path_of_bag_file>
```
