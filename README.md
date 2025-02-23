<div align="center">
    <h1>Simple ROS2 node simulating odometry msgs from csv files</h1>
</div>

## :package: Prerequisite packages
> What we need are ros2 humble, pandas and numpy.

```commandline
sudo apt-get install ros-humble-ros-gz-bridge python3-numpy python3-pandas -y
```
## :gear: How to build & Run
```commandline
git clone https://github.com/leonardonels/AC_ros2_out_project.git
cd AC_ros2_out_project
```
> use '-l' to loop the csv input file
```commandline
python3 output_node.py -l
```

## Todo List

- [ ]  apply the covariate matrices
