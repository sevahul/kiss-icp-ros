# This repository is obsolete. Now when kiss-icp is open sourced, please just use the original implementation :)
# kiss-icp-ros
![alt text](https://github.com/sevagul/kiss-icp-ros/blob/main/media/result.png?raw=true)<br/>
A wrapper around kiss-icp package that includes kiss-icp ros-node. <br/>
Please note, that the code was developed at the moment when source code was not available, so it uses binaries provided with pip package. It might not be the best way to use kiss-icp nowadays, as there is no way to change the source code from here, and the version might be old / not working
# Licensing
This code uses [kiss-icp](https://github.com/PRBonn/kiss-icp/blob/main/LICENSE) -> see [kiss-icp-license.txt](https://github.com/sevagul/kiss-icp-ros/blob/main/kiss-icp-license.txt)

## Dependencies
```
pip3 install kiss-icp scipy
```
If fail, try to follow the original kiss-icp [README](https://github.com/PRBonn/kiss-icp), install older version of `numpy`, update `pip`, etc. <br/>

## Usage:
To run kiss-icp, run
```
roslaunch kiss_icp run_icp.launch [config:=<your config>] [points_in_topic:=<your point cloud topic>]
```
To run it on the bag-file, run
```
roslaunch kiss_icp run_icp_bag.launch [bagfile:=<your bagfile>] [config:=<your config>] [points_in_topic:=<your point cloud topic>]
```
