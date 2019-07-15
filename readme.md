# deptheye_ros

Proof-of-concept support for the DepthEye 3D TOF depth camera from http://pointcloud.ai and seedstudio: https://www.seeedstudio.com/DepthEye-3D-visual-TOF-Depth-Camera-p-3025.html

Publishes a PCL depth cloud at /points and a depth sensor_msgs::Image (no CameraInfo) at /image_raw.

Uses caktinised files moved around from the DepthEye repo: https://github.com/pointcloud-ai/DepthEyeSdk
The only original file of mine is `src/deptheye_node.cpp`, and a small change to `src/DepthEyeInterface.cpp` to request the 
The TI chip's SDK for reference: https://github.com/3dtof/voxelsdk

Clone the repo into your catkin workspace's src. You will need to have the environment variable `VOXEL_SDK_PATH` set before building and also for running (it loads some config files during startup).
This will run using the devel workspace– I haven't done any work to make it installable.

```
export VOXEL_SDK_PATH=`rospack find deptheye_ros`
catkin_make
rosrun deptheye_ros deptheye_node
```