# realsense_d435_gazebo

Gazebo simulation package for Intel RealSense D435 and D435i cameras. It contains the camera URDF/xacro models, D435 mesh, and the integrated Gazebo plugin libraries needed by those models.

## Third-party code

This package combines and trims files adapted from:

- [m-tartari/realsense_gazebo_description](https://github.com/m-tartari/realsense_gazebo_description)
- [m-tartari/realsense_gazebo_plugin](https://github.com/m-tartari/realsense_gazebo_plugin)

The integrated plugin source keeps the original Apache 2.0 license headers and upstream acknowledgements for Intel Corporation, PAL Robotics, SyrianSpock, and other RealSense Gazebo plugin contributors.

## Use

Include the D435 macro from another xacro file:

```xml
<xacro:include filename="$(find realsense_d435_gazebo)/urdf/_d435.urdf.xacro"/>
<xacro:sensor_d435 parent="base_link" name="D435_camera" topics_ns="D435_camera">
  <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
</xacro:sensor_d435>
```

Include the D435i macro when IMU frames/topics are needed:

```xml
<xacro:include filename="$(find realsense_d435_gazebo)/urdf/_d435i.urdf.xacro"/>
<xacro:sensor_d435i parent="base_link" name="D435i_camera" topics_ns="D435i_camera">
  <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
</xacro:sensor_d435i>
```

For multi-camera simulations, give each instance a unique `name` and `topics_ns` to avoid frame, sensor, plugin, and topic conflicts.

When `align_depth` is true, the model publishes aligned depth topics such as `aligned_depth_to_color/image_raw` instead of the regular `depth/image_rect_raw` pair.
