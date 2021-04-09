## Clean build

> `tools/autoware_auto_cmake/autoware_auto_cmake.cmake`
```cmake
# suppress warnings as errors
# -Werror
```

```bash
colcon build --packages-skip rubis* --cmake-args -DBUILD_TESTING=OFF -Dtf2_INCLUDE_DIRS="/home/rubis/ros2_foxy/install/tf2_sensor_msgs/include/;/opt/ros/foxy/include/"
```

### Error handling
> `perception/filters/off_map_obstacles_filter`

```bash
In file included from /home/rubis/AutowareAuto2/src/perception/filters/off_map_obstacles_filter/src/off_map_obstacles_filter.cpp:15:
/home/rubis/AutowareAuto2/src/perception/filters/off_map_obstacles_filter/include/off_map_obstacles_filter/off_map_obstacles_filter.hpp:30:10: fatal error: visualization_msgs/msg/marker_array.hpp: No such file or directory
   30 | #include "visualization_msgs/msg/marker_array.hpp"
      |          ^~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
```

>> `perception/filters/off_map_obstacles_filter/package.xml`

```xml
...
<!-- add dependency -->
<depend>visualization_msgs</depend>
...
```

## Build

```bash
source ./install/setup.bash
colcon build --packages-select rubis_base
```
