#!/bin/bash
p_x="-75.62998962402344"
p_y="-47.51282501220703"
p_z="0.0"
r_x="0.0"
r_y="0.0"
r_z="0.7570310443853707"
r_w="0.6533789083195102"

echo "P($p_x, $p_y, $p_z) R($r_x, $r_y, $r_z, $r_w)"
ros2 topic pub --once /localization/initialpose geometry_msgs/msg/PoseWithCovarianceStamped "
{header : {
    stamp : {
        sec: $(date +%s),
        nanosec: 0
    },
    frame_id : "map"
},
pose : {
    pose : {
        position : {
            x: $p_x,
            y: $p_y,
            z: $p_z,
        },
        orientation : {
            x: $r_x,
            y: $r_y,
            z: $r_z,
            w: $r_w,
        },
    },
    covariance : [
        0.25, 0.0,  0.0, 0.0, 0.0, 0.0,
        0.0,  0.25, 0.0, 0.0, 0.0, 0.0,
        0.0,  0.0,  0.0, 0.0, 0.0, 0.0,
        0.0,  0.0,  0.0, 0.0, 0.0, 0.0,
        0.0,  0.0,  0.0, 0.0, 0.0, 0.0,
        0.0,  0.0,  0.0, 0.0, 0.0, 0.068,
    ],
}}
"