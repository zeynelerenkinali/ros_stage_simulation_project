define laser ranger(
    sensor(
        range [0.0 5.6]
        fov 240.0
        samples 240
    )

    color "white"
    edge_color "white"
    size [0.035 0.035 0.035]  

    block(
        points 3
        point[0] [0 -0.5]
        point[1] [0.25 -1]
        point[2] [0.1 1]
        z [0 1]
    )  
)

define robot_0 position(
    odom_error [0.03 0.03 0.00 0.05]
    localization "odom"
    size [0.1 0.1 0.3]  # Smaller size
    gui_nose 1
    drive "diff"
    color "gray50"
    edge_color "navy"
    

    laser(pose [0.060 0.0 -0.025 0.05])  
)