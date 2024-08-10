define laser ranger(
    sensor(
        range [0.0 5.6]
        fov 240.0
        samples 240
    )

    color "red"
    edge_color "white"
    size [0.014 0.014 0.014]  

    block(
        points 36
        point[0] [0.035 0.0]
        point[1] [0.034 0.006]
        point[2] [0.033 0.012]
        point[3] [0.030 0.018]
        point[4] [0.027 0.023]
        point[5] [0.024 0.028]
        point[6] [0.020 0.032]
        point[7] [0.016 0.034]
        point[8] [0.012 0.035]
        point[9] [0.006 0.034]
        point[10] [0.000 0.035]
        point[11] [-0.006 0.034]
        point[12] [-0.012 0.033]
        point[13] [-0.016 0.032]
        point[14] [-0.020 0.028]
        point[15] [-0.024 0.023]
        point[16] [-0.027 0.018]
        point[17] [-0.030 0.012]
        point[18] [-0.033 0.006]
        point[19] [-0.035 0.000]
        point[20] [-0.034 -0.006]
        point[21] [-0.033 -0.012]
        point[22] [-0.030 -0.018]
        point[23] [-0.027 -0.023]
        point[24] [-0.024 -0.028]
        point[25] [-0.020 -0.032]
        point[26] [-0.016 -0.034]
        point[27] [-0.012 -0.035]
        point[28] [-0.006 -0.034]
        point[29] [0.000 -0.035]
        point[30] [0.006 -0.034]
        point[31] [0.012 -0.033]
        point[32] [0.016 -0.032]
        point[33] [0.020 -0.028]
        point[34] [0.024 -0.023]
        point[35] [0.027 -0.018]
        z [0.1 0.1]
    )  
)

define robot_0 position(
    odom_error [0.03 0.03 0.00 0.05]
    localization "odom"
    size [0.4 0.4 1.2]  # Smaller size
    gui_nose 1
    drive "diff"
    color "black"
    edge_color "navy"
    block(
        points 12
        point[0] [0.05 0.03]
        point[1] [0.03 0.05]
        point[2] [0.027 0.0458]
        point[3] [0.034 0.0388]
        point[4] [0.0272 0.0356]
        point[5] [0.034 0.0326]
        point[6] [0.025 0.03]
        point[7] [0.034 0.0272]
        point[8] [0.0276 0.024]
        point[9] [0.034 0.0212]
        point[10] [0.027 0.0136]
        point[11] [0.03 0.01]
        z [0.00000 0.00000]
    )
    block(
        points 3
        point[0] [0.0449 0.0351]
        point[1] [0.0456 0.0402]
        point[2] [0.04 0.04]
        z [0.00000 0.00000]
    )
    block(
        points 3
        point[0] [0.0449 0.0249]
        point[1] [0.0454 0.0198]
        point[2] [0.04 0.02]
        z [0.00000 0.00000]
    )
    block(
        points 3
        point[0] [0.042 0.0302]
        point[1] [0.0392 0.028]
        point[2] [0.0392 0.0322]
        z [0.00000 0.00000]
    ) 

    laser(pose [0.20 0.0 -1.2 0.09])  
)