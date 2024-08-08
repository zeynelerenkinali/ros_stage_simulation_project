define laser ranger(
	sensor(
		range [0.0 5.6]
		fov 240.0
		samples 240
	)

	color "gray"
	size [0.04 0.04 0.04]  

	block(
		points 6
		point[0] [0 0.5]
		point[1] [0.25 1]
		point[2] [0.75 1]
		point[3] [1 0.5]
		point[4] [0.75 0]
		point[5] [0.25 0]
		z [0 1]
	)  
)

define robot_0 position(
	odom_error [0.03 0.03 0.00 0.05]
	localization "odom"
	size [0.2 0.2 0.1]
	gui_nose 1
	drive "diff"
	color "black"
    edge_color "gray"
	laser(pose [0.11 0.0 -0.07 0.0])
    block(
        points 6
        point[0] [0.05 0.1]
        point[1] [0.07 0.12]
        point[2] [0.1 0.12]
        point[3] [0.12 0.1]
        point[4] [0.1 0.08]
        point[5] [0.07 0.08]
        z [0 1]
    )
)