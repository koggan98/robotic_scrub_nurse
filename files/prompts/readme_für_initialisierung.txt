pip install
	tabulate
	mediapipe
	pyrealsense2
	
realsense sdk installieren!	

ws sourcen in .bashrc

package clonen in ws/src
das andere package auch clonen in ws/src
in ws colcon build

in ~/$WORKSPACE$/src/ur3e_hand_tracking_package/src/tracking_pkg$:
	chmod +x src/publisher/camera_publisher.py
	chmod +x src/publisher/frame_publisher.py
	chmod +x src/publisher/hand_pose_visualization_publisher.py
	chmod +x src/publisher/box_publisher.py
	chmod +x src/hand_tracker/hand_tracker.py
	chmod +x src/socket_mover/socket_mover.py
	chmod +x src/moveit_mover/gripper_mover.py

place aruco board -40cm away from robot base center | oder pass in code an in datei frame_publisher
aruco board consists of 4 markers placed 20cm away from each other at 4 corners | oder pass in code an in datei frame_publisher
all frame axis have to be colinear 

to use moveit connect via manual control

bau kamera auf. realsense software muss installiert sein.

