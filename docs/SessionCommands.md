### Start ROS
roscore

naosi
cd local_webpage
python3 -m http.server 8111

naosi
rosrun error_recovery_si_nao localwebpage.py

naosi
python3 webcam_check/webcam_check.py

naosi
rosrun error_recovery_si_nao detemine_action.py 

naosi
rosrun error_recovery_si_nao nao_action.py

naosi
rosrun error_recovery_si_nao websocket.py

roslaunch audio_capture capture.launch

naosi
cd .. (get back up to src)
roslaunch Azure_Kinect_ROS_Driver-melodic/launch/driver_with_bodytracking.launch
# check for K4A Started

Paste from Participants URL in a new terminal window
rosbag record /audio/audio /audio/audio_info /body_tracking_data /rgb/camera_info /rgb/image_raw/compressed /space_invaders/game/game_condition /space_invaders/game/game_mode /space_invaders/game/nao_action /space_invaders/game/robot_action /tf /tf_static -O participantbags/PX.bag

rosbag info participantbags/PXbag


For troubleshooting: rostopic pub -1 /space_invaders/game/robot_action std_msgs/String wake