### Start ROS
roscore

python3 -m http.server 8111

rosrun error_recovery_si_nao localwebpage.py

python3 webcam_check/webcam_check.py

rosrun error_recovery_si_nao detemine_action.py 

rosrun error_recovery_si_nao nao_action.py

rostopic pub -1 /space_invaders/game/robot_action std_msgs/String wake

rosrun error_recovery_si_nao websocket.py

roslaunch audio_capture capture.launch

roslaunch Azure_Kinect_ROS_Driver-melodic/launch/driver_with_bodytracking.launch

