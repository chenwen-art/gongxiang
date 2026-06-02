###gmapping with abot###
gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 3; source ~/demo/devel/setup.bash; roslaunch abot_bringup robot_with_imu.launch; exec bash"' \
--tab -e 'bash -c "sleep 4; source ~/demo/devel/setup.bash; roslaunch robot_slam navigation.launch; exec bash"' \
--tab -e 'bash -c "sleep 4; source ~/demo/devel/setup.bash; roslaunch track_tag usb_cam_with_calibration.launch; exec bash"' \
--tab -e 'bash -c "sleep 4; source ~/demo/devel/setup.bash; roslaunch track_tag ar_track_camera.launch; exec bash"' \
--tab -e 'bash -c "sleep 4; source ~/demo/devel/setup.bash; roslaunch abot_vlm vlm_node.launch; exec bash"' \
--tab -e 'bash -c "sleep 4; source ~/demo/devel/setup.bash; roslaunch robot_slam multi_goal.launch; exec bash"' \
--tab -e 'bash -c "sleep 4; source ~/demo/devel/setup.bash; roslaunch robot_slam view_nav.launch; exec bash"' \
--tab -e 'bash -c "sleep 4; source ~/demo/devel/setup.bash; rosrun TTS_audio TTS.py; exec bash"' \
--tab -e 'bash -c "sleep 4; source ~/demo/devel/setup.bash; roslaunch robot_slam GameStart.launch; exec bash"' \
#--tab -e 'bash -c "sleep 4; source ~/anaconda3/etc/profile.d/conda.sh; conda activate audio; roslaunch TTS_audio tts.launch; exec bash"'
### --tab -e 'bash -c "sleep 4; source ~/new_vision/devel/setup.bash; roslaunch find_object_2d find_object_2d.launch; exec bash"' \
