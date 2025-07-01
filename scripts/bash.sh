
################# Aliases #################
alias gd='gedit ~/.bashrc'
alias gs='gedit ~/shared_volume/bash.sh'
alias src='source ~/.bashrc'
alias zenoh='ros2 run rmw_zenoh_cpp rmw_zenohd'
alias sss='source install/setup.bash'
alias qgc='cd ~/shared_volume && ./QGroundControl.AppImage'
alias px4='cd ~/shared_volume/PX4-Autopilot && make px4_sitl gz_x500_twin_stereo_twin_velodyne'
alias px4_tug='cd ~/shared_volume/PX4-Autopilot && PX4_GZ_MODEL_POSE="0,0,0.1,0,0,0" make px4_sitl gz_x500_stereo_cam_3d_lidar PX4_GZ_WORLD=tugbot_depot' 

################# Build #################
alias cbuav='cd ~/shared_volume/ros2_ws && colcon build --packages-select uav_gz_sim'

################# ROS #################
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
# export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0

################# Launch the simulation #######################
alias tug='ros2 launch uav_gz_sim sim.launch.py world_type:=tugbot_depot'

################# Github Repos #################

export GIT_USER=
export GIT_TOKEN=