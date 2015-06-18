export ROS_ROOT=/opt/ros/indigo/share/ros
export ROS_PACKAGE_PATH=/home/dash/talos_indigo_workspace/src:/opt/ros/indigo/share:/opt/ros/indigo/stacks

export ROBOT=`cat /etc/ros/robot`
export ROS_MASTER_URI=`cat /etc/ros/master`
export PATH=$ROS_ROOT/bin:$PATH
export PYTHONPATH=$PYTHONPATH:$ROS_ROOT/core/roslib/src
source $ROS_ROOT/tools/rosbash/rosbash
