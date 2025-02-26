FIRMWARE_DIR="$HOME/PX4-Autopilot/"

# 切换到PX4固件目录
cd $FIRMWARE_DIR

# 构建PX4和Gazebo仿真环境
# make px4_sitl_default gazebo

# 设置Gazebo环境变量
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default

# 更新ROS_PACKAGE_PATH环境变量
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo

# 添加gazebo模型的环境变量，以便gazebo能够找到我们自定义的模型位置
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:$(pwd)/Tools/sitl_gazebo/models

# 启动PX4 SITL仿真
# roslaunch px4 posix_sitl.launch

roslaunch maxi_aruco_det_pkg mavros_posix_sitl_aruco.launch



