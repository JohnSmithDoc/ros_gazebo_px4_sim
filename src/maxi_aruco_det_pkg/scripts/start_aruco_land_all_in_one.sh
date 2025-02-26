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
# # roslaunch px4 posix_sitl.launch
# roslaunch maxi_aruco_det_pkg mavros_posix_sitl_aruco.launch

# 以下为新加的代码，依次开启新终端并启动相应节点
ARUCO_DEMO_DIR="$HOME/ros_gazebo_px4_sim_ws/"
cd $ARUCO_DEMO_DIR


# Function to open a new terminal and run a command
open_terminal_and_run() {
    local cmd=$1
    gnome-terminal -- bash -c "$cmd; exec bash"
}

# Terminal commands without sourcing
cmd1="roslaunch maxi_aruco_det_pkg mavros_posix_sitl_aruco.launch"
cmd2="roslaunch maxi_aruco_det_pkg aruco_det.launch"
cmd3="rostopic echo /aruco/pose"
cmd4="rqt_image_view"
cmd5="roslaunch maxi_aruco_det_pkg aruco_det_for_gazebo.launch"

# Open new terminals for each command
open_terminal_and_run "$cmd1"
sleep 10 # 等待10秒
open_terminal_and_run "$cmd2"
sleep 10 # 等待10秒
open_terminal_and_run "$cmd3"
sleep 1
open_terminal_and_run "$cmd4"
sleep 1
open_terminal_and_run "$cmd5"

# Additional commands to be executed in the current terminal




