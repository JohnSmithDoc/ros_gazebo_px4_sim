# ros_gazebo_px4_sim_ws
使用ros和px4已经gazebo进行联合仿真的项目文件,工作空间

使用的PX4的代码分支为1.11.3稳定版本，在fork的仓库的分支为：master_v1_11_3

## 使用步骤（临时）

<ol>
  <li>首先运行 “setup_px4_ros.sh“ 这个脚本，这个脚本会拉起px4的SITL以及gazebo并加载相应模型</li>
  <li>拉起mavros，对于单架无人机软仿，使用如下指令拉起mavros节点，以方便和px4通信：</li>    

  > roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"

  <li>最后再拉起我们自己编写的ros节点，来控制无人机。正常来说，我们自己的ros节点是通过mavros节点间接控制无人机的，即我们发布话题/服务请求/Action给mavros，然后mavros再转换，通过mavlink发送给飞控</li>    
</ol>

## TODO
后续需要将mavlink和mavros这两个仓库作为ros_gazebo_px4_sim_ws的子模块来自己管理起来，不然有些麻烦


