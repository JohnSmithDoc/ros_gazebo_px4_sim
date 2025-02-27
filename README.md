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

## Demo测试

<ol>
  <li>简单的offboard控制测试</li>  

  首先利用官方提供的代码，建立了offb_node节点，实现控制无人机进入offboard并起飞到2m然后一直保持在2m的高度，官方历程请 ref:https://docs.px4.io/v1.13/en/ros/mavros_offboard_cpp.html
  注意：运行之前需要source一下（如果将source加入了bashrc.sh中就不用每次都source）

  运行此测试节点的命令如下：
  > rosrun ros_ctrl_px4_test offb_node

  <li>使用gazebo仿真二维码降落</li>  
  这个Demo是在这个开源项目上进行了小小的修改而来的，项目链接：https://gitee.com/maxibooksiyi/maxi_aruco_det_pkg
  关于这个的讲解作者也专门出了两期视频，很有用，视频连接：https://blog.csdn.net/sinat_16643223/article/details/138092922?spm=1001.2014.3001.5501

  此Demo说明如下：
  <ol>
  <li>使用的是PX4 V1.11.3版本，自己独立维护了一个分支，分支为：https://github.com/JohnSmithDoc/PX4-Autopilot/tree/master_v1_11_3</li> 

  <li>同时PX4的Gazebo子模块也独立维护了一个分支，分支为：https://github.com/JohnSmithDoc/gazebo_v1_11_3/tree/gazebo_for_master_v1_11_3</li> 

  <li>在PX4编译时尽量不要用到 make distclean 命令，不然你需要重新拉取一下gazebo的仓库</li> 

  <li>我使用的opencv 版本是4.2.0，也可以正常使用</li> 

  <li>ubuntu 版本是20.04，ros版本是neotic</li> 

  <li>可以直接使用maxi_aruco_det_pkg/scripts中的start_aruco_land_all_in_one.sh一次性拉起所有节点，不过第一次使用还是建议先自己一个一个拉起节点测试，因为第一次gazebo的加载可能会花比较长的时间</li> 

  </ol>

</ol>

## TODO
后续需要将mavlink和mavros这两个仓库作为ros_gazebo_px4_sim_ws的子模块来自己管理起来，不然有些麻烦


