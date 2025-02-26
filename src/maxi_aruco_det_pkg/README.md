# aruco二维码功能包

环境需要opencv3+opencv_contrib3  

## 使用方法

相机内参和畸变参数在config/camera.yaml里填写  
aruco_det.launch里设置aruco字典，要识别的aruco二维码id，aruco二维码的边长(单位米)，订阅的图像话题名称。  
```
roslaunch aruco_det aruco_det.launch
```

aruco二维码检测得到的平移向量和旋转向量对应的四元数存放在了geometry_msgs::PoseStamped类型的/aruco/pose话题里发布出来，可以订阅使用。  
注意aruco二维码检测得到的位姿是相机系下的aruco坐标系的旋转平移。  

aruco_land_demo.cpp是可以配合使用的aruco二维码降落逻辑节点，启动命令如下，可以订阅aruco二维码检测结果话题/aruco/pose话题进行aruco二维码降落，其降落逻辑对应aruco_landing_pad.world中的降落场景，具体在此视频中有详细讲解 https://www.bilibili.com/video/BV1VE421T7wJ/?spm_id_from=333.999.0.0   
程序里默认是自动解锁切offboard，如果是实飞注意关闭自动解锁切offboard   
```
rosrun aruco_det aruco_land_demo
```

## aruco detection without ros

need opencv3.3.1 and opencv_contrib3.3.1

compile command
```
g++ -o aruco_detection aruco_detection.cpp `pkg-config --cflags --libs opencv`

```
usage
```
./aruco_detection
```


