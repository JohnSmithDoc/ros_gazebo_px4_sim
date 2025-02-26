/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * STcak and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>


using namespace Eigen;
using namespace std;

mavros_msgs::State current_state;

//Twb的意思是body系在world系下的位姿，也是body系到world系的变换矩阵，Twb = Tb->w
Eigen::Isometry3d Twa,Tca,Tbc,Twb,Twc;  //变换矩阵
//Isometry3d Twa=Eigen::Isometry3d::Identity();
//Isometry3d Tca=Eigen::Isometry3d::Identity();
//Isometry3d Tbc=Eigen::Isometry3d::Identity();
//Isometry3d Twb=Eigen::Isometry3d::Identity();
//Isometry3d Twc=Eigen::Isometry3d::Identity();
//Tbc(0,0) =            0, Tbc(0,1) =           -1, Tbc(0,2) =            0, Tbc(0,3) =             0;
//Tbc(1,0) =           -1, Tbc(1,1) =            0, Tbc(1,2) =            0, Tbc(1,3) =             0;
//Tbc(2,0) =            0, Tbc(2,1) =            0, Tbc(2,2) =           -1, Tbc(2,3) =             0;
//Tbc(3,0) =            0, Tbc(3,1) =            0, Tbc(3,2) =            0, Tbc(3,3) =             1;


//Eigen::Matrix3d rotation_matrix_bc = Eigen::Matrix3d::Identity();
Eigen::Matrix3d rotation_matrix_bc,rotation_matrix_wb,rotation_matrix_ca;;

Eigen::Vector3d t_bc,t_wb,t_ca;

Quaterniond q_wb,q_ca;

//double drone_position_ENU[3]={0,0,0}; //无人机ENU世界系下位置

//double aruco_position_cam[3]={0,0,0}; //aruco二维码检测得到的位置也就是相机系下的位置

//double aruco_position_ENU[3]={0,0,0}; //aruco二维码在东北天世界系下的位置

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
        {
            //drone_position_ENU[0] = msg->pose.position.x;
            //drone_position_ENU[1] = msg->pose.position.y;
            //drone_position_ENU[2] = msg->pose.position.z;

            t_wb << msg->pose.position.x ,msg->pose.position.y ,msg->pose.position.z;
            //在Eigen库中,四元数的存储顺序是w、x、y、z
            q_wb = Eigen::Quaterniond(msg->pose.orientation.w ,msg->pose.orientation.x ,msg->pose.orientation.y ,msg->pose.orientation.z);
            rotation_matrix_wb = q_wb.toRotationMatrix();
            //Twb.rotate( rotation_matrix_wb );
            //Twb.pretranslate( t_wb );
            Twb.linear() = rotation_matrix_wb;
            Twb.translation() = t_wb;
        }

void aruco_pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
        {
            //aruco_position_cam[0] = msg->pose.position.x;
            //aruco_position_cam[1] = msg->pose.position.y;
            //aruco_position_cam[2] = msg->pose.position.z;

            t_ca << msg->pose.position.x ,msg->pose.position.y ,msg->pose.position.z;

            q_ca = Eigen::Quaterniond(msg->pose.orientation.w ,msg->pose.orientation.x ,msg->pose.orientation.y ,msg->pose.orientation.z);
            rotation_matrix_ca = q_ca.toRotationMatrix();
            //Tca.rotate( rotation_matrix_ca );
            //Tca.pretranslate( t_ca );
            Tca.linear() = rotation_matrix_ca;
            Tca.translation() = t_ca;
        }



int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh1("~");

    bool AUTO_ARM_OFFBOARD;

    nh1.param<bool>("auto_arm_offboard", AUTO_ARM_OFFBOARD, true);

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, pos_cb);  

    ros::Subscriber aruco_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/aruco/pose", 10, aruco_pos_cb);

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::Publisher setpoint_raw_local_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    t_bc <<  0, 0, 0;
    rotation_matrix_bc << 0, -1, 0,
                         -1 , 0, 0,
                          0, 0, -1;

    Tbc.linear() = rotation_matrix_bc;
    Tbc.translation() = t_bc;
    t_ca << 0, 0, 0;

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::PositionTarget pos_setpoint;  //创建一个mavros_msgs::PositionTarget类型的实例化对象
    pos_setpoint.type_mask = 0b100111111000;  // 100 111 111 000  xyz + yaw
    pos_setpoint.coordinate_frame = 1;
    pos_setpoint.position.x = 0;
    pos_setpoint.position.y = 0;
    pos_setpoint.position.z = 2;
    pos_setpoint.yaw = 0;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        setpoint_raw_local_pub.publish(pos_setpoint);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::SetMode land_set_mode;
    land_set_mode.request.custom_mode = "AUTO.LAND";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    int offboard_flag = 0;//防止切land降落之后又去解锁切offboard
    int land_flag = 0;
    int state_flag = 0;
    ros::Time time_snap;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        if(AUTO_ARM_OFFBOARD==true)
        {
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))&&(offboard_flag==0)){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))&&(offboard_flag==0)){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();//可以通过这个方式保持当前指点持续几秒钟啊。
            }
        }
        }
        if(AUTO_ARM_OFFBOARD==false)
        {
        }

        //ros::Duration(1.0).sleep();
        if((abs(t_wb[0])<0.2&&abs(t_wb[1])<0.2&&abs(t_wb[2]-2)<0.2)&&(ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            state_flag = 1;
        }

        if( state_flag == 1)
        {
         std::cout << "state 1" << std::endl;
         pos_setpoint.position.x = 1.5;
         pos_setpoint.position.y = 1.5;
         pos_setpoint.position.z = 2; 
  
         if((abs(t_wb[0]-1.5)<0.2&&(abs(t_wb[1]-1.5)<0.2)&&abs(t_wb[2]-2)<0.2)&&(t_ca[2] != 0))
         {
            state_flag = 2;
         }
        }

        if( state_flag == 2)
        {
         std::cout << "state 2" << std::endl;
         //aruco_position_ENU[0] = drone_position_ENU[0] - aruco_position_cam[1];
         //aruco_position_ENU[1] = drone_position_ENU[1] - aruco_position_cam[0];
         //aruco_position_ENU[2] = drone_position_ENU[2] - aruco_position_cam[2];

         //Twc = Twb * Tbc ;
         //Twa =Twc * Tca ;
         //Twa = Tca * Tbc * Twb; //注意这样是错的
         Twa = Twb * Tbc * Tca ;

         /***
         std::cout << "t_wb" << t_wb  << std::endl;
         std::cout << "Twb" << Twb.matrix() << std::endl;
         std::cout << "Tbc" << Tbc.matrix() << std::endl;
         std::cout << "t_ca" << t_wb  << std::endl;
         std::cout << "Tca" << Tca.matrix() << std::endl;
         std::cout << "Twc" << Twc.matrix() << std::endl;
         std::cout << "Twa" << Twa.matrix() << std::endl;
         ***/
         // 获取T中的平移向量
         Vector3d t_wa = Twa.translation();

         pos_setpoint.position.x = t_wa[0];
         pos_setpoint.position.y = t_wa[1];
         pos_setpoint.position.z = t_wa[2] + 2; 

         if((abs(t_ca[0])<0.1)&&(abs(t_ca[1])<0.1))
         {
            state_flag = 3;
            time_snap = ros::Time::now();
         }
        }

        if( state_flag == 3)
        {
         std::cout << "state 3" << std::endl;

         ros::Time now_time = ros::Time::now();
         int now_time_sec = now_time.sec;
         int now_time_nsec = now_time.nsec;
         double now_time_sec_sum = now_time_sec + now_time_nsec*(1e-9);

         int time_snap_sec = time_snap.sec;
         int time_snap_nsec = time_snap.nsec;
         double time_snap_sec_sum = time_snap_sec + time_snap_nsec*(1e-9);
         double now_time_relate_from_state2 = now_time_sec_sum - time_snap_sec_sum;


         //aruco_position_ENU[0] = drone_position_ENU[0] - aruco_position_cam[1];
         //aruco_position_ENU[1] = drone_position_ENU[1] - aruco_position_cam[0];
         //aruco_position_ENU[2] = drone_position_ENU[2] - aruco_position_cam[2];

         /***
         pos_setpoint.type_mask = 0b100111000100;
         pos_setpoint.coordinate_frame = 1;
         pos_setpoint.velocity.x = 0;
         pos_setpoint.velocity.y = 0;
         pos_setpoint.velocity.z = -0.1;
         pos_setpoint.position.x = aruco_position_ENU[0];
         pos_setpoint.position.y = aruco_position_ENU[1];
         pos_setpoint.yaw = 0;
         ***/

         //Twc = Twb * Tbc ;
         Twa = Twb * Tbc * Tca ;
         // 获取T中的平移向量
         Vector3d t_wa = Twa.translation();

         pos_setpoint.position.x = t_wa[0];
         pos_setpoint.position.y = t_wa[1];
         pos_setpoint.position.z = t_wa[2] + 2 - 0.1*now_time_relate_from_state2; 

         if(t_wb[2] < 0.35)
         {
            state_flag = 4;
         }
        }


         if( state_flag == 4)
         {
         std::cout << "state 4" << std::endl;
         //aruco_position_ENU[0] = drone_position_ENU[0] - aruco_position_cam[1];
         //aruco_position_ENU[1] = drone_position_ENU[1] - aruco_position_cam[0];
         //aruco_position_ENU[2] = drone_position_ENU[2] - aruco_position_cam[2];

         //Twc = Twb * Tbc ;
         //Twa =Twc * Tca ;
         Twa = Twb * Tbc * Tca ;
         // 获取T中的平移向量
         Vector3d t_wa = Twa.translation();

         pos_setpoint.type_mask = 0b100111111000;  // 100 111 111 000  xyz + yaw
         pos_setpoint.coordinate_frame = 1;

         pos_setpoint.position.x = t_wa[0];
         pos_setpoint.position.y = t_wa[1];
         pos_setpoint.position.z = t_wa[2] + 0.3; 
         pos_setpoint.yaw = 0;
        
         if((abs(t_ca[0])<0.1)&&(abs(t_ca[1])<0.1))
         {
            state_flag = 5;
         } 
        }

         if( state_flag == 5)
         {
         if( current_state.mode != "AUTO.LAND" )
         {
         set_mode_client.call(land_set_mode);
         if(land_set_mode.response.mode_sent)
           {
             ROS_INFO("land enabled");
             offboard_flag = 1;    
           }  
          }
         }

        setpoint_raw_local_pub.publish(pos_setpoint);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}



