#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include "../include/my_offboard_node/RectangularTrajectory.h"

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

// 当前本地高度（用于判断降落是否完成）
geometry_msgs::PoseStamped current_local_pos;
void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_local_pos = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rect_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, local_pos_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    // setpoint 发布频率必须高于 2Hz
    ros::Rate rate(20.0);

    // 等待与飞控建立连接
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    // 矩形轨迹：宽4m、长6m，起点(0,0)，高度2m，速度1m/s
    RectangularTrajectory trajectory(4.0, 6.0, 0.0, 0.0, 2.0, 1.0);

    geometry_msgs::PoseStamped pose;

    // 起飞前先连续发送初始位置，让 OFFBOARD 模式可以切换
    for(int i = 100; ros::ok() && i > 0; --i){
        pose = trajectory.calculate_pose(0);
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    ros::Time traj_start;
    bool traj_started = false;   // 轨迹计时是否已开始
    bool landing = false;        // 是否已进入降落阶段

    while(ros::ok()){
        // ---------- 1. 切换 OFFBOARD 模式 ----------
        if(current_state.mode != "OFFBOARD" &&
           (ros::Time::now() - last_request > ros::Duration(5.0))){
            if(set_mode_client.call(offb_set_mode) &&
               offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        }
        // ---------- 2. 解锁（Arm）----------
        else {
            if(!current_state.armed &&
               (ros::Time::now() - last_request > ros::Duration(5.0))){
                if(arming_client.call(arm_cmd) &&
                   arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        // ---------- 3. 飞矩形轨迹 ----------
        if(!landing){
            // 飞控进入 OFFBOARD 且已解锁后开始计时
            if(current_state.mode == "OFFBOARD" && current_state.armed){
                if(!traj_started){
                    traj_start = ros::Time::now();
                    traj_started = true;
                    ROS_INFO("Rectangular trajectory started.");
                }

                double elapsed = (ros::Time::now() - traj_start).toSec();
                pose = trajectory.calculate_pose(elapsed);
                local_pos_pub.publish(pose);

                // 飞完一圈，切换到自动降落
                if(trajectory.is_finished(elapsed)){
                    ROS_INFO("Trajectory complete. Switching to AUTO.LAND...");
                    mavros_msgs::SetMode land_set_mode;
                    land_set_mode.request.custom_mode = "AUTO.LAND";
                    if(set_mode_client.call(land_set_mode) &&
                       land_set_mode.response.mode_sent){
                        ROS_INFO("AUTO.LAND enabled");
                        landing = true;
                    }
                }
            } else {
                // 还未进入 OFFBOARD/解锁，持续发送初始悬停点
                pose = trajectory.calculate_pose(0);
                local_pos_pub.publish(pose);
            }
        }
        // ---------- 4. 等待降落完成 ----------
        else {
            // 高度低于 0.15m 视为已落地，加锁并退出
            if(current_local_pos.pose.position.z < 0.15){
                ROS_INFO("Landed. Disarming...");
                mavros_msgs::CommandBool disarm_cmd;
                disarm_cmd.request.value = false;
                if(arming_client.call(disarm_cmd) && disarm_cmd.response.success){
                    ROS_INFO("Vehicle disarmed. Mission complete.");
                }
                break;
            }
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
