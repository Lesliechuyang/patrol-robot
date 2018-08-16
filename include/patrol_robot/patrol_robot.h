#ifndef PATROL_ROBOT_H
#define PATROL_ROBOT_H

#include <patrol_robot/controller.h>
#include <patrol_robot/trajectory.h>
#include <ros/ros.h>
#include <patrol_robot/SendCommands.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>

namespace patrol_robot{
class PatrolRobot{
    public:
        PatrolRobot(tf::TransformListener& tf);
        ~PatrolRobot();
        bool receiveGoalsService(patrol_robot::SendCommands::Request  &req, patrol_robot::SendCommands::Response &res);
        void odomCB(const nav_msgs::Odometry::ConstPtr &odom);
        geometry_msgs::Pose getCurrentPose();//获取当前位子
    private:
        tf::TransformListener& tf_;
        Trajectory* trajectory_planner;//路径规划器
        ros::ServiceServer receive_goals;//接受目标点服务
        ros::Publisher vel_pub;//速度发布
        ros::Subscriber odom_sub;//里程计接收
        geometry_msgs::Pose current_pose;//当前姿态
        geometry_msgs::Twist current_twist;//当前速度
        std::vector< std::vector<geometry_msgs::Pose> > trajectory;//储存规划好的路径点
        double vel_default;//默认前进速度，用于计算步长，默认0.1m/s
        double controller_frequency;//控制频率,默认10hz
        double step_dis;//步长
        double controller_patience_pose, controller_patience_theta;//位置和角度偏移容忍度
        double Kx, Ky, Ktheta;//控制器参数
        Controller* controller;//控制器对象
        int timeout;//等待超时设置
};
};

#endif