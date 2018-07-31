#include <patrol_robot/patrol_robot.h>
#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>

namespace patrol_robot{
    PatrolRobot::PatrolRobot(tf::TransformListener& tf):
    tf_(tf),
    trajectory_planner(NULL),
    controller_frequency(10),
    vel_default(0.1),
    controller_patience_pose(0.05),
    controller_patience_theta(0.15),
    Kx(0.9), Ky(0.9), Ktheta(0.9),
    controller(NULL)
    {
        ros::NodeHandle nh;
        trajectory_planner = new Trajectory();//路径规划器对象
        controller = new Controller(trajectory_planner, controller_frequency, controller_patience_pose, controller_patience_theta, Kx, Ky, Ktheta);//控制器对象
        vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        odom_sub = nh.subscribe<nav_msgs::Odometry>("odom", 1, boost::bind(&PatrolRobot::odomCB, this, _1));
        //用于接受服务消息
        receive_goals = nh.advertiseService("abc", &PatrolRobot::receiveGoalsService, this);

        step_dis = vel_default / controller_frequency;//计算步长，默认速度/控制频率
    }

    PatrolRobot::~PatrolRobot(){
        delete trajectory_planner;
        delete controller;
    }

    bool PatrolRobot::receiveGoalsService(patrol_robot::SendGoals::Request  &req,
         patrol_robot::SendGoals::Response &res){

        int success = 0;
        unsigned int num = req.goals.poses.size();
        std::vector<geometry_msgs::Pose> goals_nav;
        goals_nav.resize(num);
        for(unsigned int i = 0; i < num; ++i){
            goals_nav.push_back(req.goals.poses[i]);
            ROS_INFO("Reveive goals");  
        }

        //规划路径
        if(!trajectory_planner->makePlan(current_pose, goals_nav, trajectory, step_dis))
            ROS_INFO("Trajectory planner failed to plan!\n");

        //控制器
        for(int i = 0; i < num; ++i){
            int point_index = trajectory_planner->getSteps(i);//获取第i条路径的点数
            int j = 0;
            geometry_msgs::Twist twist;//存放规划的速度
            ros::Rate r(controller_frequency);//控制频率
            //当还没到达位置目标点，循环执行
            while(!trajectory_planner->goalReached(i, current_pose, controller_patience_pose)){
                if(controller->computePositionControlActions(current_pose, twist, i, j)){
                    vel_pub.publish(twist);
                    j += 1;
                }
                else{
                    ROS_INFO("Position control failed!\n");
                }
                r.sleep();
            }
            //角度控制
            double angle_diff;//角度差
            while(!trajectory_planner->OrientationReached(i, current_pose, controller_patience_theta, angle_diff)){
                if(controller->computeOrientationControlActions(twist, angle_diff)){
                    vel_pub.publish(twist);
                }
                else{
                    ROS_INFO("Orientation control failed!\n");
                }
                r.sleep();
            }
            success++;//执行完一次累加
        }

        if(success == num)  
        {
            ROS_INFO("Hooray, service successful!");  
            res.success = true;
            ROS_INFO("sending back response");
            return true;
        }
        else
        {
            res.success = false;
            ROS_INFO("%d points success, %d points failed!", success, num - success);  
            return true;
        }

        return false;
    }

    void PatrolRobot::odomCB(const nav_msgs::Odometry::ConstPtr &odom){
        current_pose = odom->pose.pose;
        current_twist = odom->twist.twist;
    }
};