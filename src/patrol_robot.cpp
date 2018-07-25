#include <patrol_robot/patrol_robot.h>
#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>

namespace patrol_robot{
    PatrolRobot::PatrolRobot():
    trajectory_planner(NULL),
    step_dis(0.01)
    {
        ros::NodeHandle private_nh("~");
        ros::NodeHandle nh;

        trajectory_planner = new Trajectory();//路径规划器对象
        vel_pub = private_nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        odom_sub = private_nh.subscribe<nav_msgs::Odometry>("odom", 1, boost::bind(&PatrolRobot::odomCB, this, _1));
        //用于接受服务消息
        receive_goals = private_nh.advertiseService("abc", &PatrolRobot::receiveGoalsService, this);

    }

    PatrolRobot::~PatrolRobot(){
        
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