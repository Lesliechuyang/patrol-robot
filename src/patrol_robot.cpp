#include <patrol_robot/patrol_robot.h>
#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>
#include <unistd.h>
#include <patrol_robot/ElevatorAndCamera.h>


namespace patrol_robot{
    PatrolRobot::PatrolRobot(tf::TransformListener& tf):
    tf_(tf),
    trajectory_planner(NULL),
    controller_frequency(10),
    vel_default(0.1),
    controller_patience_pose(0.05),
    controller_patience_theta(0.05),
    Kx(0.9), Ky(0.9), Ktheta(0.9),
    controller(NULL),
    timeout(300)  //50个循环，每个循环0.1s，5s等待
    {
        ros::NodeHandle nh;
        trajectory_planner = new Trajectory();//路径规划器对象
        controller = new Controller(trajectory_planner, controller_frequency, controller_patience_pose, controller_patience_theta, Kx, Ky, Ktheta);//控制器对象
        vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        odom_sub = nh.subscribe<nav_msgs::Odometry>("odom", 1, boost::bind(&PatrolRobot::odomCB, this, _1));
        //用于接受服务消息
        receive_goals = nh.advertiseService("send_goals", &PatrolRobot::receiveGoalsService, this);
        //初始化初始位置
        current_pose.position.x = 0;
        current_pose.position.y = 0;
        current_pose.orientation.x = 0;
        current_pose.orientation.y = 0;
        current_pose.orientation.z = 0;
        current_pose.orientation.w = 1;
        step_dis = vel_default / controller_frequency;//计算步长，默认速度/控制频率
        printf("hello\n");
    }

    PatrolRobot::~PatrolRobot(){
        delete trajectory_planner;
        delete controller;
    }

    bool PatrolRobot::receiveGoalsService(patrol_robot::SendGoals::Request  &req,
         patrol_robot::SendGoals::Response &res){

        unsigned int num = req.goals.poses.size();
        std::vector<geometry_msgs::Pose> goals_nav;
        for(unsigned int i = 0; i < num; ++i){
            goals_nav.push_back(req.goals.poses[i]);
            ROS_INFO("Receive goals");  
        }
        printf("The number of goals is %d\n", num);
        //规划路径
        if(!trajectory_planner->makePlan(current_pose, goals_nav, trajectory, step_dis)){
            ROS_INFO("Trajectory planner failed to plan!\n");
            return false;
        }
            
        int defeat = 0;//失败次数
        //控制器
        for(int i = 0; i < num; ++i){
            int point_num = trajectory_planner->getSteps(i);//获取第i条路径的路径点数
            printf("%dth control loop, full loops are %d\n", i, num);
            int j = 0;//当前路径的第j次循环
            geometry_msgs::Twist twist;//存放规划的速度
            geometry_msgs::Pose c_pose; //当前位资
            ros::Rate r(controller_frequency);//控制频率
            printf("begin position control!\n");
            //当还没到达位置目标点，循环执行
            do{
                ros::spinOnce();//spinonce去获取odom的回调函数，否则current pose不会变化
                c_pose = getCurrentPose();
                if(controller->computePositionControlActions(c_pose, twist, i, j)){
                    //printf("current_pose %f, %f, %f, %f\n", current_pose.position.x, current_pose.position.y, current_pose.orientation.z, current_pose.orientation.w); 
                    //printf("c_pose %f, %f, %f, %f\n", c_pose.position.x, c_pose.position.y, c_pose.orientation.z, c_pose.orientation.w);                                                           
                    vel_pub.publish(twist);
                    j += 1;
                    //printf("j = %d\n", j);
                }
                else{
                    ROS_INFO("Position control failed!\n");
                }
                //检查等待超时
                if(j > point_num + timeout){
                    defeat++;
                    printf("position control failed\n");
                    break;//退出当前while循环
                }else{
                    r.sleep();
                }
            }while(!trajectory_planner->goalReached(i, current_pose, controller_patience_pose));
            
            ///////////////////////////
            if(trajectory_planner->goalReached(i, current_pose, controller_patience_pose)){
                printf("position control successfully, begin orientation control!\n");
                //角度控制
                double angle_diff;//角度差
                int run_time = 0;//运行次数
                do{
                    ros::spinOnce();
                    c_pose = getCurrentPose();
                    if(controller->computeOrientationControlActions(twist, angle_diff)){
                        vel_pub.publish(twist);
                        printf("v = %f, w = %f\n", twist.linear.x, twist.angular.z);
                    }
                    else{
                        ROS_INFO("Orientation control failed!\n");
                    }
                    //检查等待超时
                    if(run_time > timeout){
                        defeat++;
                        printf("orientation control failed\n");
                        printf("the %dth trajectory can't reached\n", i+1);
                        break;//退出当前while循环
                    }else{
                        run_time++;
                        r.sleep();
                    }
                }while(!trajectory_planner->OrientationReached(i, current_pose, controller_patience_theta, angle_diff));
            }
            else{
                printf("the %dth trajectory can't reached\n", i+1);
            }

            sleep(1);
            ElevatorTest();
            sleep(1);
            CameraTest("5");
            sleep(5);
        }

        if(!defeat)  
        {
            ROS_INFO("Hooray, service successful!");  
            res.success = true;
            ROS_INFO("sending back response");
            return true;
        }
        else
        {
            res.success = false;
            ROS_INFO("%d points success, %d points failed!", num - defeat, defeat);  
            return true;
        }

        return false;
    }

    void PatrolRobot::odomCB(const nav_msgs::Odometry::ConstPtr &odom){
        current_pose = odom->pose.pose;
        //printf("%f, %f, %f, %f\n", current_pose.position.x, current_pose.position.y, current_pose.orientation.z, current_pose.orientation.w);

        current_twist = odom->twist.twist;
    }

    geometry_msgs::Pose PatrolRobot::getCurrentPose(){
        return current_pose;
    }
};

// int main(int argc, char** argv){
//     ros::init(argc, argv, "patrol_robot_node");
//     tf::TransformListener tf(ros::Duration(10));
//     patrol_robot::PatrolRobot patrol_robot( tf );

//     ros::spin( );
    
//     return 0;
// }