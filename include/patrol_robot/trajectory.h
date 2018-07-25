#ifndef PATROL_ROBOT_TRAJECTORY_H
#define PATROL_ROBOT_TRAJECTORY_H

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <vector>
#include <tf/tf.h>

namespace patrol_robot{
class Trajectory{
    public:
        Trajectory();
        ~Trajectory();
        bool makePlan(geometry_msgs::Pose current_pose, std::vector<geometry_msgs::Pose> g, std::vector< std::vector<geometry_msgs::Pose> >& traj, double s);
        bool getPositionAt(int index1, int index2, geometry_msgs::Pose& pose);//index1：第几条路径，index2:第几个点
        double calDistance(double x, double y);
        geometry_msgs::Quaternion getOrientation(geometry_msgs::Pose p1, geometry_msgs::Pose p2);
        std::vector<geometry_msgs::Pose> createTrajectory(geometry_msgs::Pose begin, geometry_msgs::Pose end, geometry_msgs::Pose end_n);
        geometry_msgs::Vector3 quaternionToEuler(geometry_msgs::Quaternion q);
        geometry_msgs::Quaternion eulerToQuaterion(geometry_msgs::Vector3 v);
    private:
        std::vector< std::vector<geometry_msgs::Pose> > trajectory;
        std::vector<geometry_msgs::Pose> goals;
        double step_dis;
};
};

#endif