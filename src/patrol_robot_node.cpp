#include <patrol_robot/patrol_robot.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "patrol_robot_node");
    tf::TransformListener tf(ros::Duration(10));
    patrol_robot::PatrolRobot patrol_robot( tf );

    ros::spin();
    
    return 0;
}