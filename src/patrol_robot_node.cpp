#include <patrol_robot/patrol_robot.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "patrol_robot_node");
    patrol_robot::PatrolRobot patrol_robot();

    ros::spin();
    
    return 0;
}