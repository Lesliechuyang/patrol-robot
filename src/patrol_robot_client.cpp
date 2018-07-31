#include "ros/ros.h"
#include "patrol_robot/SendGoals.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "patrol_robot_client");
  //if (argc != 4)
  //{
  //  ROS_INFO("usage: add three parameters X Y W");
  //  return 1;
  //}

  ros::NodeHandle n;
  ros::Rate r(1);
  ros::ServiceClient client = n.serviceClient<patrol_robot::SendGoals>("patrol_robot_client");
  patrol_robot::SendGoals srv;

  geometry_msgs::Pose goal;
  std::vector<geometry_msgs::Pose> goals_nav;
  // double x = 0;
  // for(int i = 0; i < 4; ++i){
  //   goal.position.x = x;
  //   goal.position.y = 0;
  //   goal.position.z = 0;
  //   goal.orientation.x = 0;
  //   goal.orientation.y = 0;
  //   goal.orientation.z = 1;
  //   goal.orientation.w = 0;
  //   goals_nav.push_back(goal);
  //   x += 0.7;
  // }

  goal.position.x = 2;
  goal.position.y = 1;
  goal.position.z = 0;
  goal.orientation.x = 0;
  goal.orientation.y = 0;
  goal.orientation.z = -0.7;
  goal.orientation.w = 0.7;
  goals_nav.push_back(goal);

  goal.position.x = 2;
  goal.position.y = -1;
  goal.position.z = 0;
  goal.orientation.x = 0;
  goal.orientation.y = 0;
  goal.orientation.z = -1;
  goal.orientation.w = 0;
  goals_nav.push_back(goal);

  goal.position.x = 0;
  goal.position.y = -1;
  goal.position.z = 0;
  goal.orientation.x = 0;
  goal.orientation.y = 0;
  goal.orientation.z = 0.7;
  goal.orientation.w = 0.7;
  goals_nav.push_back(goal);

  goal.position.x = 0;
  goal.position.y = 1;
  goal.position.z = 0;
  goal.orientation.x = 0;
  goal.orientation.y = 0;
  goal.orientation.z = 0;
  goal.orientation.w = 1;
  goals_nav.push_back(goal);

  srv.request.goals.poses.resize(goals_nav.size());
  for(unsigned int i = 0; i < goals_nav.size(); ++i){
    srv.request.goals.poses[i] = goals_nav[i];
  }

  //while(1){
  if (client.call(srv))
  {
      if(srv.response.success)
      {
          ROS_INFO("Result: successful!");
      }
      else
      {
          ROS_INFO("Result: failed!");
      }
  }
  else  
  {
    ROS_ERROR("Failed to call service");
    return 1;
  }
  r.sleep();
  //}

  return 0;
}
