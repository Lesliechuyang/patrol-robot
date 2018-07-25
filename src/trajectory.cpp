#include <patrol_robot/trajectory.h>
#include <math.h>

namespace patrol_robot{
    Trajectory::Trajectory(){
        
    }

    Trajectory::~Trajectory(){

    }

    bool Trajectory::makePlan(geometry_msgs::Pose current_pose, std::vector<geometry_msgs::Pose> g, std::vector< std::vector<geometry_msgs::Pose> >& traj, double s){
        //目标点向量为空，返回false
        if(!g.size()){
            ROS_INFO("Empty goals!");
            return false;
        }
        goals = g;//目标点
        step_dis = s;//步长
        int num = goals.size();//目标点个数
        //如果只有一个点
        if(num == 1 ){
            //处理第一个目标点
            traj.push_back(createTrajectory(current_pose, goals[0], current_pose));
            trajectory.push_back(createTrajectory(current_pose, goals[0], current_pose));
            if(traj.size() == 0 || traj[0].size() == 0) return false;
            return true;
        }
        else{
            //对剩余每个目标点进行规划,除了最后一个点
            for(int i = 1; i < (num - 1); ++i){
                traj.push_back(createTrajectory(goals[i-1], goals[i], goals[i+1]));
                trajectory.push_back(createTrajectory(goals[i-1], goals[i], goals[i+1]));
                if(traj.size() == 0 || traj[i].size() == 0) return false;
            }
            //对最后一个点进行处理
            traj.push_back(createTrajectory(goals[num - 1], goals[num], goals[num - 1]));
            trajectory.push_back(createTrajectory(goals[num - 1], goals[num], goals[num - 1]));
            if(traj.size() == 0 || traj[num].size() == 0) return false;
        }
        //生成完毕
    }

    bool Trajectory::getPositionAt(int index1, int index2, geometry_msgs::Pose& pose){
        //判断是否超出长度
        if( (index1 > trajectory.size()) || (index2 > trajectory[index1].size()) )  return false;
        pose = trajectory[index1][index2];
        return true;
    }

    double Trajectory::calDistance(double x, double y){
        return sqrt(x*x + y*y);
    }

    geometry_msgs::Quaternion Trajectory::getAngleDiff(geometry_msgs::Pose p1, geometry_msgs::Pose p2){

    }

    std::vector<geometry_msgs::Pose> Trajectory::createTrajectory(geometry_msgs::Pose begin, geometry_msgs::Pose end, geometry_msgs::Pose end_n){
        geometry_msgs::Pose pose_tmp;
        std::vector<geometry_msgs::Pose> traj_tmp;
        double x_0 = 0, y_0 = 0, delta_x = 0, delta_y = 0;
        x_0 = begin.position.x;
        y_0 = begin.position.y;
        delta_x = end.position.x - x_0;
        delta_y = end.position.y - y_0;
        int steps = calDistance(delta_x, delta_y) / step_dis;//总步数
        //计算一个路径上的每个点
        for(int i = 0; i < steps; ++i){
            pose_tmp.position.x = x_0 + i * step_dis;
            pose_tmp.position.y = y_0 + i * step_dis;
            traj_tmp.push_back(pose_tmp);
        }
        pose_tmp.position = end.position;
        pose_tmp.orientation = getAngleDiff(end, end_n);//计算当前点和下个点的连线的角度，设为最后一个点的orientaiton
        traj_tmp.push_back(pose_tmp);

        return traj_tmp;
    }
};
