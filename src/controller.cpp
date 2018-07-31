#include <patrol_robot/controller.h>

namespace patrol_robot{
    Controller::Controller(Trajectory* traj, double controller_frequency, double controller_patience_pose, double controller_patience_theta, double Kx, double Ky, double Ktheta):
    traj_(traj), 
    controller_patience_pose_(controller_patience_pose), 
    controller_patience_theta_(controller_patience_theta), 
    Kx_(Kx), Ky_(Ky), Ktheta_(Ktheta),
    x_n(0), y_n(0), theta_n(0),
    x_ref_n(0), y_ref_n(0), theta_ref_n(0),
    x_ref_n_plus_1(0), y_ref_n_plus_1(0),
    traj_index(0), point_index(0)
    {

    }
    
    Controller::~Controller(){
        
    }

    bool Controller::setCurrentPose(geometry_msgs::Pose current_pose){
        x_n = current_pose.position.x;
        y_n = current_pose.position.y;
        theta_n = traj_->quaternionToEuler(current_pose.orientation).z;
        return true;
    }

    bool Controller::setCurrentReference(){
        geometry_msgs::Pose reference;
        if(traj_->getPositionAt(traj_index, point_index, reference)){
            x_ref_n = reference.position.x;
            y_ref_n = reference.position.y;
            return true;
        }
        else{
            ROS_INFO("Can't get current reference! \n");
            return false;
        }
    }

    bool Controller::setNextReference(){
        geometry_msgs::Pose reference;
        if(traj_->getPositionAt(traj_index, point_index + 1, reference)){
            x_ref_n_plus_1 = reference.position.x;
            y_ref_n_plus_1 = reference.position.y;
            return true;
        }
        else{
            ROS_INFO("Can't get next reference! \n");
            return false;
        }
    }

    double Controller::getDelta_x_n(){
        return x_ref_n_plus_1 - Kx_ * (x_ref_n - x_n) - x_n;
    }

    double Controller::getDelta_y_n(){
        return y_ref_n_plus_1 - Ky_ * (y_ref_n - y_n) - y_n;
    }

    double Controller::getDelta_theta_n(){
        return theta_ref_n - Ktheta_ * (theta_ref_n - theta_n) - theta_n;
    }

    double Controller::computeTheta_ez_n(){
        return atan2(getDelta_y_n(), getDelta_x_n());
    }

    double Controller::computeV_c_n(){
        double delta_x_n = getDelta_x_n();
        double delta_y_n = getDelta_y_n();
        
        return (delta_x_n * cos(theta_ref_n) + delta_y_n * sin(theta_ref_n)) / ( 1.0 / controller_frequency );
    }

    double Controller::computeW_c_n(){
        double w_n = getDelta_theta_n() / ( 1.0 / controller_frequency );
       
        return atan2(sin(w_n), cos(w_n));
    }

    bool Controller::computePositionControlActions(geometry_msgs::Pose current_pose, geometry_msgs::Twist& twist, int t_i, int p_i){
        traj_index = t_i;
        point_index = p_i;
        if(setCurrentPose(current_pose) && setCurrentReference() && setNextReference()){
            theta_ref_n = computeTheta_ez_n();
            v_c_n = computeV_c_n();
            w_c_n = computeW_c_n();
            twist.linear.x = v_c_n;
            twist.angular.z = w_c_n;
            return true;
        }
        else{
            ROS_INFO("Can't computer control action! \n");
            return false;
        }
    }

    bool Controller::computeOrientationControlActions(geometry_msgs::Twist& twist, double angle_diff){
        twist.linear.x = 0;
        twist.linear.y = 0;
        twist.angular.z = -0.5 * angle_diff;
        return true;
    }
};