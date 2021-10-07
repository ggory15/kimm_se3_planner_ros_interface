#include <pinocchio/fwd.hpp>
#include "ros/ros.h"

#include "kimm_se3_planner_ros_interface/plan_se3_path.h"
#include "kimm_se3_planner_ros_interface/action_se3_path.h"

#include "std_msgs/Bool.h"
#include "geometry_msgs/Transform.h"

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <kimm_trajectory_smoother/Trajectory.h>
#include <kimm_trajectory_smoother/Path.h>
#include <kimm_hqp_controller/trajectory/trajectory_se3.hpp>


using namespace std;
using namespace Eigen;
using namespace kimmhqp::trajectory;
using namespace pinocchio;

int playTime_;
bool is_calc_, is_run_, is_wholebody_;

ros::ServiceServer planning_server_;
ros::ServiceServer action_server_;

std::shared_ptr<TrajectorySE3Cubic> traj_cubic_;
std::shared_ptr<TrajectorySE3Timeopt> traj_timeopt_;
std::shared_ptr<TrajectorySE3Constant> traj_const_;
int type_ = 0;
Eigen::VectorXd kp_, kv_;
Eigen::Vector3d vel_limit_, acc_limit_;
Eigen::VectorXi mask_;
std::vector<Eigen::VectorXd> res_traj_;
std::vector<geometry_msgs::Transform> res_j_traj_;
SE3 c_se3_, t_se3_;
double duration_ = 0;

bool calculation(kimm_se3_planner_ros_interface::plan_se3_path::Request &req, kimm_se3_planner_ros_interface::plan_se3_path::Response &res)
{
    type_ = req.traj_type;
    is_wholebody_ = req.iswholebody.data;

    c_se3_.translation()(0) = req.current_se3.translation.x;
    c_se3_.translation()(1) = req.current_se3.translation.y;
    c_se3_.translation()(2) = req.current_se3.translation.z;
    Eigen::Quaterniond quat;
    quat.x() = req.current_se3.rotation.x ;
    quat.y() = req.current_se3.rotation.y;
    quat.z() = req.current_se3.rotation.z;
    quat.w() = req.current_se3.rotation.w;
    quat.normalize();
    c_se3_.rotation() = quat.toRotationMatrix();
    
    t_se3_.translation()(0) = req.target_se3[0].translation.x;
    t_se3_.translation()(1) = req.target_se3[0].translation.y;
    t_se3_.translation()(2) = req.target_se3[0].translation.z;
    quat.x() = req.target_se3[0].rotation.x;
    quat.y() = req.target_se3[0].rotation.y;
    quat.z() = req.target_se3[0].rotation.z;
    quat.w() = req.target_se3[0].rotation.w;
    quat.normalize();
    t_se3_.rotation() = quat.toRotationMatrix();
    
    mask_.setZero(6);
    kp_.setZero(6);
    kv_.setZero(6);

    vel_limit_.setOnes(3);
    acc_limit_.setOnes(3);
        
    for (int i=0; i<6; i++){
        mask_(i) = req.mask[i].data;
        kp_(i) = req.kp[i];
        kv_(i) = req.kv[i];
    }

    if (type_ == 0){
        traj_const_->setReference(t_se3_);
        res_traj_= traj_const_->getWholeTrajectory();
    }
    else if (type_ == 1){
        duration_ = req.duration;
        traj_cubic_->setInitSample(c_se3_);
        traj_cubic_->setStartTime(0.);
        traj_cubic_->setDuration(duration_);
        traj_cubic_->setGoalSample(t_se3_);

        res_traj_= traj_cubic_->getWholeTrajectory();
    }
    else{
        vel_limit_ *= req.vel_limit;
        acc_limit_ *= req.acc_limit;

        traj_timeopt_->setMaxVelocity(vel_limit_);
        traj_timeopt_->setMaxAcceleration(acc_limit_);
        traj_timeopt_->clearWaypoints();
        traj_timeopt_->setStartTime(0.0);
        traj_timeopt_->addWaypoint(c_se3_);

        for (int j=0; j<req.target_se3.size(); j++){
            t_se3_.translation()(0) = req.target_se3[j].translation.x;
            t_se3_.translation()(1) = req.target_se3[j].translation.y;
            t_se3_.translation()(2) = req.target_se3[j].translation.z;
            quat.x() = req.target_se3[j].rotation.x;
            quat.y() = req.target_se3[j].rotation.y;
            quat.z() = req.target_se3[j].rotation.z;
            quat.w() = req.target_se3[j].rotation.w;
            t_se3_.rotation() = quat.toRotationMatrix();
            traj_timeopt_->addWaypoint(t_se3_);
        }
    
        res_traj_= traj_timeopt_->getWholeTrajectory();
    }
    geometry_msgs::Transform step;
    Eigen::Matrix3d rot; 
    for (int i=0; i<res_traj_.size(); i++){
        step.translation.x = res_traj_[i](0);
        step.translation.y = res_traj_[i](1);
        step.translation.z = res_traj_[i](2);
        rot.col(0) = res_traj_[i].segment(3,3);
        rot.col(1) = res_traj_[i].segment(6,3);
        rot.col(2) = res_traj_[i].segment(9,3);
        quat = rot;
        step.rotation.x = quat.x();
        step.rotation.y = quat.y();
        step.rotation.z = quat.z();
        step.rotation.w = quat.w();
        
        res.res_traj.push_back(step);
    }
    res_j_traj_ = res.res_traj;

    is_calc_ = true;
    is_run_ = false;
    return true;
}
bool updateTrajectory(kimm_se3_planner_ros_interface::action_se3_path::Request &req, kimm_se3_planner_ros_interface::action_se3_path::Response &res)
{
    if (is_calc_ && !is_run_){
    	is_run_ = true;
 	    ROS_INFO("is_run: [%d]", is_run_ ); // res.q_trajectory.se3_names[0].c_str()
        res.kp.clear();
        res.kv.clear();

        res.res_traj = res_j_traj_;
        res.iswholebody.data = is_wholebody_;
        for (int i=0; i<6; i++){
            res.kp.push_back(kp_(i));
            res.kv.push_back(kv_(i));
        }

        return true;
    }
    else{
    	res_j_traj_.clear();
    	res.res_traj.clear();
        return true;
    }
    return true;
}

int main(int argc, char **argv)
{

    struct timespec spec;
    clock_gettime(CLOCK_REALTIME, &spec);
    srand(spec.tv_nsec);

    ros::init(argc, argv, "SE3TrajPlannerServer");
    ros::NodeHandle nh("~");  
    
    planning_server_ = nh.advertiseService("plan_se3_path", calculation);
    action_server_ = nh.advertiseService("action_se3_path", updateTrajectory);
    
    traj_cubic_ = std::make_shared<TrajectorySE3Cubic>("traj_cubic");
    traj_timeopt_ = std::make_shared<TrajectorySE3Timeopt>("traj_timeopt");
    traj_const_ = std::make_shared<TrajectorySE3Constant>("traj_constant");

    ROS_INFO("ready srv server!");
    
    while (ros::ok())
    { 


        ros::spinOnce();       
    } 

    return 0;
}
