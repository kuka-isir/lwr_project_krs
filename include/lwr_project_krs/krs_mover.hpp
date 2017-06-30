#ifndef LWRPROJECTKRS_KRSMOVER_HPP
#define LWRPROJECTKRS_KRSMOVER_HPP

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <krl_msgs/PTPAction.h>
#include <krl_msgs/LINAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf_conversions/tf_eigen.h>

#include <rtt_ros_kdl_tools/chain_utils.hpp>

class KrsMover
{
  public:
    KrsMover();
     
    bool moveToJointPosition(const std::vector<double> joint_vals,double velocity_percent);
    bool moveToJointPositionRel(const std::vector<double> joint_vals,double velocity_percent);
    bool moveToCartesianPose(const geometry_msgs::Pose pose, double velocity_percent,bool stop_on_force = false,double max_force = 0.5);
    bool moveToCartesianPoseUsingPTP(const geometry_msgs::Pose pose,bool use_relative,double velocity_percent);
    bool moveLinRel(const geometry_msgs::Pose pose,double velocity_percent,bool stop_on_force = false ,double max_force = 0.5 );
    bool moveLinRelInTool(const geometry_msgs::Pose pose,double velocity_percent,bool stop_on_force = false ,double max_force = 0.5);
    bool moveAPlat(double velocity_percent);
    bool moveToHeight(double height,double velocity_percent,bool stop_on_force = false ,double max_force = 0.5);
    bool moveToStart(double velocity_percent);
    
    void state_callback(const sensor_msgs::JointState::ConstPtr& msg);
    
    KDL::Frame getCurrentCartPos(){
      return curr_cart_pos_;
    }
    KDL::Twist getCurrentCartVel(){
      return curr_cart_vel_;
    }
    KDL::JntArray getCurrentJointPos()
    {
      return curr_jnt_pos_;
    }
    KDL::JntArray getCurrentJointVel()
    {
      return curr_jnt_vel_;
    }

     
  private:
    
    actionlib::SimpleActionClient<krl_msgs::PTPAction> ptp_ac_;
    actionlib::SimpleActionClient<krl_msgs::LINAction> lin_ac_;
    rtt_ros_kdl_tools::ChainUtils arm_;    
    ros::NodeHandle nh_;
    ros::AsyncSpinner spinner_;
    ros::Subscriber state_sub_;
    
    KDL::JntArray curr_jnt_pos_;
    KDL::JntArray curr_jnt_vel_;
    KDL::Frame curr_cart_pos_;
    KDL::Twist curr_cart_vel_;
    
    std::string ee_frame_;
};

#endif // LWRPROJECTKRS_KRSMOVER_HPP