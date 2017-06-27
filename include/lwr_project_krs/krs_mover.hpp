#ifndef LWRPROJECTKRS_KRSMOVER_HPP
#define LWRPROJECTKRS_KRSMOVER_HPP

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <krl_msgs/PTPAction.h>
#include <krl_msgs/LINAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf_conversions/tf_eigen.h>


class KrsMover
{
  public:
     KrsMover();
     
     bool moveToJointPosition(const std::vector<double> joint_vals,double velocity_percent);
     bool moveToCartesianPose(const geometry_msgs::Pose pose, double velocity_percent);
     bool moveToCartesianPoseUsingPTP(const geometry_msgs::Pose pose,bool use_relative,double velocity_percent);
     bool moveLinRel(const geometry_msgs::Pose pose,double velocity_percent);
     bool moveLinRelInTool(const geometry_msgs::Pose pose,double velocity_percent);
     bool moveAPlat(double velocity_percent);
     bool moveToHeight(double height,double velocity_percent,bool stop_on_force ,double max_force );
     bool moveToStart(double velocity_percent);
     
  private:
    actionlib::SimpleActionClient<krl_msgs::PTPAction> ptp_ac;
    actionlib::SimpleActionClient<krl_msgs::LINAction> lin_ac;
};

#endif // LWRPROJECTKRS_KRSMOVER_HPP