#include <lwr_project_krs/krs_mover.hpp>

int main(int argc, char** argv) {
  ros::init(argc, argv, "krs_mover_node");
  ros::NodeHandle nh;
  
  KrsMover krs_mover;
  
  double speed_percent = 50;
  double max_force = 0.5;
  
  krs_mover.moveToStart(speed_percent);
  
  geometry_msgs::Pose rel_pose;
  rel_pose.orientation.w = 1;
  rel_pose.orientation.x = 0;
  rel_pose.orientation.y = 0;
  rel_pose.orientation.z = 0;
  rel_pose.position.x = 0;
  rel_pose.position.y = 0;
  rel_pose.position.z = -0.3;
  krs_mover.moveLinRel(rel_pose,speed_percent,true,max_force);
  
  geometry_msgs::Pose rel_ori;
  rel_ori.orientation.w = 0.854;
  rel_ori.orientation.x = -0.001;
  rel_ori.orientation.y = -0.494;
  rel_ori.orientation.z = 0.165;
  rel_ori.position.x = 0;
  rel_ori.position.y = 0;
  rel_ori.position.z = 0;
  krs_mover.moveLinRel(rel_ori,speed_percent);
  
  krs_mover.moveLinRelInTool(rel_pose,speed_percent);
  
  krs_mover.moveAPlat(speed_percent);
  
  krs_mover.moveToHeight(0.4,speed_percent,true, max_force);

  ros::shutdown();
  return 0;
}