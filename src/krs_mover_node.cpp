#include <lwr_project_krs/krs_mover.hpp>

int main(int argc, char** argv) {
  ros::init(argc, argv, "krs_mover_node");
  ros::NodeHandle nh;
  
  double speed_percent = 10;
  
  KrsMover krs_mover;
  krs_mover.moveToStart(100);
  
  geometry_msgs::Pose rel_pose;
  rel_pose.orientation.w = 1;
  rel_pose.orientation.x = 0;
  rel_pose.orientation.y = 0;
  rel_pose.orientation.z = 0;
  rel_pose.position.x = 0;
  rel_pose.position.y = 0;
  rel_pose.position.z = -0.1;
  krs_mover.moveLinRel(rel_pose,100);
  
  
  geometry_msgs::Pose rel_ori;
  rel_ori.orientation.w = 0.854;
  rel_ori.orientation.x = -0.001;
  rel_ori.orientation.y = -0.494;
  rel_ori.orientation.z = 0.165;
  rel_ori.position.x = 0;
  rel_ori.position.y = 0;
  rel_ori.position.z = 0;
  krs_mover.moveLinRel(rel_ori,100);
  
  krs_mover.moveLinRelInTool(rel_pose,100);
  
  krs_mover.moveAPlat(100);
  
  krs_mover.moveToHeight(0.4,100,false,0);

  ros::shutdown();
  return 0;
}