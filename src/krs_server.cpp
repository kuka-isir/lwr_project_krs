#include <lwr_project_krs/krs_server.hpp>

int main(int argc, char** argv) {
  ros::init(argc, argv, "krs_server");
  ros::NodeHandle nh;
  
  KrsMover krs_mover;

  ros::shutdown();
  return 0;
}