#include <lwr_project_krs/krs_mover.hpp>

KrsMover::KrsMover() : 
  ptp_ac_("ptp"), lin_ac_("lin"), spinner_(1)
{
  ee_frame_ = "ati_link";
  
  // Start AsyncSpinner
  spinner_.start();

  // Wait for the action servers to be ready
  ROS_INFO("Waiting for the action servers to be ready ...");
  lin_ac_.waitForServer();
  ptp_ac_.waitForServer();
  ROS_INFO("Action servers ready !");
  
  // Initialize chain utils
  arm_.init();
  
  // Subscribe to joint_state to get the current state of the robot
  state_sub_ = nh_.subscribe("/joint_states", 1, &KrsMover::state_callback, this);
  
  // Wait to give time to update the current model
  sleep(1);
}


void KrsMover::state_callback(const sensor_msgs::JointState::ConstPtr& msg){
  // Update the kinematic model
  arm_.setState(msg->position, msg->velocity);
  arm_.updateModel();
  curr_jnt_pos_ = arm_.getJointPositions();
  curr_jnt_vel_ = arm_.getJointVelocities();
  curr_cart_pos_ = arm_.getSegmentPosition(ee_frame_);
  curr_cart_vel_ = arm_.getSegmentVelocity(ee_frame_);
}

bool KrsMover::moveToJointPosition(const std::vector<double> joint_vals,double velocity_percent){
  std::vector<unsigned char> mask(joint_vals.size(), 1);
  krl_msgs::PTPGoal ptp_goal;
  ptp_goal.RPY_mask.x = 1;
  ptp_goal.RPY_mask.y = 1;
  ptp_goal.RPY_mask.z = 1;
  ptp_goal.XYZ_mask.x = 1;
  ptp_goal.XYZ_mask.y = 1;
  ptp_goal.XYZ_mask.z = 1;
  ptp_goal.vel_percent = velocity_percent;

  std::vector<float> goal_vals;
  for(int i = 0; i<joint_vals.size();i++ )
    goal_vals.push_back(joint_vals[i]);

  ptp_goal.ptp_goal = goal_vals;
  ptp_goal.ptp_input_type = krl_msgs::PTPGoal::Joint;
  ptp_goal.ptp_mask = mask;
  ptp_goal.use_radians = true;
  ptp_goal.use_relative = false;

  ptp_ac_.sendGoalAndWait(ptp_goal);
  return true;
}

bool KrsMover::moveToJointPositionRel(const std::vector<double> joint_vals,double velocity_percent){
  std::vector<unsigned char> mask(joint_vals.size(), 1);
  krl_msgs::PTPGoal ptp_goal;
  ptp_goal.RPY_mask.x = 1;
  ptp_goal.RPY_mask.y = 1;
  ptp_goal.RPY_mask.z = 1;
  ptp_goal.XYZ_mask.x = 1;
  ptp_goal.XYZ_mask.y = 1;
  ptp_goal.XYZ_mask.z = 1;
  ptp_goal.vel_percent = velocity_percent;

  std::vector<float> goal_vals;
  for(int i = 0; i<joint_vals.size();i++ )
    goal_vals.push_back(joint_vals[i]);

  ptp_goal.ptp_goal = goal_vals;
  ptp_goal.ptp_input_type = krl_msgs::PTPGoal::Joint;
  ptp_goal.ptp_mask = mask;
  ptp_goal.use_radians = true;
  ptp_goal.use_relative = true;

  ptp_ac_.sendGoalAndWait(ptp_goal);
  return true;
}

bool KrsMover::moveToCartesianPose(const geometry_msgs::Pose pose, double velocity_percent,bool stop_on_force ,double max_force ){
  krl_msgs::LINGoal lin_goal;
  lin_goal.RPY_mask.x = 1;
  lin_goal.RPY_mask.y = 1;
  lin_goal.RPY_mask.z = 1;
  lin_goal.XYZ_mask.x = 1;
  lin_goal.XYZ_mask.y = 1;
  lin_goal.XYZ_mask.z = 1;
  lin_goal.vel_percent = velocity_percent;
  lin_goal.use_relative = false;
  lin_goal.in_tool_frame = false;
  lin_goal.stop_on_force = stop_on_force;
  lin_goal.max_allowed_force = max_force;
  geometry_msgs::Vector3 xyz, rpy;
  xyz.x = pose.position.x;
  xyz.y = pose.position.y;
  xyz.z = pose.position.z;
  lin_goal.XYZ = xyz;
  tf::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  tf::Matrix3x3 m(q);

  m.getRPY(rpy.x, rpy.y, rpy.z);

  lin_goal.RPY = rpy;

  lin_ac_.sendGoalAndWait(lin_goal);
  return true;
}

bool KrsMover::moveToCartesianPoseUsingPTP(const geometry_msgs::Pose pose,bool use_relative,double velocity_percent){
  krl_msgs::PTPGoal ptp_goal;
  ptp_goal.RPY_mask.x = 1;
  ptp_goal.RPY_mask.y = 1;
  ptp_goal.RPY_mask.z = 1;
  ptp_goal.XYZ_mask.x = 1;
  ptp_goal.XYZ_mask.y = 1;
  ptp_goal.XYZ_mask.z = 1;
  ptp_goal.vel_percent = velocity_percent;

  geometry_msgs::Vector3 xyz, rpy;
  xyz.x = pose.position.x;
  xyz.y = pose.position.y;
  xyz.z = pose.position.z;

  tf::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  tf::Matrix3x3 m(q);

  m.getRPY(rpy.x, rpy.y, rpy.z);

  ptp_goal.XYZ = xyz;
  ptp_goal.RPY = rpy;
  ptp_goal.ptp_input_type = krl_msgs::PTPGoal::Cartesian;
  ptp_goal.use_radians = true;
  ptp_goal.use_relative = use_relative;

  ptp_ac_.sendGoalAndWait(ptp_goal);
  return true;
}

bool KrsMover::moveLinRel(const geometry_msgs::Pose pose,double velocity_percent,bool stop_on_force ,double max_force ){
  krl_msgs::LINGoal lin_goal;
  lin_goal.RPY_mask.x = 1;
  lin_goal.RPY_mask.y = 1;
  lin_goal.RPY_mask.z = 1;
  lin_goal.XYZ_mask.x = 1;
  lin_goal.XYZ_mask.y = 1;
  lin_goal.XYZ_mask.z = 1;
  lin_goal.vel_percent = velocity_percent;
  lin_goal.use_relative = true;
  lin_goal.in_tool_frame = false;
  lin_goal.stop_on_force = stop_on_force;
  lin_goal.max_allowed_force = max_force;
  geometry_msgs::Vector3 xyz, rpy;
  xyz.x = pose.position.x;
  xyz.y = pose.position.y;
  xyz.z = pose.position.z;
  lin_goal.XYZ = xyz;
  tf::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  tf::Matrix3x3 m(q);

  m.getRPY(rpy.x, rpy.y, rpy.z);

  lin_goal.RPY = rpy;

  lin_ac_.sendGoalAndWait(lin_goal);
  return true;
}

bool KrsMover::moveLinRelInTool(const geometry_msgs::Pose pose,double velocity_percent,bool stop_on_force ,double max_force ){
  krl_msgs::LINGoal lin_goal;
  lin_goal.RPY_mask.x = 1;
  lin_goal.RPY_mask.y = 1;
  lin_goal.RPY_mask.z = 1;
  lin_goal.XYZ_mask.x = 1;
  lin_goal.XYZ_mask.y = 1;
  lin_goal.XYZ_mask.z = 1;
  lin_goal.vel_percent = velocity_percent;
  lin_goal.use_relative = true;
  lin_goal.in_tool_frame = true;
  lin_goal.stop_on_force = stop_on_force;
  lin_goal.max_allowed_force = max_force;
  geometry_msgs::Vector3 xyz, rpy;
  xyz.x = pose.position.x;
  xyz.y = pose.position.y;
  xyz.z = pose.position.z;
  lin_goal.XYZ = xyz;
  tf::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  tf::Matrix3x3 m(q);
  m.getRPY(rpy.x, rpy.y, rpy.z);
  lin_goal.RPY = rpy;
  
  lin_ac_.sendGoalAndWait(lin_goal);
  return true;
}

bool KrsMover::moveAPlat(double velocity_percent){
  krl_msgs::LINGoal lin_goal;
  lin_goal.RPY_mask.x = 1;
  lin_goal.RPY_mask.y = 1;
  lin_goal.RPY_mask.z = 0;
  lin_goal.XYZ_mask.x = 0;
  lin_goal.XYZ_mask.y = 0;
  lin_goal.XYZ_mask.z = 0;
  lin_goal.vel_percent = velocity_percent;
  lin_goal.use_relative = false;
  lin_goal.in_tool_frame = false;
  lin_goal.stop_on_force = false;
  geometry_msgs::Vector3 rpy;
  rpy.x = M_PI;
  rpy.y = 0.0;//M_PI;
  //rpy.z = 0.0;
  lin_goal.RPY = rpy;

  lin_ac_.sendGoalAndWait(lin_goal);
  return true;
}

bool KrsMover::moveToHeight(double height,double velocity_percent,bool stop_on_force ,double max_force ){ 
  krl_msgs::LINGoal lin_goal;
  lin_goal.RPY_mask.x = 0;
  lin_goal.RPY_mask.y = 0;
  lin_goal.RPY_mask.z = 0;
  lin_goal.XYZ_mask.x = 0;
  lin_goal.XYZ_mask.y = 0;
  lin_goal.XYZ_mask.z = 1;
  lin_goal.vel_percent = velocity_percent;
  lin_goal.use_relative = false;
  lin_goal.in_tool_frame = false;
  lin_goal.stop_on_force = stop_on_force;
  lin_goal.max_allowed_force = max_force;
  lin_goal.XYZ.z = height;

  lin_ac_.sendGoalAndWait(lin_goal);
  return true;
}

bool KrsMover::moveToStart(double velocity_percent){
  double start_tab[] = {M_PI/2.0, 0.0, 0.0, -M_PI/2.0, 0.0, M_PI/2.0, 0.0};
  std::vector<double> start;
  start.assign(start_tab,start_tab+7);
  
  return moveToJointPosition(start, velocity_percent);
}