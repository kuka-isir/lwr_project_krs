#include <stdio.h>
#include <string.h>    //strlen
#include <sys/socket.h>
#include <arpa/inet.h> //inet_addr
#include <unistd.h>    //write
#include <iostream>
#include <map>
#include <string>

#include <ros/ros.h>
#include <std_srvs/Empty.h>
 
#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"

#include <lwr_project_krs/krs_mover.hpp>

using namespace rapidjson; 

enum Types { not_defined_type,
             stop, 
             home,
             unstop,
             move_joint,
             move_tool,
             get_status
};

std::map<std::string, Types> s_mapStringValues;

void Initialize();

void Initialize(){
  s_mapStringValues["stop"] = stop;
  s_mapStringValues["unstop"] = unstop;
  s_mapStringValues["home"] = home;
  s_mapStringValues["move_joint"] = move_joint;
  s_mapStringValues["move_tool"] = move_tool;
  s_mapStringValues["get_status"] = get_status;
}

int socket_desc , client_sock;
struct sockaddr_in server , client;

KrsMover* krs_mover;
ros::ServiceClient stop_client, unstop_client;

bool process_request(std::string query, std::string& answer);

bool connect(int port_nb);


int main(int argc , char *argv[]){
  ros::init(argc, argv, "krs_server");
  ros::NodeHandle nh;
  stop_client = nh.serviceClient<std_srvs::Empty>("/lwr_krl_tool/send_stop2");
  unstop_client = nh.serviceClient<std_srvs::Empty>("/lwr_krl_tool/unset_stop2");
  
  // Init the string map
  Initialize();
  
  krs_mover = new KrsMover;

  // Read port number from argument or use default one
  int port_nb = 8888;
  if (argc > 1)
    port_nb = atoi(argv[1]);
  
  // If socket connection fails, exit
  if(connect(port_nb)==false)
    return 1;
  
  int read_size;
  char client_message[2000];
  // Receive a message from client
  while( (read_size = recv(client_sock , client_message , 2000 , 0)) > 0 ){
    
    // Read message
    std::string json(client_message,read_size);
    
    // Process message
    std::string answer, empty_str;
    if(!process_request(json, answer)){
      write(client_sock , empty_str.c_str() , strlen(empty_str.c_str()));
      continue;
    }
    
    // Send the response to the client
    std::cout << "Sending answer : \n" <<answer << std::endl;
    answer += "\r\n"; 
    write(client_sock , answer.c_str() , strlen(answer.c_str()));
  }
    
  if(read_size == 0)
    std::cout <<"Client disconnected"<<std::endl;
  
  else if(read_size == -1)
    std::cerr <<"recv failed"<<std::endl;
  
  close(client_sock);
  close(socket_desc);
  return 0;
}

bool process_request(std::string query, std::string& answer){
  std::cout << "Message received : \n" <<query << std::endl;
  
  answer = "";
        
  KDL::JntArray jnts = krs_mover->getCurrentJointPos();
  KDL::Frame pose = krs_mover->getCurrentCartPos();
  geometry_msgs::Wrench wrench = krs_mover->getCurrentFtWrench();
  double roll,pitch,yaw;
  pose.M.GetRPY(roll, pitch, yaw);
  
  // Parse JSON
  Document document;
  document.Parse(query.c_str());
  
  if(!document.IsObject()){
    std::cerr << "Received data could not be parsed as JSON..." <<std::endl;
    return false;
  }

  // Modify it by DOM
  if(!document.HasMember("type")){
    std::cerr << "Message does not contain \"type\" " <<std::endl;
    return false;
  }
  
  // Read request type
  Value& type = document["type"];
  
  std::cout << "Request is of type " <<type.GetString() << std::endl;
  
  // Executing request
  std_srvs::Empty empty_request;
  double speed = 1, max_force = 0.5;
  bool relative = false, in_tool = false, stop_on_force = false;
  bool succes = true;
  std::vector<double> joint_vals;
  std::vector<double> pose_vals;
  StringBuffer buffer;
  Writer<StringBuffer> writer(buffer);
  writer.StartObject(); 
  
  switch(s_mapStringValues[type.GetString()]){
    
    case stop:
      std::cout << "Sending STOP2"<<std::endl;
      stop_client.call(empty_request);
      break;
      
    case unstop:
      std::cout << "Removing STOP2"<<std::endl;
      unstop_client.call(empty_request);
      break;
      
    case home:
      if(document.HasMember("params")){
        Value& params = document["params"];
        if(params.HasMember("speed"))
           speed = params["speed"].GetDouble();
      }
      krs_mover->moveToStart(speed);
      break;
      
    case move_joint:
      if(document.HasMember("params")){
        Value& params = document["params"];
        if(params.HasMember("speed"))
           speed = params["speed"].GetDouble();
        
        if(params.HasMember("relative"))
           relative = params["relative"].GetBool();
        
        if(params.HasMember("array")){
          //
          Value& array = params["array"]; 
          if(!array.IsArray()){
            std::cerr << "\"array\" param needs to be an array of doubles \"..."<<std::endl;
            succes = false;
          }
          
          // Read array
          joint_vals.resize(array.Size());
          for (SizeType i = 0; i < array.Size(); i++)
            joint_vals[i]= array[i].GetDouble();

          // Check there is enough value in array
          if(joint_vals.size()!=7){
            std::cerr << "\"array\" should be an array of 7 doubles \"..."<<std::endl;
            succes =  false;
          }          
          
          // Move to joint position
          if(relative){
            krs_mover->moveToJointPositionRel(joint_vals,speed);
          }else
            krs_mover->moveToJointPosition(joint_vals,speed);
        }
        break;
      }
      else{
        std::cerr << "Params required for type \""<<type.GetString()<<"\" ..."<<std::endl;
        succes = false;
      }
      break;
      
    case move_tool:
      if(document.HasMember("params")){
        Value& params = document["params"];
        if(params.HasMember("speed"))
           speed = params["speed"].GetDouble();
        
        if(params.HasMember("relative"))
           relative = params["relative"].GetBool();
        
        if(params.HasMember("in_tool"))
           in_tool = params["in_tool"].GetBool();
        
        if(params.HasMember("stop_on_force"))
           stop_on_force = params["stop_on_force"].GetBool();
        
        if(params.HasMember("max_force"))
           max_force = params["max_force"].GetDouble();
        
        if(params.HasMember("array")){
          //
          Value& array = params["array"]; 
          if(!array.IsArray()){
            std::cerr << "\"array\" param needs to be an array of doubles \"..."<<std::endl;
            succes = false;
          }
          
          // Read array
          pose_vals.resize(array.Size());
          for (SizeType i = 0; i < array.Size(); i++)
            pose_vals[i]= array[i].GetDouble();

          // Check there is enough value in array
          if(pose_vals.size()!=6){
            std::cerr << "\"array\" should be an array of 6 doubles \"..."<<std::endl;
            succes = false;
          }          
          
          // Convert vector to geometry Pose
          geometry_msgs::Pose pose;
          pose.position.x = pose_vals[0];
          pose.position.y = pose_vals[1];
          pose.position.z = pose_vals[2];
          tf::Quaternion quat = tf::createQuaternionFromRPY(pose_vals[3],pose_vals[4],pose_vals[5]);
          pose.orientation.x = quat.x();
          pose.orientation.y = quat.y();
          pose.orientation.z = quat.z();
          pose.orientation.w = quat.w();
          
          // Move to joint position
          if(relative){
            if(in_tool)
              krs_mover->moveLinRelInTool(pose, speed, stop_on_force, max_force);
            else
              krs_mover->moveLinRel(pose, speed, stop_on_force, max_force);
          }else
            krs_mover->moveToCartesianPose(pose,speed,stop_on_force,max_force);
        }
        break;
      }
      else{
        std::cerr << "Params required for type \""<<type.GetString()<<"\" ..."<<std::endl;
        succes = false;
      }
      break;
      
    case get_status:
      std::cout << "Returning robot's status !" << std::endl;
      writer.Key("joints");
      writer.StartArray();
      for(int i=0; i<7;i++)
        writer.Double(jnts.data[i]);      
      writer.EndArray();
      writer.Key("tool");
      writer.StartArray();
      for(int i=0; i<3;i++)
        writer.Double(pose.p.data[i]);
      writer.Double(roll);
      writer.Double(pitch);
      writer.Double(yaw);
      writer.EndArray();
      writer.Key("ft_sensor");
      writer.StartArray();
      writer.Double(wrench.force.x);
      writer.Double(wrench.force.y);
      writer.Double(wrench.force.z);
      writer.Double(wrench.torque.x);
      writer.Double(wrench.torque.y);
      writer.Double(wrench.torque.z);
      writer.EndArray();
      break;
    
    case not_defined_type:
      std::cerr << "Type \""<<type.GetString()<<"\" is not known ..."<<std::endl;
      succes = false;
      break;
      
    default:
      std::cerr << "Type \""<<type.GetString()<<"\" is not known ..."<<std::endl;
      succes = false;
      break;
  }
  
  writer.Key("success");
  writer.Bool(succes);
  writer.EndObject();
  
  answer = buffer.GetString();
  
  return true;
}

bool connect(int port_nb){
  int c;
    
  // Create socket
  socket_desc = socket(AF_INET , SOCK_STREAM , 0);
  if (socket_desc == -1){
    std::cerr << "Could not create socket"<<std::endl;
    return false;  
  }
  std::cout <<"Socket created"<<std::endl;
  
  // Prepare the sockaddr_in structure
  server.sin_family = AF_INET;
  server.sin_addr.s_addr = INADDR_ANY;
  server.sin_port = htons( port_nb );
    
  // Bind
  if( bind(socket_desc,(struct sockaddr *)&server , sizeof(server)) < 0){
    std::cerr <<"bind failed. Error"<<std::endl;
    return false;
  }
  std::cout <<"bind done on port "<<port_nb <<std::endl;
    
  // Listen
  listen(socket_desc , 3);
    
  // Accept and incoming connection
  std::cout <<"Waiting for incoming connections..."<<std::endl;
  c = sizeof(struct sockaddr_in);
  client_sock = accept(socket_desc, (struct sockaddr *)&client, (socklen_t*)&c);
  if (client_sock < 0){
    std::cerr <<"accept failed"<<std::endl;
    return false;
  }
  std::cout <<"Connection accepted"<<std::endl;
  return true;
}
