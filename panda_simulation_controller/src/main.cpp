// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <stdio.h>
#include <signal.h>
#include "spline.h"
#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/model.h>
#include <franka/rate_limiting.h>

#include <franka_core_msgs/JointCommand.h>
#include <franka_core_msgs/RobotState.h>

#include "examples_common.h"

#include <iostream>
#include <fstream> 
#include <jsoncpp/json/json.h>
#pragma comment(lib, "jsoncpp.lib")
using namespace std;
typedef std::array<double,7> J;
J joint_states={0,0,0,0,0,0,0};
J coriolis={0,0,0,0,0,0,0};
void ctrlchandler(int){exit(EXIT_SUCCESS);}
void killhandler(int){exit(EXIT_SUCCESS);}
franka::RobotState robot_state;
bool desired_recv=false;
bool is_sim = true;
void rosJointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
	for(int i = 0;i<7;i++){
		robot_state.q.at(i) = msg->position[i+2];
		robot_state.dq.at(i) = msg->velocity[i+2];
	}
}
void rosDesiredJointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
	for(int i = 0;i<7;i++){
		robot_state.q_d.at(i) = msg->position[i];
		robot_state.dq_d.at(i) = msg->position[i];
	}
	desired_recv = true;
}
void rosRobotStateCallback(const franka_core_msgs::RobotState::ConstPtr& msg)
{
	for(int i = 0;i<7;i++){
		coriolis.at(i) = msg->coriolis[i];
	}
}
void printJ(J input,char* str){
	std::cout<<"\t"<<str<<" : ";
	for(int i = 0;i<6;i++)
		std::cout<<input.at(i)<<",";
	std::cout<<input.at(6)<<std::endl;
}
namespace {
class Controller {
 public:
  Controller(size_t dq_filter_size,
             const std::array<double, 7>& K_P,  // NOLINT(readability-identifier-naming)
             const std::array<double, 7>& K_D)  // NOLINT(readability-identifier-naming)
      : dq_current_filter_position_(0), dq_filter_size_(dq_filter_size), K_P_(K_P), K_D_(K_D) {
    std::fill(dq_d_.begin(), dq_d_.end(), 0);
    dq_buffer_ = std::make_unique<double[]>(dq_filter_size_ * 7);
    std::fill(&dq_buffer_.get()[0], &dq_buffer_.get()[dq_filter_size_ * 7], 0);
  }
  inline franka::Torques step(const franka::RobotState& state) {
    updateDQFilter(state);
    std::array<double, 7> tau_J_d;  // NOLINT(readability-identifier-naming)

    for (size_t i = 0; i < 7; i++) {
      tau_J_d[i] = K_P_[i] * (q_d_[i] - state.q[i]) + K_D_[i] * (dq_d_[i] - getDQFiltered(i))+coriolis[i];
    }
    return tau_J_d;
  }
  void updateDQFilter(const franka::RobotState& state) {
    for (size_t i = 0; i < 7; i++) {
      dq_buffer_.get()[dq_current_filter_position_ * 7 + i] = state.dq[i];
    }
    dq_current_filter_position_ = (dq_current_filter_position_ + 1) % dq_filter_size_;
  }
  double getDQFiltered(size_t index) const {
    double value = 0;
    for (size_t i = index; i < 7 * dq_filter_size_; i += 7) {
      value += dq_buffer_.get()[i];
    }
    return value / dq_filter_size_;
  }
  void printJ(J input,char* str){
	std::cout<<"\t"<<str<<" : ";
	for(int i = 0;i<6;i++)
		std::cout<<input.at(i)<<",";
	std::cout<<input.at(6)<<std::endl;
 }
  void setDesired(std::array<double, 7> q, std::array<double, 7> dq){
  	for(int i =0;i<7;i++){
  		dq_d_[i] = dq[i];
  		q_d_[i] = q[i];
  	}
  	//printJ(dq_d_,"dq_d_");
  	//printJ(q_d_,"q_d_");

  }
 private:
  size_t dq_current_filter_position_;
  size_t dq_filter_size_;
  const std::array<double, 7> K_P_;  // NOLINT(readability-identifier-naming)
  const std::array<double, 7> K_D_;  // NOLINT(readability-identifier-naming)
  std::array<double, 7> dq_d_;
  std::array<double, 7> q_d_;


  std::unique_ptr<double[]> dq_buffer_;
};
std::vector<double> generateTrajectory(double a_max) {
  // Generating a motion with smooth velocity and acceleration.
  // Squared sine is used for the acceleration/deceleration phase.
  std::vector<double> trajectory;
  constexpr double kTimeStep = 0.001;          // [s]
  constexpr double kAccelerationTime = 1;      // time spend accelerating and decelerating [s]
  constexpr double kConstantVelocityTime = 1;  // time spend with constant speed [s]
  // obtained during the speed up
  // and slow down [rad/s^2]
  double a = 0;  // [rad/s^2]
  double v = 0;  // [rad/s]
  double t = 0;  // [s]
  while (t < (2 * kAccelerationTime + kConstantVelocityTime)) {
    if (t <= kAccelerationTime) {
      a = pow(sin(t * M_PI / kAccelerationTime), 2) * a_max;
    } else if (t <= (kAccelerationTime + kConstantVelocityTime)) {
      a = 0;
    } else {
      const double deceleration_time =
          (kAccelerationTime + kConstantVelocityTime) - t;  // time spent in the deceleration phase
      a = -pow(sin(deceleration_time * M_PI / kAccelerationTime), 2) * a_max;
    }
    v += a * kTimeStep;
    t += kTimeStep;
    trajectory.push_back(v);
  }
  return trajectory;
}
}  // anonymous namespace


bool ReadFromFile(const char* filename, char* buffer, int len){
	FILE* r = fopen(filename,"rb");
	if (NULL == r)
	   return false;
	size_t fileSize = fread(buffer, 1, len, r);
	fclose(r);
	return true;

}
bool loadJson(Json::Value& input,char* JSON_FILE){
	const int BufferLength = 102400;
	char readBuffer[BufferLength] = {0,};
	if (false == ReadFromFile(JSON_FILE, readBuffer, BufferLength)) 
	  return 0;
	std::string config_doc = readBuffer;

	Json::Reader reader;
	bool parsingSuccessful = reader.parse(config_doc,input);
	if ( !parsingSuccessful ) { 
	std::cout << "Failed to parse configuration\n" << reader.getFormatedErrorMessages(); 
	return 0; 
	}
	return 1;

}


int main(int argc, char** argv) {
	if(argc == 2){

	}else{
		std::cout<<"###PLEASE JSON FILE NAME###"<<std::endl;
		std::cout<<"./controller <json file name> "<<std::endl;
		return -1;
	}





    //***************************JSON LOAD****************************************//
    char* JSON_FILE= argv[1];
    std::string robot_name = "panda_robot";

	Json::Value rootr;
	bool ret = loadJson(rootr, JSON_FILE);

    J q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
	if(ret == 1)
		for(int i = 0;i<7;i++)
			q_goal.at(i) = rootr[robot_name]["init_q"][i].asFloat();
	else
		return 0;
	for(int i = 0;i<7;i++)
		robot_state.q_d.at(i) =q_goal.at(i);

	is_sim = rootr[robot_name]["is_sim"].asBool();
	std::string robot_ip = rootr[robot_name]["robot_ip"].asString();
    //***************************JSON LOAD****************************************//

	std::array<double, 7> K_P_SIM{{20.0, 20.0, 20.0, 20.0, 20.0, 10.0, 10.0}};
	// SIM ROBOT NOLINTNEXTLINE(readability-identifier-naming)
	std::array<double, 7> K_D_SIM{{1.0, 1.0, 1.0, 1.0, 0.5, 0.1, 0.1}};
	for(int i =0;i<7;i++){
		K_P_SIM.at(i) = K_P_SIM.at(i)*30;
		K_D_SIM.at(i) = K_D_SIM.at(i)*10;
			
	}

  const size_t joint_number{3};
  const size_t filter_size{5};




  // NOLINTNEXTLINE(readability-identifier-naming)
  //const std::array<double, 7> K_P{{2000.0, 2000.0, 2000.0, 2000.0, 2000.0, 2000.0, 2000.0}};
  // NOLINTNEXTLINE(readability-identifier-naming)
 // const std::array<double, 7> K_D{{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}};


  

	const double max_acceleration{1.0};

	Controller sim_controller(filter_size, K_P_SIM, K_D_SIM);

	try{ros::init(argc,argv,"trajectory_subscriber");}
	catch(int e){ctrlchandler(1);}


	ros::NodeHandle nh;
	ros::NodeHandle nh2;
	ros::Subscriber desired_sub = nh.subscribe("/desired_joint_states", 1,rosDesiredJointStateCallback); 


	ros::Subscriber sub = nh.subscribe("/panda_simulator/custom_franka_state_controller/joint_states", 1,rosJointStateCallback); 
	ros::Subscriber sub2 = nh.subscribe("/panda_simulator/custom_franka_state_controller/robot_state", 1,rosRobotStateCallback); 
	ros::Publisher pub =  nh2.advertise<franka_core_msgs::JointCommand>("/panda_simulator/motion_controller/arm/joint_commands", 1);	
	ros::Publisher pub_joint =  nh2.advertise<sensor_msgs::JointState>("/joint_states2", 1);	

	ros::Rate r(1000);
	std::cout<<"ROS JOINT CONTROLLER IS ON"<<std::endl;

	unsigned int count = 0;
	

		sim_controller.setDesired(robot_state.q_d,robot_state.dq_d);	
		franka_core_msgs::JointCommand pubmsg;
		pubmsg.mode =pubmsg.TORQUE_MODE;	
		J tau ={0,0,0,0,0,0,0};
		J filtervalue = {0,0,0,0,0,0,0};
		std::unique_ptr<double[]> dq_buffer_;

		size_t dq_current_filter_position_=0;
		size_t dq_filter_size_=20;
		sensor_msgs::JointState joint_msg;

		while (ros::ok()){
			pubmsg.header.stamp = ros::Time::now();
			franka::Torques tq = sim_controller.step(robot_state);
			double err_sum = 0;
			for(int i = 0;i<7;i++){
				tau[i] = K_P_SIM[i]*(robot_state.q_d[i]-robot_state.q[i])+K_D_SIM[i]*(robot_state.dq_d[i]-robot_state.dq[i])+coriolis[i];
				err_sum += (robot_state.q_d[i]-robot_state.q[i])*(robot_state.q_d[i]-robot_state.q[i]);
			}
			err_sum = sqrt(err_sum);
			//std::cout<<err_sum<<std::endl;

			printJ(robot_state.q,"q");
			printJ(robot_state.q_d,"q_d");
/*
			printJ(tq.tau_J,"tau");
			printJ(robot_state.q,"jt");

			if(	desired_recv == true){
				sim_controller.setDesired(robot_state.q_d,robot_state.dq_d);
				desired_recv = false;
			}
*/
			pubmsg = franka_core_msgs::JointCommand();
			pubmsg.header.stamp = ros::Time::now();
			pubmsg.header.frame_id = "";

			pubmsg.names.push_back("panda_joint1");
			pubmsg.names.push_back("panda_joint2");
			pubmsg.names.push_back("panda_joint3");
			pubmsg.names.push_back("panda_joint4");
			pubmsg.names.push_back("panda_joint5");
			pubmsg.names.push_back("panda_joint6");
			pubmsg.names.push_back("panda_joint7");
/*
			pubmsg.effort.push_back(tq.tau_J.at(0));
			pubmsg.effort.push_back(tq.tau_J.at(1));
			pubmsg.effort.push_back(tq.tau_J.at(2));
			pubmsg.effort.push_back(tq.tau_J.at(3));
			pubmsg.effort.push_back(tq.tau_J.at(4));
			pubmsg.effort.push_back(tq.tau_J.at(5));
			pubmsg.effort.push_back(tq.tau_J.at(6));
*/
			pubmsg.effort.push_back(tau.at(0));
			pubmsg.effort.push_back(tau.at(1));
			pubmsg.effort.push_back(tau.at(2));
			pubmsg.effort.push_back(tau.at(3));
			pubmsg.effort.push_back(tau.at(4));
			pubmsg.effort.push_back(tau.at(5));
			pubmsg.effort.push_back(tau.at(6));
			pubmsg.mode =pubmsg.TORQUE_MODE;
			pub.publish(pubmsg);
			
			joint_msg = sensor_msgs::JointState();
			joint_msg.header.stamp = ros::Time::now();

			joint_msg.name.push_back("panda_joint1");
			joint_msg.name.push_back("panda_joint2");
			joint_msg.name.push_back("panda_joint3");
			joint_msg.name.push_back("panda_joint4");
			joint_msg.name.push_back("panda_joint5");
			joint_msg.name.push_back("panda_joint6");
			joint_msg.name.push_back("panda_joint7");
			
			joint_msg.position.push_back(robot_state.q[0]);
			joint_msg.position.push_back(robot_state.q[1]);
			joint_msg.position.push_back(robot_state.q[2]);
			joint_msg.position.push_back(robot_state.q[3]);
			joint_msg.position.push_back(robot_state.q[4]);
			joint_msg.position.push_back(robot_state.q[5]);
			joint_msg.position.push_back(robot_state.q[6]);
			


			pub_joint.publish(joint_msg);
			ros::spinOnce();
			r.sleep();
		}

	

}


