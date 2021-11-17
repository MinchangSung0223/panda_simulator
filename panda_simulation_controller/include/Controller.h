#ifndef __CONTROLLER_H__
#define __CONTROLLER_H__
#include <iostream>
#include <stdio.h>
#include <math.h>
#include <vector>
#include <array>
#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/model.h>
#include <franka/rate_limiting.h>
typedef std::array<double,7> J;

class Controller {		
	public:	
		Controller(size_t dq_filter_size,const std::array<double, 7>& K_P, const std::array<double, 7>& K_D);
		inline franka::Torques step(const franka::RobotState& state) {
			updateDQFilter(state);
			std::array<double, 7> tau_J_d;  // NOLINT(readability-identifier-naming)
			for (size_t i = 0; i < 7; i++) {
			  //tau_J_d[i] = K_P_[i] * (q_d_[i] - state.q[i]) + K_D_[i] * (dq_d_[i] - getDQFiltered(i))+coriolis[i];
			  tau_J_d[i] = K_P_[i] * (q_d_[i] - state.q[i]) + K_D_[i] * (dq_d_[i] - getDQFiltered(i));
			}
			return tau_J_d;
		}
  		void updateDQFilter(const franka::RobotState& state);
  		double getDQFiltered(size_t index) const ;
  		void printJ(J input,char* str);
  		void setDesired(std::array<double, 7> q, std::array<double, 7> dq);
	private:
		size_t dq_current_filter_position_;
		size_t dq_filter_size_;
		const std::array<double, 7> K_P_;  // NOLINT(readability-identifier-naming)
		const std::array<double, 7> K_D_;  // NOLINT(readability-identifier-naming)
		std::array<double, 7> dq_d_;
		std::array<double, 7> q_d_;
		std::unique_ptr<double[]> dq_buffer_;	

};
#endif


