#include "Controller.h"

  Controller::Controller(size_t dq_filter_size,
             const std::array<double, 7>& K_P,  // NOLINT(readability-identifier-naming)
             const std::array<double, 7>& K_D)  // NOLINT(readability-identifier-naming)
      : dq_current_filter_position_(0), dq_filter_size_(dq_filter_size), K_P_(K_P), K_D_(K_D) {
    std::fill(dq_d_.begin(), dq_d_.end(), 0);
    dq_buffer_ = std::make_unique<double[]>(dq_filter_size_ * 7);
    std::fill(&dq_buffer_.get()[0], &dq_buffer_.get()[dq_filter_size_ * 7], 0);
  }

  void Controller::updateDQFilter(const franka::RobotState& state) {
    for (size_t i = 0; i < 7; i++) {
      dq_buffer_.get()[dq_current_filter_position_ * 7 + i] = state.dq[i];
    }
    dq_current_filter_position_ = (dq_current_filter_position_ + 1) % dq_filter_size_;
  }
  double Controller::getDQFiltered(size_t index) const {
    double value = 0;
    for (size_t i = index; i < 7 * dq_filter_size_; i += 7) {
      value += dq_buffer_.get()[i];
    }
    return value / dq_filter_size_;
  }
  void Controller::printJ(J input,char* str){
  std::cout<<"\t"<<str<<" : ";
  for(int i = 0;i<6;i++)
    std::cout<<input.at(i)<<",";
  std::cout<<input.at(6)<<std::endl;
 }
  void Controller::setDesired(std::array<double, 7> q, std::array<double, 7> dq){
    for(int i =0;i<7;i++){
      dq_d_[i] = dq[i];
      q_d_[i] = q[i];
    }
    //printJ(dq_d_,"dq_d_");
    //printJ(q_d_,"q_d_");

  }