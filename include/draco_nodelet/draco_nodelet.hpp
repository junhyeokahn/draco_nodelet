#pragma once

#include <apptronik_srvs/Float32.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <rt_utils/synchronizer.hpp>

#include <cassert>

#include <Eigen/Dense>

namespace draco_nodelet {
class DracoNodelet : public nodelet::Nodelet {
public:
  void spinThread();
  void onInit();
  DracoNodelet();
  ~DracoNodelet();

private:
  ros::NodeHandle nh_;
  boost::shared_ptr<aptk::comm::Synchronizer> sync_;
  boost::shared_ptr<boost::thread> spin_thread_;

  std::vector<std::string> slave_names_;
  std::string medulla_name_;

  int count_;
  int n_joint_;

  // data
  std::vector<float *> joint_positions_;
  std::vector<float *> joint_velocities_;

  // cmd
  std::vector<float *> joint_current_;

  void RegisterData();
  void CopyData();
  void CopyCommand();
  void SetServices();

  template <class SrvType>
  void CallGetService(const std::string &slave_name,
                      const std::string &srv_name, SrvType &srv_obj);
  template <class SrvType>
  void CallSetService(const std::string &slave_name,
                      const std::string &srv_name, SrvType &srv_obj);
};

} // namespace draco_nodelet
