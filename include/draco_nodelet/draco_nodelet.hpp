#pragma once

#include <apptronik_srvs/Float32.h>
#include <cortex_utils/debug_interfacer.hpp>
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
  // RT kernel
  ros::NodeHandle nh_;
  boost::shared_ptr<aptk::comm::Synchronizer> sync_;
  boost::shared_ptr<boost::thread> spin_thread_;
  boost::shared_ptr<aptk::util::DebugInterfacer> interfacer_; // for plot

  // service calls
  ros::ServiceServer mode_handler_;
  ros::ServiceServer pnc_handler_;

  // counting
  int count_;
  int n_joint_;
  int n_actuator_;
  int n_medulla_;
  int n_sensillum_;

  // electronic boards naming
  std::vector<std::string> axons_;
  std::vector<std::string> medullas_;
  std::vector<std::string> sensillums_;

  // placeholders for data coming through ecat communication
  // this placeholders shares the same order with axons_
  std::vector<float *> ph_joint_positions_data_;
  std::vector<float *> ph_joint_velocities_data_;
  std::vector<float *> ph_joint_positions_cmd_;
  std::vector<float *> ph_joint_velocities_cmd_;
  std::vector<float *> ph_joint_efforts_cmd_;
  float *ph_imu_quaternion_w_ned_, *ph_imu_quaternion_x_ned_,
      *ph_imu_quaternion_y_ned_, *ph_imu_quaternion_z_ned_;
  float *ph_imu_dvel_x_, *ph_imu_dvel_y_, *ph_imu_dvel_z_;
  float *ph_imu_ang_vel_x_, *ph_imu_ang_vel_y_, *ph_imu_ang_vel_z_;
  float *ph_rfoot_sg_, *ph_lfoot_sg_;

  // flags
  bool b_pnc_alive_;
  bool b_online_plot_;

  // register miso and mosi topics to the placeholders
  void RegisterData();
  // copy placeholder data to sensor_data
  void CopyData();
  // copy command to placeholder
  void CopyCommand();
  // read yaml, and set gains
  void SetImpedanceGains();
  // read yaml, and set current limits
  void SetCurrentLimits();
  // construct pnc
  void ConstructPnC();
  // destruct pnc
  void DestructPnC();
  // change mode
  bool ModeHandler(apptronik_srvs::Float32::Request &req,
                   apptronik_srvs::Float32::Response &res);
  // enable or disable PnC
  bool PnCHandler(apptronik_srvs::Float32::Request &req,
                  apptronik_srvs::Float32::Response &res);

  // turn off the motors
  void TurnOffMotors();
  // turn on the motors to joint impedance mode
  void TurnOnMotors();
  // clear faults on motors
  void ClearFaults();

  template <class SrvType>
  void CallGetService(const std::string &slave_name,
                      const std::string &srv_name, SrvType &srv_obj);
  template <class SrvType>
  void CallSetService(const std::string &slave_name,
                      const std::string &srv_name, SrvType &srv_obj);
};

} // namespace draco_nodelet
