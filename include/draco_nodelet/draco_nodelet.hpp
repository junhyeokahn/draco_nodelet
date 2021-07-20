#pragma once

#include <apptronik_srvs/Float32.h>
#include <cortex_framework/odometry_sensors/vn100_sensor.hpp>
#include <cortex_utils/debug_interfacer.hpp>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <rt_utils/synchronizer.hpp>

#include <cassert>

#include <Eigen/Dense>

#include <configuration.hpp>
#include <pnc/draco_pnc/draco_interface.hpp>
#include <utils/util.hpp>

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
  aptk::ctrl::VN100Sensor *vn_imu_;

  // service calls
  ros::ServiceServer mode_handler_;
  ros::ServiceServer pnc_handler_;
  ros::ServiceServer service_call_handler_;

  // timing
  double dt_ = 0.001; // TODO Read this from launch file
  int count_;

  // counting
  int n_joint_;
  int n_medulla_;
  int n_sensillum_;

  // electronic boards naming
  std::vector<std::string> axons_;
  std::vector<std::string> medullas_;
  std::vector<std::string> sensillums_;

  // actuating joint name corresponding to the axons
  std::vector<std::string> joint_names_;

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

  // PnC objects
  DracoInterface *pnc_interface_;
  DracoSensorData *pnc_sensor_data_;
  DracoCommand *pnc_command_;

  double contact_threshold_;

  YAML::Node cfg_;

  // flags
  bool b_pnc_alive_;

  // register miso and mosi topics to the placeholders
  void RegisterData();

  // copy placeholder data to sensor_data
  void CopyData();

  // copy command to placeholder
  void CopyCommand();

  // read yaml, and set gains and current limits
  void SetServiceCalls();

  // construct pnc
  void ConstructPnC();

  // destruct pnc
  void DestructPnC();

  // change mode
  // 0: Off
  // 1: JOINT_IMPEDANCE
  bool ModeHandler(apptronik_srvs::Float32::Request &req,
                   apptronik_srvs::Float32::Response &res);

  // enable or disable PnC
  // 0 : Destruct
  // 1 : Construct
  bool PnCHandler(apptronik_srvs::Float32::Request &req,
                  apptronik_srvs::Float32::Response &res);

  // set service call
  // 0 or 1 : Set service call with current yaml file
  bool ServiceCallHandler(apptronik_srvs::Float32::Request &req,
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
