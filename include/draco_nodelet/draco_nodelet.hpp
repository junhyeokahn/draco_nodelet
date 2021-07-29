#pragma once

#include <Eigen/Dense>
#include <cassert>

#include <nodelet/nodelet.h>
#include <ros/ros.h>

#include <apptronik_srvs/Float32.h>
#include <cortex_framework/odometry_sensors/vn100_sensor.hpp>
#include <cortex_utils/debug_interfacer.hpp>
#include <rt_utils/synchronizer.hpp>

#include <draco_nodelet/nodelet_configuration.hpp>

#include <configuration.hpp>
#if B_FIXED_CONFIGURATION
#include <pnc/fixed_draco_pnc/fixed_draco_interface.hpp>
#else
#include <pnc/draco_pnc/draco_interface.hpp>
#endif
#include <utils/util.hpp>

namespace draco_nodelet {

namespace control_mode {
constexpr int kOff = 0;
constexpr int kMotorCurrent = 1;
constexpr int kJointImpedance = 2;
} // namespace control_mode

class DracoNodelet : public nodelet::Nodelet {
public:
  void spinThread();
  void onInit();
  DracoNodelet();
  ~DracoNodelet();

private:
  int control_mode_;

  // RT kernel
  ros::NodeHandle nh_;
  boost::shared_ptr<aptk::comm::Synchronizer> sync_;
  boost::shared_ptr<boost::thread> spin_thread_;
  boost::shared_ptr<aptk::util::DebugInterfacer> debug_interface_;
  aptk::ctrl::VN100Sensor *vn_imu_;

  // service calls
  ros::ServiceServer mode_handler_;
  ros::ServiceServer pnc_handler_;
  ros::ServiceServer gain_limit_handler_;
  ros::ServiceServer imu_handler_;
  ros::ServiceServer fault_handler_;
  ros::ServiceServer fake_estop_handler_;
  ros::ServiceServer interrupt_handler_;

  // timing
  double pnc_dt_;
  double imu_servo_rate_;
  int count_;
  double sleep_time_;

  // counting
  int n_joint_;
  int n_medulla_;
  int n_sensillum_;

  // electronic boards naming
  std::vector<std::string> axons_;
  std::vector<std::string> lower_leg_axons_;
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
  std::vector<float *> ph_current_cmd_;
  std::vector<float *> ph_kp_;
  std::vector<float *> ph_kd_;
  float *ph_imu_quaternion_w_ned_, *ph_imu_quaternion_x_ned_,
      *ph_imu_quaternion_y_ned_, *ph_imu_quaternion_z_ned_;
  float *ph_imu_dvel_x_, *ph_imu_dvel_y_, *ph_imu_dvel_z_;
  float *ph_imu_ang_vel_x_, *ph_imu_ang_vel_y_, *ph_imu_ang_vel_z_;
  float *ph_rfoot_sg_, *ph_lfoot_sg_;

  // PnC objects
#if B_FIXED_CONFIGURATION
  FixedDracoInterface *pnc_interface_;
  FixedDracoSensorData *pnc_sensor_data_;
  FixedDracoCommand *pnc_command_;
#else
  DracoInterface *pnc_interface_;
  DracoSensorData *pnc_sensor_data_;
  DracoCommand *pnc_command_;
#endif

  double contact_threshold_;

  YAML::Node nodelet_cfg_;
  YAML::Node pnc_cfg_;

  Eigen::Vector3d world_la_offset_;
  std::vector<Eigen::Vector3d> world_la_offset_list_;
  int n_data_for_imu_initialize_;
  double vel_damping_;
  double damping_threshold_;

  // flags
  bool b_pnc_alive_;
  bool b_initializing_imu_;
  bool b_change_to_joint_impedance_mode_;
  bool b_change_to_off_mode_;
  bool b_change_to_motor_current_mode_;
  bool b_clear_faults_;
  bool b_destruct_pnc_;
  bool b_construct_pnc_;
  bool b_gains_limits_;
  bool b_fake_estop_released_;
  bool b_interrupt_;
  int interrupt_data_;
  std::string target_joint_; // for the purpose of moving joint one by one.

  // register miso and mosi topics to the placeholders
  void RegisterData();

  // copy placeholder data to sensor_data
  void CopyData();

  // copy command to placeholder
  void CopyCommand();

  // read yaml, and set gains and current limits
  void SetGainsAndLimits();

  // construct pnc
  void ConstructPnC();

  // destruct pnc
  void DestructPnC();

  // load config yaml files
  void LoadConfigFile();

  // process service calls based on the flag variable
  void ProcessServiceCalls();

  // change mode
  // 0: Off
  // 1: CURRENT
  // 2: JOINT_IMPEDANCE
  bool ModeHandler(apptronik_srvs::Float32::Request &req,
                   apptronik_srvs::Float32::Response &res);

  // enable or disable PnC
  // 0 : Destruct
  // 1 : Construct
  bool PnCHandler(apptronik_srvs::Float32::Request &req,
                  apptronik_srvs::Float32::Response &res);

  // enable or disable PnC
  // 0 or 1 : Clear the faults
  bool FaultHandler(apptronik_srvs::Float32::Request &req,
                    apptronik_srvs::Float32::Response &res);

  // reinitialize imu
  // 0 or 1 : Initialize
  bool IMUHandler(apptronik_srvs::Float32::Request &req,
                  apptronik_srvs::Float32::Response &res);

  // set gains and limits
  // 0 or 1 : Set service call with current yaml file
  bool GainsAndLimitsHandler(apptronik_srvs::Float32::Request &req,
                             apptronik_srvs::Float32::Response &res);

  // interrupt handler
  // fixed draco configuration
  //     4: swing left leg
  //     6: swing right leg
  bool InterruptHandler(apptronik_srvs::Float32::Request &req,
                        apptronik_srvs::Float32::Response &res);

  // set gains and limits
  // 0: Fake estop enabled
  // 1 : Fake estop disabled
  bool FakeEstopHandler(apptronik_srvs::Float32::Request &req,
                        apptronik_srvs::Float32::Response &res);

  // turn off the motors
  void TurnOffMotors();
  // turn on the motors to joint impedance mode
  void TurnOnJointImpedance();
  // turn on the motors to motor current mode
  void TurnOnMotorCurrent();
  // clear faults on motors
  void ClearFaults();
  // set safe commands
  void SetSafeCommand();

  template <class SrvType>
  void CallGetService(const std::string &slave_name,
                      const std::string &srv_name, SrvType &srv_obj);
  template <class SrvType>
  void CallSetService(const std::string &slave_name,
                      const std::string &srv_name, SrvType &srv_obj);
};

} // namespace draco_nodelet
