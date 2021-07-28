#include <stdexcept>

#include <draco_nodelet/draco_nodelet.hpp>

using namespace aptk::comm;
using namespace aptk::ctrl;
namespace draco_nodelet {
DracoNodelet::DracoNodelet() {

  this->LoadConfigFile();

  imu_servo_rate_ = util::ReadParameter<double>(nodelet_cfg_, "imu_servo_rate");
  pnc_dt_ = util::ReadParameter<double>(pnc_cfg_, "servo_dt");

  axons_ = {"Neck_Pitch",    "R_Hip_IE",      "R_Hip_AA",      "R_Hip_FE",
            "R_Knee_FE",     "R_Ankle_FE",    "R_Ankle_IE",    "L_Hip_IE",
            "L_Hip_AA",      "L_Hip_FE",      "L_Knee_FE",     "L_Ankle_FE",
            "L_Ankle_IE",    "L_Shoulder_FE", "L_Shoulder_AA", "L_Shoulder_IE",
            "L_Elbow",       "L_Wrist_Roll",  "L_Wrist_Pitch", "R_Shoulder_FE",
            "R_Shoulder_AA", "R_Shoulder_IE", "R_Elbow",       "R_Wrist_Roll",
            "R_Wrist_Pitch"};
  medullas_ = {"Medulla", "Medulla_V4"};
  sensillums_ = {"Sensillum_v2"};

  joint_names_ = {
      "neck_pitch",    "r_hip_ie",      "r_hip_aa",      "r_hip_fe",
      "r_knee_fe",     "r_ankle_fe",    "r_ankle_ie",    "l_hip_ie",
      "l_hip_aa",      "l_hip_fe",      "l_knee_fe",     "l_ankle_fe",
      "l_ankle_ie",    "l_shoulder_fe", "l_shoulder_aa", "l_shoulder_ie",
      "l_elbow_fe",    "l_wrist_ps",    "l_wrist_pitch", "r_shoulder_fe",
      "r_shoulder_aa", "r_shoulder_ie", "r_elbow_fe",    "r_wrist_ps",
      "r_wrist_pitch"};

  control_mode_ = control_mode::kOff;

  count_ = 0;
  sleep_time_ = 0.5;
  n_joint_ = axons_.size();
  n_medulla_ = medullas_.size();
  n_sensillum_ = sensillums_.size();

  ph_joint_positions_data_.resize(n_joint_);
  ph_kp_.resize(n_joint_);
  ph_kd_.resize(n_joint_);
  ph_joint_velocities_data_.resize(n_joint_);
  ph_joint_positions_cmd_.resize(n_joint_);
  ph_joint_velocities_cmd_.resize(n_joint_);
  ph_joint_efforts_cmd_.resize(n_joint_);
  ph_current_cmd_.resize(n_joint_);

  b_pnc_alive_ = true;
#if B_FIXED_CONFIGURATION
  pnc_interface_ = new FixedDracoInterface(false);
  pnc_sensor_data_ = new FixedDracoSensorData();
  pnc_command_ = new FixedDracoCommand();
#else
  pnc_interface_ = new DracoInterface(false);
  pnc_sensor_data_ = new DracoSensorData();
  pnc_command_ = new DracoCommand();
#endif
  b_initializing_imu_ = true;
  world_la_offset_list_.clear();
  b_change_to_off_mode_ = false;
  b_change_to_motor_current_mode_ = false;
  b_change_to_joint_impedance_mode_ = false;
  b_clear_faults_ = false;
  b_destruct_pnc_ = false;
  b_construct_pnc_ = false;
  b_gains_limits_ = false;
  b_fake_estop_released_ = false;

  world_la_offset_.setZero();
}

DracoNodelet::~DracoNodelet() {
  spin_thread_->join();
  for (int i = 0; i < n_joint_; ++i) {
    delete ph_joint_positions_data_[i];
    delete ph_kp_[i];
    delete ph_kd_[i];
    delete ph_joint_velocities_data_[i];
    delete ph_joint_positions_cmd_[i];
    delete ph_joint_velocities_cmd_[i];
    delete ph_joint_efforts_cmd_[i];
    delete ph_current_cmd_[i];
  }
  delete ph_imu_quaternion_w_ned_;
  delete ph_imu_quaternion_x_ned_;
  delete ph_imu_quaternion_y_ned_;
  delete ph_imu_quaternion_z_ned_;
  delete ph_imu_dvel_x_;
  delete ph_imu_dvel_y_;
  delete ph_imu_dvel_z_;
  delete ph_imu_ang_vel_x_;
  delete ph_imu_ang_vel_y_;
  delete ph_imu_ang_vel_z_;
  delete ph_rfoot_sg_;
  delete ph_lfoot_sg_;

  delete vn_imu_;
}

void DracoNodelet::onInit() {
  nh_ = getNodeHandle();
  spin_thread_.reset(
      new boost::thread(boost::bind(&DracoNodelet::spinThread, this)));

  fault_handler_ =
      nh_.advertiseService("/fault_handler", &DracoNodelet::FaultHandler, this);
  mode_handler_ =
      nh_.advertiseService("/mode_handler", &DracoNodelet::ModeHandler, this);
  pnc_handler_ =
      nh_.advertiseService("/pnc_handler", &DracoNodelet::PnCHandler, this);
  gain_limit_handler_ = nh_.advertiseService(
      "/gains_limits_handler", &DracoNodelet::GainsAndLimitsHandler, this);
  imu_handler_ =
      nh_.advertiseService("/imu_handler", &DracoNodelet::IMUHandler, this);
  fake_estop_handler_ = nh_.advertiseService(
      "/fake_estop_handler", &DracoNodelet::FakeEstopHandler, this);
}

void DracoNodelet::spinThread() {
  sync_.reset(new aptk::comm::Synchronizer(true, "draco_nodelet"));
  sync_->connect();
  debug_interface_.reset(new aptk::util::DebugInterfacer(
      "draco", sync_->getNodeHandle(), sync_->getLogger()));

  vn_imu_ = new VN100Sensor("imu", imu_servo_rate_);
  vn_imu_->addDebugInterfaces(debug_interface_);

  aptk::comm::enableRT(5, 2);

  RegisterData();

  SetGainsAndLimits();

  TurnOffMotors();

  ClearFaults();

  // main control loop
  while (sync_->ok()) {
    // wait for bus transaction
    sync_->awaitNextControl();
    // handle service calls based on the flag variables
    ProcessServiceCalls();
    // copy data
    CopyData();
    if (sync_->printIndicatedFaults() && (!b_fake_estop_released_)) {
      // faulted
      SetSafeCommand();
    } else {
      if (control_mode_ == control_mode::kMotorCurrent) {
        // do nothing while going through initial ramp
        SetSafeCommand();
      } else {
        // compute command from pnc
        pnc_interface_->getCommand(pnc_sensor_data_, pnc_command_);
        CopyCommand();
      }
    }

    // indicate that we're done
    sync_->finishControl();

    ++count_;
    debug_interface_->updateDebug();
  }

  sync_->awaitShutdownComplete();
}

void DracoNodelet::SetSafeCommand() {
  for (int i = 0; i < n_joint_; ++i) {
    *(ph_joint_positions_cmd_[i]) = *(ph_joint_positions_data_[i]);
    *(ph_joint_velocities_cmd_[i]) = 0.;
    *(ph_joint_efforts_cmd_[i]) = 0.;
    *(ph_current_cmd_[i]) = 0.;
  }
}

void DracoNodelet::ProcessServiceCalls() {

  if (b_change_to_off_mode_) {
    TurnOffMotors();
    b_change_to_off_mode_ = false;
  }
  if (b_change_to_motor_current_mode_) {
    TurnOnMotorCurrent();
    b_change_to_motor_current_mode_ = false;
  }
  if (b_change_to_joint_impedance_mode_) {
    TurnOnJointImpedance();
    b_change_to_joint_impedance_mode_ = false;
  }
  if (b_clear_faults_) {
    ClearFaults();
    b_clear_faults_ = false;
  }
  if (b_construct_pnc_) {
    ConstructPnC();
    b_construct_pnc_ = false;
  }
  if (b_destruct_pnc_) {
    DestructPnC();
    b_destruct_pnc_ = false;
  }
  if (b_gains_limits_) {
    SetGainsAndLimits();
    b_gains_limits_ = false;
  }
}

void DracoNodelet::RegisterData() {
  std::cout << "DracoNodelet::RegisterData()" << std::endl;
  int r_ankle_ie_idx(0), l_ankle_ie_idx(0), r_ankle_fe_idx(0),
      l_ankle_fe_idx(0);
  for (int i = 0; i < n_joint_; ++i) {
    // register encoder data
    ph_joint_positions_data_[i] = new float(0.);
    sync_->registerMISOPtr(ph_joint_positions_data_[i],
                           "js__joint__position__rad", axons_[i], false);
    ph_joint_velocities_data_[i] = new float(0.);
    sync_->registerMISOPtr(ph_joint_velocities_data_[i],
                           "js__joint__velocity__radps", axons_[i], false);

    // register commands for joint impedance control mode
    ph_joint_positions_cmd_[i] = new float(0.);
    sync_->registerMOSIPtr(ph_joint_positions_cmd_[i],
                           "cmd__joint__position__rad", axons_[i], false);
    ph_joint_velocities_cmd_[i] = new float(0.);
    sync_->registerMOSIPtr(ph_joint_velocities_cmd_[i],
                           "cmd__joint__velocity__radps", axons_[i], false);
    ph_joint_efforts_cmd_[i] = new float(0.);
    sync_->registerMOSIPtr(ph_joint_efforts_cmd_[i], "cmd__joint__effort__nm",
                           axons_[i], false);
    ph_current_cmd_[i] = new float(0.);
    sync_->registerMOSIPtr(ph_current_cmd_[i], "cmd__motor__effort__a",
                           axons_[i], false);

    ph_kp_[i] = new float(0.);
    sync_->registerMOSIPtr(ph_kp_[i], "gain__joint_impedance_kp__nmprad",
                           axons_[i], false);
    ph_kd_[i] = new float(0.);
    sync_->registerMOSIPtr(ph_kd_[i], "gain__joint_impedance_kd__nmsprad",
                           axons_[i], false);

    // for linkage table
    if (axons_[i] == "R_Ankle_FE") {
      sync_->registerMOSIPtr(ph_joint_positions_data_[i],
                             "ext__joint_position__rad", "R_Ankle_IE", false);
    }
    if (axons_[i] == "L_Ankle_FE") {
      sync_->registerMOSIPtr(ph_joint_positions_data_[i],
                             "ext__joint_position__rad", "L_Ankle_IE", false);
    }
  }

  ph_imu_quaternion_w_ned_ = new float(0.);
  sync_->registerMISOPtr(ph_imu_quaternion_w_ned_, "IMU__quaternion_w__mps2",
                         sensillums_[0], false);
  ph_imu_quaternion_x_ned_ = new float(0.);
  sync_->registerMISOPtr(ph_imu_quaternion_x_ned_, "IMU__quaternion_x__mps2",
                         sensillums_[0], false);
  ph_imu_quaternion_y_ned_ = new float(0.);
  sync_->registerMISOPtr(ph_imu_quaternion_y_ned_, "IMU__quaternion_y__mps2",
                         sensillums_[0], false);
  ph_imu_quaternion_z_ned_ = new float(0.);
  sync_->registerMISOPtr(ph_imu_quaternion_z_ned_, "IMU__quaternion_z__mps2",
                         sensillums_[0], false);
  ph_imu_dvel_x_ = new float(0.);
  sync_->registerMISOPtr(ph_imu_dvel_x_, "IMU__dVel_x__rad", sensillums_[0],
                         false);
  ph_imu_dvel_y_ = new float(0.);
  sync_->registerMISOPtr(ph_imu_dvel_y_, "IMU__dVel_y__rad", sensillums_[0],
                         false);
  ph_imu_dvel_z_ = new float(0.);
  sync_->registerMISOPtr(ph_imu_dvel_z_, "IMU__dVel_z__rad", sensillums_[0],
                         false);
  ph_imu_ang_vel_x_ = new float(0.);
  sync_->registerMISOPtr(ph_imu_ang_vel_x_, "IMU__comp_angularRate_x__radps",
                         sensillums_[0], false);
  ph_imu_ang_vel_y_ = new float(0.);
  sync_->registerMISOPtr(ph_imu_ang_vel_y_, "IMU__comp_angularRate_y__radps",
                         sensillums_[0], false);
  ph_imu_ang_vel_z_ = new float(0.);
  sync_->registerMISOPtr(ph_imu_ang_vel_z_, "IMU__comp_angularRate_z__radps",
                         sensillums_[0], false);
  ph_rfoot_sg_ = new float(0.);
  sync_->registerMISOPtr(ph_rfoot_sg_, "foot__sg__x", "R_Ankle_IE", false);
  ph_lfoot_sg_ = new float(0.);
  sync_->registerMISOPtr(ph_lfoot_sg_, "foot__sg__x", "L_Ankle_IE", false);
}

void DracoNodelet::CopyData() {
#if B_FIXED_CONFIGURATION
  // skip updating imu processing and contact sensor processing
#else
  // Process IMU
  uint16_t timestamp = sync_->getBusTimeNS();
  Eigen::Quaternion<double> local_ned_q_frame(
      *ph_imu_quaternion_w_ned_, *ph_imu_quaternion_x_ned_,
      *ph_imu_quaternion_y_ned_, *ph_imu_quaternion_z_ned_);
  Eigen::Vector3d frameAVframe(*ph_imu_ang_vel_x_, *ph_imu_ang_vel_y_,
                               *ph_imu_ang_vel_z_);
  Eigen::Vector3d local_nedLAframe__delta_vel(*ph_imu_dvel_x_, *ph_imu_dvel_y_,
                                              *ph_imu_dvel_z_);
  uint16_t status = 0; // not used

  if (b_initializing_imu_) {
    world_la_offset_list_.push_back(local_nedLAframe__delta_vel /
                                    imu_servo_rate_);
    if (world_la_offset_list_.size() == n_data_for_imu_initialize_) {
      Eigen::Vector3d sum(0., 0., 0.);
      for (int i = 0; i < n_data_for_imu_initialize_; ++i) {
        sum += world_la_offset_list_[i];
      }
      world_la_offset_ = sum / n_data_for_imu_initialize_;
      world_la_offset_list_.clear();
      b_initializing_imu_ = false;
      std::cout << "[IMU initialized]" << std::endl;
      std::cout << "world_la_offset_: " << world_la_offset_.transpose()
                << std::endl;
    }
  }
  vn_imu_->processData(timestamp, local_ned_q_frame, frameAVframe,
                       local_nedLAframe__delta_vel, status, world_la_offset_,
                       vel_damping_, damping_threshold_);
  // Set imu data
  Eigen::Isometry3d worldTframe;
  Eigen::Vector6d worldSVframe;

  vn_imu_->estimateTwist(worldSVframe);
  pnc_sensor_data_->imu_frame_vel = worldSVframe;

  vn_imu_->estimateTransform(worldTframe);
  pnc_sensor_data_->imu_frame_iso.setIdentity();
  pnc_sensor_data_->imu_frame_iso.block(0, 0, 3, 3) = worldTframe.linear();
  pnc_sensor_data_->imu_frame_iso.block(0, 3, 3, 1) = worldTframe.translation();

  // Set contact bool
  if (*(ph_rfoot_sg_) > contact_threshold_) {
    pnc_sensor_data_->b_rf_contact = true;
  } else {
    pnc_sensor_data_->b_rf_contact = false;
  }
  if (*(ph_lfoot_sg_) > contact_threshold_) {
    pnc_sensor_data_->b_lf_contact = true;
  } else {
    pnc_sensor_data_->b_lf_contact = false;
  }
#endif

  // Set encoder data
  for (int i = 0; i < n_joint_; ++i) {
    if (joint_names_[i] == "r_knee_fe") {
      // r_knee_fe_jp
      pnc_sensor_data_->joint_positions["r_knee_fe_jp"] =
          static_cast<double>(*(ph_joint_positions_data_[i])) / 2.;
      pnc_sensor_data_->joint_velocities["r_knee_fe_jp"] =
          static_cast<double>(*(ph_joint_velocities_data_[i])) / 2.;
      // r_knee_fe_jd
      pnc_sensor_data_->joint_positions["r_knee_fe_jd"] =
          static_cast<double>(*(ph_joint_positions_data_[i])) / 2.;
      pnc_sensor_data_->joint_velocities["r_knee_fe_jd"] =
          static_cast<double>(*(ph_joint_velocities_data_[i])) / 2.;
    } else if (joint_names_[i] == "l_knee_fe") {
      // l_knee_fe_jp
      pnc_sensor_data_->joint_positions["l_knee_fe_jp"] =
          static_cast<double>(*(ph_joint_positions_data_[i])) / 2.;
      pnc_sensor_data_->joint_velocities["l_knee_fe_jp"] =
          static_cast<double>(*(ph_joint_velocities_data_[i])) / 2.;
      // l_knee_fe_jd
      pnc_sensor_data_->joint_positions["l_knee_fe_jd"] =
          static_cast<double>(*(ph_joint_positions_data_[i])) / 2.;
      pnc_sensor_data_->joint_velocities["l_knee_fe_jd"] =
          static_cast<double>(*(ph_joint_velocities_data_[i])) / 2.;
    } else {
      pnc_sensor_data_->joint_positions[joint_names_[i]] =
          static_cast<double>(*(ph_joint_positions_data_[i]));
      pnc_sensor_data_->joint_velocities[joint_names_[i]] =
          static_cast<double>(*(ph_joint_velocities_data_[i]));
    }
  }
}

void DracoNodelet::CopyCommand() {
  for (int i = 0; i < n_joint_; ++i) {
    if (joint_names_[i] == "r_knee_fe") {
      *(ph_joint_positions_cmd_[i]) = static_cast<float>(
          pnc_command_->joint_positions["r_knee_fe_jd"] * 2.);
      *(ph_joint_velocities_cmd_[i]) = static_cast<float>(
          pnc_command_->joint_velocities["r_knee_fe_jd"] * 2.);
      *(ph_joint_efforts_cmd_[i]) =
          static_cast<float>(pnc_command_->joint_torques["r_knee_fe_jd"] / 2.);
    } else if (joint_names_[i] == "l_knee_fe") {
      *(ph_joint_positions_cmd_[i]) = static_cast<float>(
          pnc_command_->joint_positions["l_knee_fe_jd"] * 2.);
      *(ph_joint_velocities_cmd_[i]) = static_cast<float>(
          pnc_command_->joint_velocities["l_knee_fe_jd"] * 2.);
      *(ph_joint_efforts_cmd_[i]) =
          static_cast<float>(pnc_command_->joint_torques["l_knee_fe_jd"] / 2.);
    } else {
      *(ph_joint_positions_cmd_[i]) =
          static_cast<float>(pnc_command_->joint_positions[joint_names_[i]]);
      *(ph_joint_velocities_cmd_[i]) =
          static_cast<float>(pnc_command_->joint_velocities[joint_names_[i]]);
      *(ph_joint_efforts_cmd_[i]) =
          static_cast<float>(pnc_command_->joint_torques[joint_names_[i]]);
    }
  }
}

void DracoNodelet::SetGainsAndLimits() {
  this->LoadConfigFile();
  bool b_conservative =
      util::ReadParameter<bool>(nodelet_cfg_["service_call"], "conservative");
  apptronik_srvs::Float32 srv_float_kp;
  apptronik_srvs::Float32 srv_float_kd;
  apptronik_srvs::Float32 srv_float_current_limit;

  for (int i = 0; i < n_joint_; ++i) {
    if (b_conservative) {
      *(ph_kp_[i]) = util::ReadParameter<float>(
          nodelet_cfg_["service_call"][joint_names_[i]], "weak_kp");
      *(ph_kd_[i]) = util::ReadParameter<float>(
          nodelet_cfg_["service_call"][joint_names_[i]], "weak_kd");
      srv_float_kp.request.set_data = util::ReadParameter<float>(
          nodelet_cfg_["service_call"][joint_names_[i]], "weak_kp");
      srv_float_kd.request.set_data = util::ReadParameter<float>(
          nodelet_cfg_["service_call"][joint_names_[i]], "weak_kd");
      srv_float_current_limit.request.set_data = util::ReadParameter<float>(
          nodelet_cfg_["service_call"][joint_names_[i]], "weak_current_limit");
      // nodelet_cfg_["service_call"][joint_names_[i]], "current_limit");
    } else {
      *(ph_kp_[i]) = util::ReadParameter<float>(
          nodelet_cfg_["service_call"][joint_names_[i]], "kp");
      *(ph_kd_[i]) = util::ReadParameter<float>(
          nodelet_cfg_["service_call"][joint_names_[i]], "kd");
      srv_float_kp.request.set_data = util::ReadParameter<float>(
          nodelet_cfg_["service_call"][joint_names_[i]], "kp");
      srv_float_kd.request.set_data = util::ReadParameter<float>(
          nodelet_cfg_["service_call"][joint_names_[i]], "kd");
      srv_float_current_limit.request.set_data = util::ReadParameter<float>(
          nodelet_cfg_["service_call"][joint_names_[i]], "current_limit");
    }
    CallSetService(axons_[i], "Control__Joint__Impedance__KP", srv_float_kp);
    sleep(sleep_time_);
    CallSetService(axons_[i], "Control__Joint__Impedance__KD", srv_float_kd);
    sleep(sleep_time_);
    CallSetService(axons_[i], "Limits__Motor__Effort__Saturate__Relative_val",
                   srv_float_current_limit);
    sleep(sleep_time_);
  }
}

void DracoNodelet::ConstructPnC() {
  if (!b_pnc_alive_) {
#if B_FIXED_CONFIGURATION
    pnc_interface_ = new FixedDracoInterface(false);
#else
    pnc_interface_ = new DracoInterface(false);
#endif
    b_pnc_alive_ = true;
  } else {
    std::cout << "[[[Warning]]] PnC is already alive" << std::endl;
  }
}

void DracoNodelet::DestructPnC() {
  if (b_pnc_alive_) {
    delete pnc_interface_;
    b_pnc_alive_ = false;
  } else {
    std::cout << "[[[Warning]]] PnC is already desturcted" << std::endl;
  }
}

bool DracoNodelet::FaultHandler(apptronik_srvs::Float32::Request &req,
                                apptronik_srvs::Float32::Response &res) {
  double data = static_cast<double>(req.set_data);
  if (data == 0) {
    std::cout << "[[[Clear the faults]]]" << std::endl;
    b_clear_faults_ = true;
    return true;
  } else if (data == 1) {
    std::cout << "[[[Clear the faults]]]" << std::endl;
    b_clear_faults_ = true;
  } else {
    std::cout << "[[[Warning]]] Wrong Data Received for ModeHandler()"
              << std::endl;
    return false;
  }
}

bool DracoNodelet::ModeHandler(apptronik_srvs::Float32::Request &req,
                               apptronik_srvs::Float32::Response &res) {
  double data = static_cast<double>(req.set_data);
  if (data == 0) {
    std::cout << "[[[Change to Off Mode]]]" << std::endl;
    b_change_to_off_mode_ = true;
    control_mode_ = control_mode::kOff;
    return true;
  } else if (data == 1) {
    std::cout << "[[[Change to MOTOR_CURRENT Mode]]]" << std::endl;
    b_change_to_motor_current_mode_ = true;
    control_mode_ = control_mode::kMotorCurrent;
  } else if (data == 2) {
    std::cout << "[[[Change to JOINT_IMPEDANCE Mode]]]" << std::endl;
    b_change_to_joint_impedance_mode_ = true;
    control_mode_ = control_mode::kJointImpedance;
    return true;
  } else {
    std::cout << "[[[Warning]]] Wrong Data Received for ModeHandler()"
              << std::endl;
    return false;
  }
}

bool DracoNodelet::PnCHandler(apptronik_srvs::Float32::Request &req,
                              apptronik_srvs::Float32::Response &res) {
  double data = static_cast<double>(req.set_data);
  if (data == 0) {
    std::cout << "[[[Destruct PnC]]]" << std::endl;
    b_destruct_pnc_ = true;
    return true;
  } else if (data == 1) {
    std::cout << "[[[Construct PnC]]]" << std::endl;
    b_construct_pnc_ = true;
    return true;
  } else {
    std::cout << "[[[Warning]]] Wrong Data Received for PnCHandler()"
              << std::endl;
    return false;
  }
}

bool DracoNodelet::GainsAndLimitsHandler(
    apptronik_srvs::Float32::Request &req,
    apptronik_srvs::Float32::Response &res) {
  double data = static_cast<double>(req.set_data);
  if (data == 0) {
    std::cout << "[[[Reset Gains and Current Limits]]]" << std::endl;
    b_gains_limits_ = true;
    return true;
  } else if (data == 1) {
    std::cout << "[[[Reset Gains and Current Limits]]]" << std::endl;
    b_gains_limits_ = true;
    return true;
  } else {
    std::cout << "[[[Warning]]] Wrong Data Received for GainsAndLimitsHandler()"
              << std::endl;
    return false;
  }
}

bool DracoNodelet::IMUHandler(apptronik_srvs::Float32::Request &req,
                              apptronik_srvs::Float32::Response &res) {
  double data = static_cast<double>(req.set_data);
  if (data == 0) {
    std::cout << "[[[Reinitialize IMU]]]" << std::endl;
    b_initializing_imu_ = true;
    world_la_offset_list_.clear();
    return true;
  } else if (data == 1) {
    std::cout << "[[[Reinitialize IMU]]]" << std::endl;
    b_initializing_imu_ = true;
    world_la_offset_list_.clear();
    return true;
  } else {
    std::cout << "[[[Warning]]] Wrong Data Received for IMUHandler()"
              << std::endl;
    return false;
  }
}

bool DracoNodelet::FakeEstopHandler(apptronik_srvs::Float32::Request &req,
                                    apptronik_srvs::Float32::Response &res) {
  double data = static_cast<double>(req.set_data);
  if (data == 0) {
    std::cout << "[[[Fake Estop Enabled]]]" << std::endl;
    b_fake_estop_released_ = false;
    return true;
  } else if (data == 1) {
    std::cout << "[[[Fake Estop Disabled]]]" << std::endl;
    b_fake_estop_released_ = true;
    return true;
  } else {
    std::cout << "[[[Warning]]] Wrong Data Received for IMUHandler()"
              << std::endl;
    return false;
  }
}

void DracoNodelet::TurnOffMotors() {
  for (int i = 0; i < n_joint_; ++i) {
    sync_->changeMode("OFF", axons_[i]);
    sleep(sleep_time_);
  }
}

void DracoNodelet::TurnOnJointImpedance() {
  if (b_pnc_alive_) {
    for (int i = 0; i < n_joint_; ++i) {
      sync_->changeMode("JOINT_IMPEDANCE", axons_[i]);
      sleep(sleep_time_);
    }
  } else {
    std::cout << "PnC is not alive. Construct PnC before change the mode"
              << std::endl;
  }
}

void DracoNodelet::TurnOnMotorCurrent() {
  for (int i = 0; i < n_joint_; ++i) {
    sync_->changeMode("MOTOR_CURRENT", axons_[i]);
    sleep(sleep_time_);
  }
}

void DracoNodelet::ClearFaults() {
  for (int i = 0; i < n_joint_; ++i) {
    sync_->clearFaults(axons_[i]);
    sleep(sleep_time_);
  }
}

void DracoNodelet::LoadConfigFile() {
#if B_FIXED_CONFIGURATION
  nodelet_cfg_ = YAML::LoadFile(THIS_COM "config/fixed_draco/nodelet.yaml");
  pnc_cfg_ = YAML::LoadFile(THIS_COM "config/fixed_draco/pnc.yaml");
#else
  nodelet_cfg_ = YAML::LoadFile(THIS_COM "config/draco/nodelet.yaml");
  pnc_cfg_ = YAML::LoadFile(THIS_COM "config/draco/pnc.yaml");
#endif

  target_joint_ =
      util::ReadParameter<std::string>(nodelet_cfg_, "target_joint");

  // TODO : tune this parameters
  vel_damping_ = util::ReadParameter<double>(nodelet_cfg_, "vel_damping");
  // TODO : tune this parameters
  damping_threshold_ =
      util::ReadParameter<double>(nodelet_cfg_, "damping_threshold");
  n_data_for_imu_initialize_ =
      util::ReadParameter<int>(nodelet_cfg_, "n_data_for_imu_initialize");

  contact_threshold_ =
      util::ReadParameter<double>(nodelet_cfg_, "contact_threshold");
}

template <class SrvType>
void DracoNodelet::CallSetService(const std::string &slave_name,
                                  const std::string &srv_name,
                                  SrvType &srv_obj) {
  std::string full_set_service =
      "/" + slave_name + "/" + srv_name + "/" + "set";
  ros::NodeHandle nh = getPrivateNodeHandle(); // for Nodelets

  ros::ServiceClient client = nh.serviceClient<SrvType>(full_set_service);

  if (client.call(srv_obj)) {
    NODELET_INFO_STREAM("Called /" << slave_name.c_str() << "/"
                                   << srv_name.c_str()); // for Nodelets
  } else {
    NODELET_INFO_STREAM(
        "Failed to call service: " << full_set_service.c_str()); // for Nodelets
  }
}

template <class SrvType>
void DracoNodelet::CallGetService(const std::string &slave_name,
                                  const std::string &srv_name,
                                  SrvType &srv_obj) {
  std::string full_get_service =
      "/" + slave_name + "/" + srv_name + "/" + "get";
  ros::NodeHandle nh = getPrivateNodeHandle(); // for Nodelets

  ros::ServiceClient client = nh.serviceClient<SrvType>(full_get_service);

  if (client.call(srv_obj)) {
    NODELET_INFO_STREAM("Called /" << slave_name.c_str() << "/"
                                   << srv_name.c_str()); // for Nodelets
  } else {
    NODELET_INFO_STREAM(
        "Failed to call service: " << full_get_service.c_str()); // for Nodelets
  }
}

} // namespace draco_nodelet

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(draco_nodelet::DracoNodelet, nodelet::Nodelet)
