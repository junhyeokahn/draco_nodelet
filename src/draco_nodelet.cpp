#include <stdexcept>

#include <draco_nodelet/draco_nodelet.hpp>

using namespace aptk::comm;
namespace draco_nodelet {
DracoNodelet::DracoNodelet() {
  axons_ = {"Neck_Pitch",    "R_Hip_IE",      "R_Hip_AA",      "R_Hip_FE",
            "R_Knee_FE",     "R_Ankle_FE",    "R_Ankle_IE",    "L_Hip_IE",
            "L_Hip_AA",      "L_Hip_FE",      "L_Knee_FE",     "L_Ankle_FE",
            "L_Ankle_IE",    "L_Shoulder_FE", "L_Shoulder_AA", "L_Shoulder_IE",
            "L_Elbow",       "L_Wrist_Roll",  "L_Wrist_Pitch", "R_Shoulder_FE",
            "R_Shoulder_AA", "R_Shoulder_IE", "R_Elbow",       "R_Wrist_Roll",
            "R_Wrist_Pitch"};
  medullas_ = {"Medulla", "Medulla_V4"};
  sensillums_ = {"Sensillum_v2"};

  count_ = 0;
  n_actuator_ = axons_.size();
  n_joint_ = n_actuator_ + 2; // include two knee proximal joint
  n_medulla_ = medullas_.size();
  n_sensillum_ = sensillums_.size();

  ph_joint_positions_data_.resize(n_actuator_);
  ph_joint_velocities_data_.resize(n_actuator_);
  ph_joint_positions_cmd_.resize(n_actuator_);
  ph_joint_velocities_cmd_.resize(n_actuator_);
  ph_joint_efforts_cmd_.resize(n_actuator_);

  b_pnc_alive_ = false;
  b_online_plot_ = true;
}

DracoNodelet::~DracoNodelet() {
  spin_thread_->join();
  for (int i = 0; i < n_actuator_; ++i) {
    delete ph_joint_positions_data_[i];
    delete ph_joint_velocities_data_[i];
    delete ph_joint_positions_cmd_[i];
    delete ph_joint_velocities_cmd_[i];
    delete ph_joint_efforts_cmd_[i];
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
}

void DracoNodelet::onInit() {
  nh_ = getNodeHandle();
  spin_thread_.reset(
      new boost::thread(boost::bind(&DracoNodelet::spinThread, this)));

  mode_handler_ =
      nh_.advertiseService("/mode_handler", &DracoNodelet::ModeHandler, this);
  pnc_handler_ =
      nh_.advertiseService("/pnc_handler", &DracoNodelet::PnCHandler, this);
}

void DracoNodelet::spinThread() {
  // set up controller
  sync_.reset(new aptk::comm::Synchronizer(true, "draco_nodelet"));
  sync_->connect();

  interfacer_.reset(new aptk::util::DebugInterfacer(
      "draco", sync_->getNodeHandle(), sync_->getLogger()));

  aptk::comm::enableRT(5, 2);

  RegisterData();

  SetImpedanceGains();

  SetCurrentLimits();

  ConstructPnC();

  TurnOffMotors();

  ClearFaults();

  // main control loop
  while (sync_->ok()) {

    // wait for bus transaction
    sync_->awaitNextControl();

    CopyData();
    if (sync_->printIndicatedFaults()) {
      // Faulted
    } else {
      // Compute Commands
      CopyCommand();
    }

    sync_->getLogger()->captureLine();

    // indicate that we're done
    sync_->finishControl();

    ++count_;

    // for plot
    // TODO add boolean, because we don't want to do this at every control loop
    // stop this after initiate exp
    if (b_online_plot_) {
      interfacer_->updateDebug();
    }
  }

  sync_->awaitShutdownComplete();
}

void DracoNodelet::RegisterData() {
  for (int i = 0; i < n_actuator_; ++i) {
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

  // TODO : add correct name and topic
  // interfacer_->addEigen(&data_eigen_vector_, "/eigen_vector", {"pos",
  // "vel"}); interfacer_->addPrimitive(&data_double_, "double");
}

void DracoNodelet::CopyData() {
  // TODO (JH) : When I have SensorData
  // copy data from placeholder to sensor data
  // figure out contact boolean as well
  // check if data is all safe before sending it
}

void DracoNodelet::CopyCommand() {
  // TODO (JH) : When I have Command
  // copy data from command to placeholder
  // check if command is all safe before sending it
}

void DracoNodelet::SetImpedanceGains() {
  // TODO (JH) read yaml and set gain

  // Here are some example code for it.
  // apptronik_srvs::Float32 srv_float;
  // srv_float.request.set_data = 0.123;
  // for (int i = 0; i < n_joint_; ++i) {
  // CallSetService(axons_[i], "Control__Joint__Impedance__KP", srv_float);
  //}
}

void DracoNodelet::SetCurrentLimits() {
  // TODO (JH) read yaml and set current limits

  // Here are some example code for it.
  // apptronik_srvs::Float32 srv_float;
  // srv_float.request.set_data = 0.123;
  // for (int i = 0; i < n_joint_; ++i) {
  // CallSetService(axons_[i], "Control__Joint__Impedance__KP", srv_float);
  //}
}

void DracoNodelet::ConstructPnC() {
  if (!b_pnc_alive_) {
    // TODO (JH) : construct pnc
    b_pnc_alive_ = true;
  }
}

void DracoNodelet::DestructPnC() {
  if (b_pnc_alive_) {
    // TODO (JH) : destruct pnc
    b_pnc_alive_ = false;
  }
}

bool DracoNodelet::ModeHandler(apptronik_srvs::Float32::Request &req,
                               apptronik_srvs::Float32::Response &res) {
  double data = static_cast<double>(req.set_data);
  if (data == 0) {
    std::cout << "Change to Off Mode" << std::endl;
    TurnOffMotors();
    return true;
  } else if (data == 1) {
    std::cout << "Change to Off Mode" << std::endl;
    TurnOnMotors();
    return true;
  } else {
    std::cout << "Wrong Data Received for ModeHandler()" << std::endl;
    return false;
  }
}

bool DracoNodelet::PnCHandler(apptronik_srvs::Float32::Request &req,
                              apptronik_srvs::Float32::Response &res) {
  double data = static_cast<double>(req.set_data);
  if (data == 0) {
    std::cout << "Destruct PnC" << std::endl;
    DestructPnC();
    return true;
  } else if (data == 1) {
    std::cout << "Construct PnC" << std::endl;
    ConstructPnC();
    return true;
  } else {
    std::cout << "Wrong Data Received for PnCHandler()" << std::endl;
    return false;
  }
}

void DracoNodelet::TurnOffMotors() {
  for (int i = 0; i < n_actuator_; ++i) {
    sync_->changeMode("OFF", axons_[i]);
  }
}

void DracoNodelet::TurnOnMotors() {
  for (int i = 0; i < n_actuator_; ++i) {
    sync_->changeMode("JOINT_IMPEDANCE", axons_[i]);
  }
}

void DracoNodelet::ClearFaults() {
  for (int i = 0; i < n_actuator_; ++i) {
    sync_->clearFaults(axons_[i]);
  }
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
