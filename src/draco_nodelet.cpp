#include <stdexcept>

#include <draco_nodelet/draco_nodelet.hpp>

using namespace aptk::comm;
namespace draco_nodelet {
DracoNodelet::DracoNodelet() {
  count_ = 0;
  // TODO : change slave name
  slave_names_ = {"QDM9_3"};
  // TODO : add medulla
  // TODO : set right n_joint_
  n_joint_ = 1;

  // TODO : add more data
  joint_positions_.resize(n_joint_);
  joint_velocities_.resize(n_joint_);

  // TODO : use the right commands
  joint_current_.resize(n_joint_);

  // TODO : Initialize PnC
}

DracoNodelet::~DracoNodelet() { spin_thread_->join(); }

void DracoNodelet::onInit() {
  nh_ = getNodeHandle();
  spin_thread_.reset(
      new boost::thread(boost::bind(&DracoNodelet::spinThread, this)));
}

void DracoNodelet::spinThread() {
  // set up controller
  sync_.reset(new aptk::comm::Synchronizer(true, "draco_nodelet"));
  sync_->connect();
  aptk::comm::enableRT(5, 2);

  // Initialize
  RegisterData();
  SetServices();

  for (std::size_t i = 0; i < n_joint_; ++i) {
    sync_->clearFaults(slave_names_[i]);
    // TODO : Set the right mode
    sync_->changeMode("MOTOR_CURRENT", slave_names_[i]);
  }

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
  }

  sync_->awaitShutdownComplete();
}

void DracoNodelet::RegisterData() {
  // TODO : Register more data
  for (int i = 0; i < n_joint_; ++i) {
    // Register State
    joint_positions_[i] = new float(0.);
    sync_->registerMISOPtr(joint_positions_[i], "js__joint__position__rad",
                           slave_names_[i], false);
    joint_velocities_[i] = new float(0.);
    sync_->registerMISOPtr(joint_velocities_[i], "js__joint__velocity__radps",
                           slave_names_[i], false);

    // Register Command
    joint_current_[i] = new float(0.);
    sync_->registerMOSIPtr(joint_current_[i], "cmd__motor__effort__a",
                           slave_names_[i], false);
  }
}

void DracoNodelet::CopyData() {
  // TODO : Copy data to SensorData
  if (count_ % 200 == 0) {
    std::cout << *(joint_positions_[0]) << std::endl;
  }
}

void DracoNodelet::CopyCommand() {
  // TODO : Copy command from Command
  // TODO : Choose right data
  for (int i = 0; i < n_joint_; ++i) {
    *(joint_current_[i]) = 0.;
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

void DracoNodelet::SetServices() {
  // TODO : Use yaml reader
  // TODO : Set ros service
  apptronik_srvs::Float32 srv_float;
  srv_float.request.set_data = 0.123;
  for (int i = 0; i < n_joint_; ++i) {
    CallSetService(slave_names_[i], "Control__Joint__Impedance__KP", srv_float);
  }
}

} // namespace draco_nodelet

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(draco_nodelet::DracoNodelet, nodelet::Nodelet)
