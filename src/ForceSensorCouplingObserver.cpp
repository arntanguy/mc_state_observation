#include <mc_control/MCController.h>
#include <mc_observers/ObserverMacros.h>
#include <mc_state_observation/ForceSensorCouplingObserver.h>

namespace mc_state_observation
{

void ForceSensorCouplingObserver::configure(const mc_control::MCController & ctl, const mc_rtc::Configuration & config)
{
  robot_ = config("robot", ctl.robot().name());
  forceSensor_ = static_cast<std::string>(config("forceSensor"));
  updateSensor_ = config("updateSensor", forceSensor_);
  datastoreName_ = config("datastoreName", name());
  desc_ = fmt::format("{} (sensor={}, update sensor={})", name_, forceSensor_, updateSensor_);
}

void ForceSensorCouplingObserver::reset(const mc_control::MCController & ctl)
{
}

bool ForceSensorCouplingObserver::run(const mc_control::MCController & ctl)
{
  auto & robot = ctl.robot(robot_);
  const auto & fs = robot.forceSensor(forceSensor_);
  double Fz = fs.wrench().force().z();
  double Tx = fs.wrench().moment().x();
  double Ty = fs.wrench().moment().y();
  return Fz - theta_.dot(Eigen::Vector3d{Tx, Ty, 0.});
}

void ForceSensorCouplingObserver::update(mc_control::MCController & ctl)
{
  auto & data = *robot.data();
  forceSensors[data.forceSensorsIndex.at(forceSensor_)];
  auto & sensor = data.bodySensors.at(data.bodySensorsIndex.at(updateSensor_));
  sensor.orientation(quat);
}

void ForceSensorCouplingObserver::addToLogger(const mc_control::MCController &,
                                   mc_rtc::Logger & logger,
                                   const std::string & category)
{
  MC_RTC_LOG_HELPER(category+"_theta", theta_);
}

void ForceSensorCouplingObserver::removeFromLogger(mc_rtc::Logger & logger, const std::string & category)
{
  logger.removeLogEntry(category + "_orientation");
  if(log_kf_)
  {
    config_.removeFromLogger(logger, category);
  }
}

void ForceSensorCouplingObserver::addToGUI(const mc_control::MCController & ctl,
                                mc_rtc::gui::StateBuilder & gui,
                                const std::vector<std::string> & category)
{
  auto kf_category = category;
  kf_category.push_back("KalmanFilter");
  config_.addToGUI(gui, kf_category);

  gui.addElement(category, mc_rtc::gui::Button("Reset config to default", [this]() { config_ = defaultConfig_; }),
                 mc_rtc::gui::Button("Reset",
                                     [this, &ctl]() {
                                       mc_rtc::log::info("[{}] Manual reset triggerred", name());
                                       reset(ctl);
                                     }),
                 mc_rtc::gui::ArrayLabel("Result", {"r [deg]", "p [deg]", "y [deg]"}, [this]() -> Eigen::Vector3d {
                   return mc_rbdyn::rpyFromMat(m_orientation.transpose()) * 180. / mc_rtc::constants::PI;
                 }));
}

void ForceSensorCouplingObserver::KalmanFilterConfig::addToLogger(mc_rtc::Logger & logger, const std::string & category)
{
  logger.addLogEntry(category + "_covariance_state", [this]() { return stateCov; });
  logger.addLogEntry(category + "_covariance_ori_acc", [this]() { return orientationAccCov; });
  logger.addLogEntry(category + "_covariance_acc", [this]() { return acceleroCovariance; });
  logger.addLogEntry(category + "_covariance_gyr", [this]() { return gyroCovariance; });
}

void ForceSensorCouplingObserver::KalmanFilterConfig::removeFromLogger(mc_rtc::Logger & logger, const std::string & category)
{
  logger.removeLogEntry(category + "_covariance_state");
  logger.removeLogEntry(category + "_covariance_ori_acc");
  logger.removeLogEntry(category + "_covariance_acc");
  logger.removeLogEntry(category + "_covariance_gyr");
}

void ForceSensorCouplingObserver::KalmanFilterConfig::addToGUI(mc_rtc::gui::StateBuilder & gui,
                                                    const std::vector<std::string> & category)
{
  // clang-format off
  gui.addElement(category,
    mc_state_observation::gui::make_input_element("Compensate Mode", compensateMode),
    mc_state_observation::gui::make_input_element("acceleroCovariance", acceleroCovariance),
    mc_state_observation::gui::make_input_element("gyroCovariance", gyroCovariance),
    mc_state_observation::gui::make_input_element("orientationAccCov", orientationAccCov),
    mc_state_observation::gui::make_input_element("linearAccCov", linearAccCov),
    mc_state_observation::gui::make_input_element("stateCov", stateCov),
    mc_state_observation::gui::make_input_element("stateInitCov", stateInitCov),
    mc_state_observation::gui::make_rpy_input("offset", offset));
  // clang-format on
}

void ForceSensorCouplingObserver::KalmanFilterConfig::removeFromGUI(mc_rtc::gui::StateBuilder & gui,
                                                         const std::vector<std::string> & category)
{
  gui.removeCategory(category);
}

} // namespace mc_state_observation
EXPORT_OBSERVER_MODULE("ForceSensorCoupling", mc_state_observation::ForceSensorCouplingObserver)
