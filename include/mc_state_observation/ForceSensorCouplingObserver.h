#pragma once

#include <mc_observers/Observer.h>

namespace mc_state_observation
{

struct ForceSensorCouplingObserver : public mc_observers::Observer
{
 public:
  void configure(const mc_control::MCController & ctl, const mc_rtc::Configuration &) override;
  void reset(const mc_control::MCController & ctl) override;
  bool run(const mc_control::MCController & ctl) override;
  void update(mc_control::MCController & ctl) override;

 public:
  Eigen::Vector2d theta_{0, 0};

protected:
  void addToLogger(const mc_control::MCController &, mc_rtc::Logger &, const std::string & category) override;
  void removeFromLogger(mc_rtc::Logger &, const std::string & category) override;

  void addToGUI(const mc_control::MCController &,
                mc_rtc::gui::StateBuilder &,
                const std::vector<std::string> & /* category */) override;

protected:
  /// @{
  std::string robot_ = ""; ///< Name of robot to which the IMU sensor belongs
  std::string forceSensor_ = ""; ///< Name of the sensor used for IMU readings
  std::string updateSensor_ = ""; ///< Name of the sensor to update with the results (default: imuSensor_)
  std::string datastoreName_ = ""; ///< Name on the datastore (default name())
  /// @}
};

} // namespace mc_state_observation
