#ifndef PARKER_ACTUATOR_HARDWARE_INTERFACE_HPP
#define PARKER_ACTUATOR_HARDWARE_INTERFACE_HPP

#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace parker_actuator
{

// Safety limits (hardware-enforced in software)
static constexpr double MIN_POSITION_M      = 0.0;    // metres
static constexpr double MAX_POSITION_M      = 0.40;   // metres
static constexpr double MAX_VELOCITY_MS     = 0.50;   // metres/sec
static constexpr double MIN_VELOCITY_MS     = 0.001;  // avoid zero-velocity commands
static constexpr double POSITION_DEADBAND_M = 0.0002; // 0.2 mm – skip write if within band

class ParkerHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ParkerHardwareInterface)

  ParkerHardwareInterface();  // ← explicit constructor to initialize members

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Serial
  int         serial_fd_{-1};   // ← initialised to -1 (bug fix #1)
  std::string port_;
  int         baud_rate_{115200};

  // Joint state / command
  double hw_position_state_{0.0};
  double hw_velocity_state_{0.0};
  double hw_position_command_{0.0};
  double hw_velocity_command_{0.1};

  // Previous command – used to detect changes and avoid re-triggering (bug fix #4)
  double last_sent_position_{-9999.0};

  // E-Stop flag
  std::atomic<bool> e_stop_active_{false};

  // Serial helpers
  bool        openSerial();
  void        closeSerial();
  bool        sendCommand(const std::string & cmd);    // returns false on write fail
  std::string readResponse(int timeout_ms = 50);

  // Drive helpers
  bool        enableDrive();
  void        disableDrive();
};

}  // namespace parker_actuator

#endif  // PARKER_ACTUATOR_HARDWARE_INTERFACE_HPP
