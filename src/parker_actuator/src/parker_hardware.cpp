#include "parker_actuator/parker_hardware.hpp"

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <sstream>
#include <string>
#include <termios.h>
#include <unistd.h>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace parker_actuator
{

// ─────────────────────────────────────────────────────────────────────────────
// Constructor
// ─────────────────────────────────────────────────────────────────────────────
ParkerHardwareInterface::ParkerHardwareInterface()
  : serial_fd_(-1),
    baud_rate_(115200),
    hw_position_state_(0.0),
    hw_velocity_state_(0.0),
    hw_position_command_(0.0),
    hw_velocity_command_(0.1),
    last_sent_position_(-9999.0),
    e_stop_active_(false)
{
}

// ─────────────────────────────────────────────────────────────────────────────
// on_init
// ─────────────────────────────────────────────────────────────────────────────
hardware_interface::CallbackReturn
ParkerHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Bug fix #2 – catch missing parameters gracefully instead of crashing
  try {
    port_      = info.hardware_parameters.at("serial_port");
    baud_rate_ = std::stoi(info.hardware_parameters.at("baud_rate"));
  } catch (const std::out_of_range &) {
    RCLCPP_WARN(rclcpp::get_logger("ParkerHardwareInterface"),
      "serial_port or baud_rate not set in URDF – using defaults: /dev/ttyUSB0, 115200");
    if (port_.empty()) port_ = "/dev/ttyUSB0";
  } catch (const std::invalid_argument & e) {
    RCLCPP_ERROR(rclcpp::get_logger("ParkerHardwareInterface"),
      "Invalid baud_rate parameter: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("ParkerHardwareInterface"),
    "Initialised – port: %s  baud: %d", port_.c_str(), baud_rate_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ─────────────────────────────────────────────────────────────────────────────
// Export interfaces
// ─────────────────────────────────────────────────────────────────────────────
std::vector<hardware_interface::StateInterface>
ParkerHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.emplace_back(
    info_.joints[0].name, hardware_interface::HW_IF_POSITION, &hw_position_state_);
  state_interfaces.emplace_back(
    info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &hw_velocity_state_);
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
ParkerHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.emplace_back(
    info_.joints[0].name, hardware_interface::HW_IF_POSITION, &hw_position_command_);
  command_interfaces.emplace_back(
    info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &hw_velocity_command_);
  return command_interfaces;
}

// ─────────────────────────────────────────────────────────────────────────────
// Lifecycle – activate
// ─────────────────────────────────────────────────────────────────────────────
hardware_interface::CallbackReturn
ParkerHardwareInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("ParkerHardwareInterface"),
    "Activating – opening %s at %d baud", port_.c_str(), baud_rate_);

  if (!openSerial()) {
    RCLCPP_ERROR(rclcpp::get_logger("ParkerHardwareInterface"),
      "Could not open serial port %s: %s", port_.c_str(), std::strerror(errno));
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Release E-Stop on fresh activation
  e_stop_active_.store(false);
  last_sent_position_ = -9999.0; // Force first write through

  // Enable drive
  if (!enableDrive()) {
    RCLCPP_WARN(rclcpp::get_logger("ParkerHardwareInterface"),
      "Drive enable command failed – check Compax3 state");
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ─────────────────────────────────────────────────────────────────────────────
// Lifecycle – deactivate
// ─────────────────────────────────────────────────────────────────────────────
hardware_interface::CallbackReturn
ParkerHardwareInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("ParkerHardwareInterface"), "Deactivating – disabling drive");
  disableDrive();
  closeSerial();
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ─────────────────────────────────────────────────────────────────────────────
// read() – poll actual position AND velocity from Compax3
// ─────────────────────────────────────────────────────────────────────────────
hardware_interface::return_type
ParkerHardwareInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (e_stop_active_.load()) {
    RCLCPP_ERROR(rclcpp::get_logger("ParkerHardwareInterface"), "E-STOP active – read halted");
    return hardware_interface::return_type::ERROR;
  }

  // ── Read actual position (Object 680.5, unit: mm) ──────────────────────
  sendCommand("O680.5");
  std::string resp_pos = readResponse();
  try {
    if (!resp_pos.empty()) {
      hw_position_state_ = std::stod(resp_pos) / 1000.0; // mm → m
    }
  } catch (const std::invalid_argument &) {
    RCLCPP_WARN(rclcpp::get_logger("ParkerHardwareInterface"),
      "Position parse failed (invalid): '%s'", resp_pos.c_str());
  } catch (const std::out_of_range &) {
    RCLCPP_WARN(rclcpp::get_logger("ParkerHardwareInterface"),
      "Position parse failed (range): '%s'", resp_pos.c_str());
  }

  // ── Read actual velocity (Object 680.6, unit: mm/s) ────────────────────
  // Bug fix #6 – velocity state was never updated
  sendCommand("O680.6");
  std::string resp_vel = readResponse();
  try {
    if (!resp_vel.empty()) {
      hw_velocity_state_ = std::stod(resp_vel) / 1000.0; // mm/s → m/s
    }
  } catch (...) {
    // keep last known velocity
  }

  return hardware_interface::return_type::OK;
}

// ─────────────────────────────────────────────────────────────────────────────
// write() – send position + velocity only when command has changed
// ─────────────────────────────────────────────────────────────────────────────
hardware_interface::return_type
ParkerHardwareInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Safety: block writes during E-Stop
  if (e_stop_active_.load()) {
    sendCommand("O400.1=0"); // Halt command
    return hardware_interface::return_type::ERROR;
  }

  // ── Software safety clamps ─────────────────────────────────────────────
  double safe_pos = std::clamp(hw_position_command_, MIN_POSITION_M, MAX_POSITION_M);
  double safe_vel = std::clamp(std::abs(hw_velocity_command_), MIN_VELOCITY_MS, MAX_VELOCITY_MS);

  if (safe_pos != hw_position_command_) {
    RCLCPP_WARN(rclcpp::get_logger("ParkerHardwareInterface"),
      "[SAFETY] Position clamped: %.4f → %.4f m", hw_position_command_, safe_pos);
  }
  if (std::abs(hw_velocity_command_) != safe_vel) {
    RCLCPP_WARN(rclcpp::get_logger("ParkerHardwareInterface"),
      "[SAFETY] Velocity clamped: %.4f → %.4f m/s", hw_velocity_command_, safe_vel);
  }

  // ── Bug fix #4 – only trigger move when command changes beyond deadband ─
  if (std::abs(safe_pos - last_sent_position_) < POSITION_DEADBAND_M) {
    return hardware_interface::return_type::OK; // nothing to do
  }

  // Build and send commands
  std::stringstream ss_pos, ss_vel;
  ss_pos << "O620.1=" << std::fixed << (safe_pos * 1000.0); // m → mm
  ss_vel << "O620.2=" << std::fixed << (safe_vel * 1000.0); // m/s → mm/s

  bool pos_ok = sendCommand(ss_vel.str()); // velocity FIRST so drive uses it for next move
  bool vel_ok = sendCommand(ss_pos.str());
  bool go_ok  = sendCommand("O400.1=1");   // trigger move

  // Bug fix #5 – check that all writes succeeded
  if (!pos_ok || !vel_ok || !go_ok) {
    RCLCPP_ERROR(rclcpp::get_logger("ParkerHardwareInterface"),
      "Serial write failed – possible disconnect. Activating E-Stop.");
    e_stop_active_.store(true);
    return hardware_interface::return_type::ERROR;
  }

  last_sent_position_ = safe_pos;

  RCLCPP_DEBUG(rclcpp::get_logger("ParkerHardwareInterface"),
    "CMD → pos: %.4f m  vel: %.4f m/s", safe_pos, safe_vel);

  return hardware_interface::return_type::OK;
}

// ─────────────────────────────────────────────────────────────────────────────
// Drive enable / disable
// ─────────────────────────────────────────────────────────────────────────────
bool ParkerHardwareInterface::enableDrive()
{
  // Compax3: enable power stage – O300.1=1 (check your specific config)
  return sendCommand("O300.1=1");
}

void ParkerHardwareInterface::disableDrive()
{
  sendCommand("O300.1=0"); // disable power stage
  sendCommand("O400.1=0"); // cancel any active motion
}

// ─────────────────────────────────────────────────────────────────────────────
// openSerial
// ─────────────────────────────────────────────────────────────────────────────
bool ParkerHardwareInterface::openSerial()
{
  serial_fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
  if (serial_fd_ == -1) return false;

  struct termios tty;
  if (tcgetattr(serial_fd_, &tty) != 0) {
    // Bug fix #3 – close fd on tcgetattr failure to prevent leaking
    close(serial_fd_);
    serial_fd_ = -1;
    return false;
  }

  speed_t baud = B115200;
  if      (baud_rate_ == 9600)  baud = B9600;
  else if (baud_rate_ == 19200) baud = B19200;
  else if (baud_rate_ == 38400) baud = B38400;
  else if (baud_rate_ == 57600) baud = B57600;

  cfsetospeed(&tty, baud);
  cfsetispeed(&tty, baud);

  tty.c_cflag  = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit chars
  tty.c_iflag &= ~IGNBRK;
  tty.c_lflag  = 0;         // no signalling chars, no echo, no canonical processing
  tty.c_oflag  = 0;         // no remapping, no delays
  tty.c_cc[VMIN]  = 0;      // non-blocking reads
  tty.c_cc[VTIME] = 5;      // 0.5 second read timeout
  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // no software flow control
  tty.c_cflag |=  (CLOCAL | CREAD);
  tty.c_cflag &= ~(PARENB | PARODD);      // no parity
  tty.c_cflag &= ~CSTOPB;                 // 1 stop bit
  tty.c_cflag &= ~CRTSCTS;               // no HW flow control

  if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
    close(serial_fd_);
    serial_fd_ = -1;
    return false;
  }

  // Flush stale data
  tcflush(serial_fd_, TCIOFLUSH);
  return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// closeSerial
// ─────────────────────────────────────────────────────────────────────────────
void ParkerHardwareInterface::closeSerial()
{
  if (serial_fd_ != -1) {
    close(serial_fd_);
    serial_fd_ = -1;
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// sendCommand – Bug fix #5: check return value
// ─────────────────────────────────────────────────────────────────────────────
bool ParkerHardwareInterface::sendCommand(const std::string & cmd)
{
  if (serial_fd_ == -1) return false;

  std::string full_cmd = cmd + "\r";
  ssize_t written = ::write(serial_fd_, full_cmd.c_str(), full_cmd.length());

  if (written < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("ParkerHardwareInterface"),
      "Serial write error for cmd '%s': %s", cmd.c_str(), std::strerror(errno));
    return false;
  }
  if (static_cast<size_t>(written) < full_cmd.length()) {
    RCLCPP_WARN(rclcpp::get_logger("ParkerHardwareInterface"),
      "Partial write for cmd '%s': wrote %zd of %zu bytes",
      cmd.c_str(), written, full_cmd.length());
    return false;
  }
  return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// readResponse – character-by-character, terminates on CR/LF
// ─────────────────────────────────────────────────────────────────────────────
std::string ParkerHardwareInterface::readResponse(int timeout_ms)
{
  if (serial_fd_ == -1) return "";

  std::string response;
  response.reserve(32);
  char c;
  int loops = timeout_ms; // 1 ms per iteration approx

  while (loops-- > 0) {
    ssize_t n = ::read(serial_fd_, &c, 1);
    if (n == 1) {
      if (c == '\r' || c == '\n') {
        if (!response.empty()) break; // end of frame
        // else skip leading terminators
      } else {
        response += c;
      }
    } else if (n == 0 || (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))) {
      usleep(1000); // wait 1 ms then retry
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("ParkerHardwareInterface"),
        "Serial read error: %s", std::strerror(errno));
      break;
    }
  }

  return response;
}

}  // namespace parker_actuator

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(parker_actuator::ParkerHardwareInterface, hardware_interface::SystemInterface)
