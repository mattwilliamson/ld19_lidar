#ifndef INCLUDE_LD19_LIDAR_LD19_NODE
#define INCLUDE_LD19_LIDAR_LD19_NODE

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <boost/system/system_error.hpp>

#include "lipkg.h"
#include "async_serial.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"

/*Fixed parameters of the sensor*/
static const float ANGLE_MIN = 0;
static const float ANGLE_MAX = 2 * M_PI;
static const float RANGE_MIN = 0.03;
static const float RANGE_MAX = 12.0;

enum class Status {
  INIT,
  ERROR,
  PUBLISHING
};

class LD19Node : public rclcpp::Node
{
public:
  LD19Node();
  void populate_message(const std::vector<PointData> & laser_data);
  bool init_device();
  void timer_callback();
  void init_parameters();
  void callback_updateDiagnostic(diagnostic_updater::DiagnosticStatusWrapper & stat);

private:
  static auto map_range(float x, float in_min, float in_max, float out_min, float out_max)->float
  {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }

  static auto wrap_angle(float angle)->float
  {
    if (angle > M_PI) {
      angle -= (M_PI * 2);
    } else if (angle < -M_PI) {
      angle += (M_PI * 2);
    }
    return angle;
  }
  std::shared_ptr<LiPkg> lidar_;
  std::shared_ptr<CallbackAsyncSerial> serial_port_;
  std::string port_;
  std::string frame_id_;
  std::string topic_name_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
  sensor_msgs::msg::LaserScan output_;
  diagnostic_updater::Updater diagnostic_;
  Status status_;
  uint16_t beams_;
  uint32_t baud_rate_;
};

#endif /* INCLUDE_LD19_LIDAR_LD19_NODE */
