#include "ld19_node.hpp"

using namespace std::chrono_literals;

LD19Node::LD19Node()
    : Node("ld19_lidar_node"), port_("/dev/ttyUSB0"), frame_id_("laser"), topic_name_("scan"), output_(), diagnostic_(this), status_(Status::INIT), beams_(0), baud_rate_(230400) {
  this->init_parameters();
  publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>(topic_name_, 10);
  timer_ = this->create_wall_timer(100ms, std::bind(&LD19Node::timer_callback, this));

  // Diagnostic updater
  diagnostic_.add("Status", this, &LD19Node::callback_updateDiagnostic);
  diagnostic_.setHardwareID("LD19");

  RCLCPP_INFO(get_logger(), "ld19_lidar_node start");
}

void LD19Node::init_parameters() {
  this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
  this->declare_parameter<std::string>("frame_id", "laser");
  this->declare_parameter<std::string>("topic_name", "scan");
  this->declare_parameter<int>("beams", 455);
  this->declare_parameter<int>("baud_rate", 230400);
  this->get_parameter("port", port_);
  this->get_parameter("frame_id", frame_id_);
  this->get_parameter("topic_name", topic_name_);
  this->get_parameter("beams", beams_);
  this->get_parameter("baud_rate", baud_rate_);

  output_.header.stamp = this->now();
  output_.header.frame_id = frame_id_;
  output_.angle_min = ANGLE_MIN;
  output_.angle_max = ANGLE_MAX;
  output_.range_min = RANGE_MIN;
  output_.range_max = RANGE_MAX;
  output_.angle_increment = output_.time_increment = 0.0;
  output_.scan_time = 0.0;
  output_.ranges.assign(beams_, 0.0);
  output_.intensities.assign(beams_, 0.0);
}

bool LD19Node::init_device() {
  lidar_ = std::make_shared<LiPkg>();

  try {
    serial_port_ = std::make_shared<CallbackAsyncSerial>(port_, baud_rate_);
  } catch (const boost::system::system_error &e) {
    RCLCPP_ERROR(this->get_logger(), "Error opening device port: %s: %s", port_.c_str(), e.what());
    status_ = Status::ERROR;
    return false;
  }

  serial_port_->setCallback(
      [=](const char *byte, size_t len) {
        if (lidar_->Parse((uint8_t *)byte, len)) {
          lidar_->AssemblePacket();
        }
      });

  lidar_->SetPopulateCallback(std::bind(&LD19Node::populate_message, this, std::placeholders::_1));

  if (!serial_port_->isOpen()) {
    RCLCPP_ERROR(this->get_logger(), "Error opening device port: %s", port_.c_str());
    status_ = Status::ERROR;
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "Successfully opened device port: %s", port_.c_str());
  status_ = Status::PUBLISHING;

  return true;
}

void LD19Node::populate_message(const std::vector<PointData> &laser_data) {
  static bool first_scan = true;
  double spin_speed = lidar_->GetSpeed();
  rclcpp::Time start_scan_time = this->now();
  static rclcpp::Time end_scan_time;
  float angle_increment = (ANGLE_MAX - ANGLE_MIN) / (float)(beams_ - 1);
  double scan_time = (start_scan_time.seconds() - end_scan_time.seconds());
  float time_increment = static_cast<float>(scan_time / (double)(beams_ - 1));

  // Figure out how fast we scan
  if (first_scan) {
    first_scan = false;
    end_scan_time = start_scan_time;
    return;
  }

  if (spin_speed <= 0) {
    return;
  }

  output_.header.stamp = start_scan_time;
  output_.time_increment = time_increment;
  output_.angle_increment = angle_increment;
  output_.scan_time = scan_time;

  // Start with NaN for all measurements
  output_.ranges.assign(beams_, std::numeric_limits<float>::quiet_NaN());
  output_.intensities.assign(beams_, std::numeric_limits<float>::quiet_NaN());

  for (auto point : laser_data) {
    float range = point.distance / 1000.0;
    float angle = ANGLE_TO_RADIAN(point.angle);
    float intensity = point.confidence;

    if ((point.distance == 0) && (intensity == 0)) {
      range = std::numeric_limits<float>::quiet_NaN();
      intensity = std::numeric_limits<float>::quiet_NaN();
    }

    int index = static_cast<int>(ceil((angle - ANGLE_MIN) / angle_increment));
    index = beams_ - index - 1;

    if (index > beams_) {
      continue;
    }

    output_.intensities[index] = intensity;

    // If the current content is Nan, it is assigned directly
    if (std::isnan(output_.ranges[index])) {
      output_.ranges[index] = range;
    } else {
      // Otherwise, only when the distance is less than the current range
      if (range < output_.ranges[index]) {
        output_.ranges[index] = range;
      }
    }
  }

  end_scan_time = start_scan_time;
}

void LD19Node::timer_callback() {
  if (lidar_->IsFrameReady()) {
    if (publisher_->get_subscription_count() > 0) {
      publisher_->publish(output_);
    }
    lidar_->ResetFrameReady();
  }
}

void LD19Node::callback_updateDiagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat) {
  if (status_ == Status::PUBLISHING) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Publishing");
    stat.addf("Scan Speed", "%.3f Hz", lidar_->GetSpeed());
  } else if (status_ == Status::ERROR) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Error");
  }
}
