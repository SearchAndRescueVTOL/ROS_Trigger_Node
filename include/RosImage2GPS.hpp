#ifndef CAMERA_GPS_SYNC_HPP
#define CAMERA_GPS_SYNC_HPP

#include <deque>
#include <optional>
#include <px4_msgs/msg/camera_trigger.hpp>
#include <px4_msgs/msg/vehicle_gps_position.hpp>
#include <rclcpp/rclcpp.hpp>

/**
 * @brief A node that listens to camera‐trigger timestamps and GPS messages,
 *        then lets you query “GPS at trigger_time + delay”.
 */
class CameraGpsSyncNode : public rclcpp::Node {
public:
  CameraGpsSyncNode();

  /// Capture an RGB image, then lookup GPS at (last_trigger + delay)
  void RGBCapture(double known_time_delay_sec);

  /// Capture an IR image, then lookup GPS at (last_trigger + delay)
  void IRCapture(double known_time_delay_sec);

private:
  // ROS callbacks
  void cameraTriggerCallback(const px4_msgs::msg::CameraTrigger::SharedPtr msg);
  void gpsCallback(const px4_msgs::msg::VehicleGpsPosition::SharedPtr msg);

  // core lookup routines
  uint64_t ros_find_last_trigger_time() const;
  std::optional<px4_msgs::msg::VehicleGpsPosition>
  ros_get_gps(uint64_t query_timestamp) const;

  // high‐level wrapper: find GPS at (last_trigger + known_delay)
  std::optional<px4_msgs::msg::VehicleGpsPosition>
  GetGPS(double known_time_delay_sec) const;

  // subscriptions
  rclcpp::Subscription<px4_msgs::msg::CameraTrigger>::SharedPtr
      camera_trigger_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleGpsPosition>::SharedPtr gps_sub_;

  // state
  uint64_t last_trigger_time_{0}; ///< in μs
  std::deque<px4_msgs::msg::VehicleGpsPosition> gps_buffer_;
  static constexpr size_t max_buffer_size_ = 100;
};

#endif // CAMERA_GPS_SYNC_HPP
