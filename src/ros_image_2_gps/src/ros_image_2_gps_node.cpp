#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/camera_trigger.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <px4_msgs/msg/sensor_gps.hpp>
#include <deque>
#include <optional>
#include <limits>
#include <fstream>
#include <inttypes.h> 
using CameraTrigger = px4_msgs::msg::CameraTrigger;
using NavSatFix     = px4_msgs::msg::SensorGps;

class ImageGpsSync : public rclcpp::Node
{
public:
  std::ofstream file_;
  uint64_t trigger_counter = 0;
  ImageGpsSync()
  : Node("image_gps_sync")
  {
    file_.open("/home/sarv-pi/TRIGGER_GPS_LOGS/output.txt", std::ios::out | std::ios::app);
    if (!file_) {
      RCLCPP_ERROR(get_logger(), "Failed to open file for writing!");
    } else {
      // RCLCPP_INFO(get_logger(), "File opened successfully.");
    }
    rmw_qos_profile_t qos_profile2 = rmw_qos_profile_sensor_data;
    auto qos2 = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile2.history, 10), qos_profile2);

    cam_sub_ = create_subscription<CameraTrigger>(
      "/fmu/out/camera_trigger", qos2,
      [&](CameraTrigger::UniquePtr msg){
        // RCLCPP_INFO(get_logger(), "[CameraTrigger] trigger!");
        trigger_counter +=1;
        last_trigger_us_ = msg->timestamp;
        auto gpsCoord = GetGPS(delay);
        if (gpsCoord){
          file_ << trigger_counter << "," << gpsCoord->latitude_deg << "," << gpsCoord->longitude_deg 
                << "," << gpsCoord->altitude_msl_m << "," << gpsCoord->heading << "," <<  gpsCoord-> timestamp << std::endl;
          // RCLCPP_INFO(get_logger(), "[CameraTrigger] GPS MATCH on seq %d!", msg->seq);
        }
        else{
          // RCLCPP_INFO(get_logger(), "[CameraTrigger] no GPS match");
        }
      });
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
    
    gps_sub_ = create_subscription<NavSatFix>(
      "/fmu/out/vehicle_gps_position", qos,
      [this](const px4_msgs::msg::SensorGps::UniquePtr msg){
        // RCLCPP_INFO(get_logger(), "Received GPS fix: lat=%.6f, lon=%.6f, alt=%.2f",
        //         msg->latitude_deg, msg->longitude_deg, msg->altitude_msl_m);
        gps_buffer_.push_back(*msg);
        if (gps_buffer_.size() > max_buffer_) {
          gps_buffer_.pop_front();
        }
      });
  }

  // void RGBCapture(double delay_s)
  // {
  //   RCLCPP_INFO(get_logger(), "[RGB] Capturing image…");
  //   auto g = GetGPS(delay_s);
  //   if (g) {
  //     RCLCPP_INFO(get_logger(),
  //       "[RGB]  lat=%.6f lon=%.6f alt=%.2f",
  //       g->latitude_deg, g->longitude_deg, g->altitude_msl_m
  //     );
  //   } else {
  //     RCLCPP_WARN(get_logger(), "[RGB]  no GPS match");
  //   }
  // }

  // void IRCapture(double delay_s)
  // {
  //   RCLCPP_INFO(get_logger(), "[IR ] Capturing image…");
  //   auto g = GetGPS(delay_s);
  //   if (g) {
  //     RCLCPP_INFO(get_logger(),
  //       "[IR ]  lat=%.6f lon=%.6f alt=%.2f",
  //       g->latitude_deg, g->longitude_deg, g->altitude_msl_m
  //     );
  //   } else {
  //     RCLCPP_WARN(get_logger(), "[IR ]  no GPS match");
  //   }
  // }

private:
  uint64_t ros_find_last_trigger_time() const { return last_trigger_us_; }
  std::optional<NavSatFix> ros_get_gps(uint64_t query_ns)
  {
    std::optional<NavSatFix> best;
    uint64_t best_diff = std::numeric_limits<uint64_t>::max();
    for (auto f = gps_buffer_.rbegin(); f != gps_buffer_.rend(); ++f){
      uint64_t t = static_cast<uint64_t>((*f).timestamp) * 1000L;
      uint64_t diff = (t > query_ns) ? (t - query_ns) : (query_ns - t);
      if (diff < best_diff) {
        best_diff = diff;
        best = *f;
      }
      else{
        break;
      }
    }
    return best;
  }

  std::optional<NavSatFix> GetGPS(double delay_s)
  {
    uint64_t trig_us = ros_find_last_trigger_time();
    uint64_t query_ns = trig_us * 1000ULL
                      + static_cast<uint64_t>(delay_s * 1e9);
    return ros_get_gps(query_ns);
  }

  rclcpp::Subscription<CameraTrigger>::SharedPtr cam_sub_;
  rclcpp::Subscription<NavSatFix>::SharedPtr gps_sub_;
  const double delay = 0.0;
  uint64_t last_trigger_us_{0};
  std::deque<NavSatFix> gps_buffer_;
  static constexpr size_t max_buffer_ = 1000;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImageGpsSync>();
  rclcpp::spin(node);
  node->file_.close();
  rclcpp::shutdown();
  return 0;
}