// Test-only observer: use actual DDS message provenance, never discovery lists.
#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <sstream>
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("mapping_tf_audit");
  auto sub = node->create_subscription<tf2_msgs::msg::TFMessage>("/tf", rclcpp::QoS(500).best_effort(),
    [](tf2_msgs::msg::TFMessage::ConstSharedPtr msg, const rclcpp::MessageInfo &info) {
      for (const auto &tf : msg->transforms) {
        if (tf.header.frame_id != "map" || tf.child_frame_id != "odom") continue;
        std::ostringstream gid;
        for (auto b : info.get_rmw_message_info().publisher_gid.data)
          gid << std::hex << std::setw(2) << std::setfill('0') << unsigned(b);
        auto wall = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
        std::cout << std::setprecision(16) << "{\"wall\":" << wall << ",\"gid\":\"" << gid.str()
          << "\",\"stamp_ns\":" << (int64_t(tf.header.stamp.sec)*1000000000LL + tf.header.stamp.nanosec)
          << "}" << std::endl;
      }
    });
  rclcpp::spin(node);
  rclcpp::shutdown();
}
