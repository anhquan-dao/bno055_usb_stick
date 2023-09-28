#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <bno055_usb_stick/bno055_usb_stick.hpp>
#include <bno055_usb_stick/decoder.hpp>
#include <bno055_usb_stick/msg/output.hpp>

#include <boost/asio/io_service.hpp>

namespace bus = bno055_usb_stick;

std::string pose_frame_id;

std::shared_ptr<rclcpp::Publisher<bus::msg::Output>> out_pub;
std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Imu>> imu_pub;
std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>> pose_pub;
std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::MagneticField>> mag_pub;
std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Temperature>> temp_pub;

std::unique_ptr<tf2_ros::TransformBroadcaster> tf_pub;
std::string tf_frame_id, tf_child_frame_id;
bool invert_tf;

void publish(const bno055_usb_stick::msg::Output &output) {
  if (out_pub->get_subscription_count() > 0) {
    out_pub->publish(output);
  }
  if (imu_pub->get_subscription_count() > 0) {
    imu_pub->publish(bus::Decoder::toImuMsg(output));
  }
  if (pose_pub->get_subscription_count() > 0) {
    pose_pub->publish(bus::Decoder::toPoseMsg(output, pose_frame_id));
  }
  if (mag_pub->get_subscription_count() > 0) {
    mag_pub->publish(bus::Decoder::toMagMsg(output));
  }
  if (temp_pub->get_subscription_count() > 0) {
    temp_pub->publish(bus::Decoder::toTempMsg(output));
  }
  if (tf_pub) {
    tf_pub->sendTransform(
        bus::Decoder::toTFTransform(output, tf_frame_id, tf_child_frame_id, invert_tf));
  }
}

int main(int argc, char *argv[]) {
  // init ROS
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("bno055_usb_stick_node");

  // load parameters
  node->declare_parameter("pose_frame_id", "fixed");
  node->declare_parameter("publish_tf", false);
  node->declare_parameter("tf_frame_id", "fixed");
  node->declare_parameter("tf_child_frame_id", "bno055");
  node->declare_parameter("invert_tf", false);

  pose_frame_id = node->get_parameter("pose_frame_id").as_string();
  const bool publish_tf = node->get_parameter("publish_tf").as_bool();
  tf_frame_id = node->get_parameter("tf_frame_id").as_string();
  tf_child_frame_id = node->get_parameter("tf_child_frame_id").as_string();
  invert_tf = node->get_parameter("invert_tf"). as_bool();

  // setup publishers
  out_pub   = node->create_publisher< bno055_usb_stick::msg::Output >("output", 1);
  imu_pub   = node->create_publisher< sensor_msgs::msg::Imu >("imu", 1);
  pose_pub  = node->create_publisher< geometry_msgs::msg::PoseStamped >("pose", 1);
  mag_pub   = node->create_publisher< sensor_msgs::msg::MagneticField >("magnetic_field", 1);
  temp_pub  = node->create_publisher< sensor_msgs::msg::Temperature >("temperature", 1);
  
  if (publish_tf) {
    tf_pub = std::make_unique<tf2_ros::TransformBroadcaster>(*node);
  }

  // construct the worker
  boost::asio::io_service asio_service;

  node->declare_parameter("port", "/dev/ttyACM0");
  node->declare_parameter("timeout", 1.0);
  node->declare_parameter("mode", "ndof");
  node->declare_parameter("frame_id", "bno055");

  bus::BNO055USBStick device(asio_service, publish, node);

  // run the worker
  while (rclcpp::ok())
  {
    asio_service.run_one();
    rclcpp::spin_some(node);
  }

  return 0;
}