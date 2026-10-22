#include <fmt/core.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <yaml-cpp/yaml.h>

#include <chrono>
#include <stdexcept>
#include <string>
#include <string_view>

#include "io/camera.hpp"
#include "tools/logger.hpp"
#include "tools/yaml.hpp"

namespace
{
const std::string keys =
  "{help h usage ? |                          | 输出命令行参数说明 }"
  "{config-path c  | configs/standard3.yaml   | standard 使用的 yaml 配置路径 }"
  "{image i        | assets/standard_fanblade.jpg | 待发送的图像路径 }"
  "{topic t        | /image_raw               | ROS2 图像话题名 }"
  "{hz             | 30                       | 发布频率 }";

std::string resolve_cli_value(
  const cv::CommandLineParser & cli, int argc, char * argv[], const std::string & long_name,
  std::string_view short_name)
{
  const std::string value = cli.get<std::string>(long_name);
  if (!value.empty() && value != "true" && value != "false") return value;

  const std::string short_opt = fmt::format("-{}", short_name);
  const std::string long_opt = fmt::format("--{}", long_name);
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if ((arg == short_opt || arg == long_opt) && i + 1 < argc) {
      return argv[i + 1];
    }
    const std::string long_prefix = long_opt + "=";
    if (arg.rfind(long_prefix, 0) == 0) {
      return arg.substr(long_prefix.size());
    }
    const std::string short_prefix = short_opt + "=";
    if (arg.rfind(short_prefix, 0) == 0) {
      return arg.substr(short_prefix.size());
    }
  }

  return value;
}

cv::Size infer_standard_image_size(const std::string & config_path)
{
  const YAML::Node yaml = tools::load(config_path);
  if (yaml["image_width"] && yaml["image_height"]) {
    return cv::Size(yaml["image_width"].as<int>(), yaml["image_height"].as<int>());
  }

  tools::logger()->info("image_width/image_height not found, probing one frame via io::Camera");
  io::Camera camera(config_path);
  cv::Mat frame;
  std::chrono::steady_clock::time_point timestamp;
  camera.read(frame, timestamp);
  if (frame.empty()) {
    throw std::runtime_error("failed to probe frame size from io::Camera");
  }

  return frame.size();
}

sensor_msgs::msg::Image make_image_msg(const cv::Mat & image, const rclcpp::Time & stamp)
{
  sensor_msgs::msg::Image msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = "standard_camera";
  msg.height = static_cast<uint32_t>(image.rows);
  msg.width = static_cast<uint32_t>(image.cols);
  msg.encoding = sensor_msgs::image_encodings::BGR8;
  msg.is_bigendian = false;
  msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(image.step);
  msg.data.assign(image.datastart, image.dataend);
  return msg;
}
}  // namespace

int main(int argc, char ** argv)
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }

  const std::string config_path = resolve_cli_value(cli, argc, argv, "config-path", "c");
  const std::string image_path = resolve_cli_value(cli, argc, argv, "image", "i");
  const std::string topic_name = resolve_cli_value(cli, argc, argv, "topic", "t");
  const double hz = cli.get<double>("hz");

  cv::Mat source = cv::imread(image_path, cv::IMREAD_COLOR);
  if (source.empty()) {
    tools::logger()->error("Failed to load image: {}", image_path);
    return 1;
  }

  cv::Size standard_size;
  try {
    standard_size = infer_standard_image_size(config_path);
  } catch (const std::exception & e) {
    tools::logger()->error("Failed to infer standard image size: {}", e.what());
    return 1;
  }

  cv::Mat publish_image;
  if (source.size() == standard_size) {
    publish_image = source;
  } else {
    cv::resize(source, publish_image, standard_size, 0.0, 0.0, cv::INTER_LINEAR);
  }
  if (!publish_image.isContinuous()) publish_image = publish_image.clone();

  tools::logger()->info(
    "Publishing {} as {}x{} to topic {}", image_path, publish_image.cols, publish_image.rows,
    topic_name);

  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("standard_ros2_image_pub_test");
  auto publisher = node->create_publisher<sensor_msgs::msg::Image>(topic_name, 10);
  rclcpp::WallRate rate(hz);

  while (rclcpp::ok()) {
    auto msg = make_image_msg(publish_image, node->get_clock()->now());
    publisher->publish(msg);
    rclcpp::spin_some(node);
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
