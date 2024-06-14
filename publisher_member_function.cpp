#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <vector>
#include <algorithm>
#include <numeric>
#include <cstring>
#include <string>
#include <cmath>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int16.hpp"
#include "parser.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;


class PreProcessNode : public rclcpp::Node
{
public:
  PreProcessNode()
  : Node("pre_process_node_VW"), count_(0), fsync_mode_(0), pre_status_(0), debug_mode(true)
  {
    subscription_fsync_ = this->create_subscription<std_msgs::msg::Int16>(
      "/Map/FileSync", 10, std::bind(&PreProcessNode::fsync_callback, this, _1));  // /Map/FileSync 토픽 구독

    pre_status_publisher = this->create_publisher<std_msgs::msg::Int16>("/Map/Preprocess_Status_VW", 100); //Map/Preprocess_Status 토픽 퍼블리셔 생성

    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&PreProcessNode::timer_callback, this));
  }

private:
  int fsync_mode = 0, pre_status = 0;
  std::unique_ptr<Parser> _parser;

  void fsync_callback(const std_msgs::msg::Int16::SharedPtr msg)
  {
    fsync_mode_ = msg->data;
    if (debug_mode == true) RCLCPP_INFO(this->get_logger(),"fsync callback '%d'", msg->data);
    generation();
  }

  void generation()
  {
    if (fsync_mode_ == 3)
    {
      vw_draw_txt();
      pre_status_ = 1;
    }
    else
    {
      pre_status_ = 0;
    }

    auto pre_status_msg = std_msgs::msg::Int16();
    pre_status_msg.data = pre_status_;
    pre_status_publisher->publish(pre_status_msg);
  }

  void vw_draw_txt()
  {
    const char* dxf_filename = (char *)"/home/user/chanyoung/abc.dxf";  // DXF 파일의 경로 지정
        //std::string dxf_file_path = "/home/user/chanyoung/abc.dxf";

    std::pair<std::vector<std::vector<double>>,std::vector<std::vector<double>>> arr = _parser-> parse_dxf(dxf_filename);

  }

  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = " sub 대기 " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message); // 주석 처리된 부분: 필요 시 수정
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr subscription_fsync_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr pre_status_publisher;
  size_t count_;
  int fsync_mode_;
  int pre_status_;
  bool debug_mode;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PreProcessNode>());
  rclcpp::shutdown();
  return 0;
}

