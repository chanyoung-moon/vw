#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using std::placeholders::_1;

class PostProcessNode : public rclcpp::Node
{
public:
  PostProcessNode()
  : Node("post_processor")
  {

    // "topic" 토픽 구독
    subscription_ = this->create_subscription<std_msgs::msg::String>("topic", 10, std::bind(&PostProcessNode::topic_callback, this, _1));

     // "/Map/Preprocess_Status_VW" 토픽 구독
    Preprocess_Status_VW_ = this->create_subscription<std_msgs::msg::Int16>("/Map/Preprocess_Status_VW", 10, std::bind(&PostProcessNode::status_callback, this, _1));

    // "/Kinematic/Joint_3D" 토픽 구독
    subscription_joint_3d_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("/Kinematic/Joint_3D", 10, std::bind(&PostProcessNode::joint_3d_callback, this, _1));

  }

private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());

  }

  void status_callback(const std_msgs::msg::Int16::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "pre_status : '%d'", msg->data);

  }

  void joint_3d_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) const
  {
    std::string data_str;
    for (const auto & value : msg->data) {
      data_str += std::to_string(value) + " ";
    }
    RCLCPP_INFO(this->get_logger(), "Joint 3D Data: '%s'", data_str.c_str());
      }


  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr Preprocess_Status_VW_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_joint_3d_;

};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PostProcessNode>());
  rclcpp::shutdown();
  return 0;
}
