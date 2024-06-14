#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "parser.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class PostProcessNode : public rclcpp::Node
{
public:
  PostProcessNode()
  : Node("post_processor")
  {

    this->rostopic();
    timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&PostProcessNode::ros_pub, this));

    pre_status = 0;
    post_status = 0;
    DrawNum.clear();
    DrawCnt.clear();
    Draw_X.clear();
    Draw_Y.clear();
    joint_3d.clear();

  }

private:

  // 변수 설정
  double Num_VW;
  int pre_status;
  int post_status;

  std::vector<double> DrawNum;
  std::vector<double> DrawCnt;
  std::vector<double> Draw_X;
  std::vector<double> Draw_Y;
  std::vector<double> joint_3d;

  std::vector<double> VW_Draw_X_Final;
  std::vector<double> VW_Draw_Y_Final;
  std::vector<int> VW_Draw_Count_Final;
  int VW_Draw_Num_Final;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr Preprocess_Status_VW_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_joint_3d_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr drawnum_publisher;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr drawcnt_publisher;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr draw_x_publisher;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr draw_y_publisher;
  rclcpp::TimerBase::SharedPtr timer_;

  void rostopic()
  {
    auto qos_profile = rclcpp::QoS(1);
    // subscriber
    subscription_ = this->create_subscription<std_msgs::msg::String>("topic", 10, std::bind(&PostProcessNode::topic_callback, this, _1));
    Preprocess_Status_VW_ = this->create_subscription<std_msgs::msg::Int16>("/Map/Preprocess_Status_VW", 10, std::bind(&PostProcessNode::status_callback, this, _1));
    subscription_joint_3d_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("/Kinematic/Joint_3D", 10, std::bind(&PostProcessNode::joint_3d_callback, this, _1));
    auto publisher = this->create_publisher<std_msgs::msg::String>("topic_name", qos_profile);

    // publisher
    drawnum_publisher = this->create_publisher<std_msgs::msg::Float64>("/VW/DdddrawNum", 10);
    drawcnt_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>("/VW/DrawCnt", 10);
    draw_x_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>("/VW/Draw_X", 10);
    draw_y_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>("/VW/Draw_Y", 10);

  }

  void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());

  }

  void status_callback(const std_msgs::msg::Int16::SharedPtr msg)
    {
        this->pre_status = msg->data;
        RCLCPP_INFO(this->get_logger(), "pre_status : '%d'", msg->data);

        if (this->pre_status == 0)
    {
        this->post_status = 0;
    }
        else if (this->pre_status == 1)
        {
            this->post_status = 1;

            std::cout << "txt 파일 로딩 " << std::endl;

            DrawNum = load_data_from_txt("/home/user/chanyoung/DrawNum_test.txt");
            DrawCnt = load_data_from_txt("/home/user/chanyoung/DrawCnt_test.txt");
            Draw_X = load_data_from_txt("/home/user/chanyoung/DrawX_test.txt");
            Draw_Y = load_data_from_txt("/home/user/chanyoung/DrawY_test.txt");

            // Print loaded data
            std::cout << "DrawNum: ";
            for (const auto &value : DrawNum)
            {
                std::cout << value << " ";
            }
            std::cout << std::endl;

            std::cout << "DrawCnt: ";
            for (const auto &value : DrawCnt)
            {
                std::cout << value << " ";
            }
            std::cout << std::endl;

            std::cout << "Draw_X: ";
            for (const auto &value : Draw_X)
            {
                std::cout << value << " ";
            }
            std::cout << std::endl;

            std::cout << "Draw_Y: ";
            for (const auto &value : Draw_Y)
            {
                std::cout << value << " ";
            }
            std::cout << std::endl;
    }
    else
    {
        this->post_status = 0;
    }}

  std::vector<double> load_data_from_txt(const std::string &file_path)
  {
      std::vector<double> data;
      try
      {
          std::ifstream file(file_path);
          if (!file.is_open())
          {
              throw std::runtime_error("Unable to open file: " + file_path);
          }
          std::string line;
          while (std::getline(file, line))
          {
              std::istringstream iss(line);
              double value;
              while (iss >> value)
              {
                  data.push_back(value);
              }
          }
      }
      catch (const std::exception &e)
      {
          RCLCPP_ERROR(this->get_logger(), "Error loading data from %s: %s", file_path.c_str(), e.what());
      }
      return data;
  }

  void joint_3d_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) const
  {
    std::string data_str;
    for (const auto & value : msg->data) {
      data_str += std::to_string(value) + " ";
    }
    RCLCPP_INFO(this->get_logger(), "Joint 3D Data: '%s'", data_str.c_str());
      }


  void ros_pub()
    {
        this->processing_node();

        auto float_msg = std_msgs::msg::Float64();
        float_msg.data = static_cast<double>(VW_Draw_Num_Final);
        drawnum_publisher->publish(float_msg);

        auto float_multi_msg = std_msgs::msg::Float64MultiArray();
        for (auto &count : VW_Draw_Count_Final)
        {
            float_multi_msg.data.push_back(count);
        }
        drawcnt_publisher->publish(float_multi_msg);

        float_multi_msg.data.clear();
        for (auto &x : VW_Draw_X_Final)
        {
            float_multi_msg.data.push_back(x);
        }
        draw_x_publisher->publish(float_multi_msg);

        float_multi_msg.data.clear();
        for (auto &y : VW_Draw_Y_Final)
        {
            float_multi_msg.data.push_back(y);
        }
        draw_y_publisher->publish(float_multi_msg);
    }

  void processing_node()
    {
      std::cout << " post processing node 실행 " << std::endl;
      if (post_status == 1)
      {
        double CoR_X = 0; //joint_3d[48];
        double CoR_Y = 0; //joint_3d[49];

        std::vector<double> Zone_Search_X = {CoR_X - 15, CoR_X + 15, CoR_X + 15, CoR_X - 15, CoR_X - 15};
        std::vector<double> Zone_Search_Y = {CoR_Y - 15, CoR_Y - 15, CoR_Y + 15, CoR_Y + 15, CoR_Y - 15};

        std::vector<std::pair<double, double>> zone;
            for (size_t i = 0; i < Zone_Search_X.size(); i++)
            {
                zone.emplace_back(Zone_Search_X[i], Zone_Search_Y[i]);
            }
      }
      else
        {
            VW_Draw_Num_Final = 0;
            VW_Draw_Count_Final.clear();
            VW_Draw_X_Final.clear();
            VW_Draw_Y_Final.clear();
        }
    }


};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PostProcessNode>());
  rclcpp::shutdown();
  return 0;
}
