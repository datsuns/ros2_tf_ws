// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "turtlesim/msg/pose.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "test_msgs/msg/num.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
    public:
        MinimalPublisher()
            : Node("minimal_publisher"), count_(0)
        {
            publisher_    = this->create_publisher<std_msgs::msg::String>("topic", 10);
            publisher2_   = this->create_publisher<test_msgs::msg::Num>("topic2", 10);
            subscription_ = this->create_subscription<turtlesim::msg::Pose>(
                    "/turtle1/pose", 10, std::bind(&MinimalPublisher::pose_callback, this, _1));

            this->declare_parameter("my_parameter", "world");
            this->turtlename_ = this->declare_parameter<std::string>("turtlename", "turtle");
            timer_ = this->create_wall_timer(
                    500ms, std::bind(&MinimalPublisher::timer_callback, this));
            tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        }

    private:
        void pose_callback(const turtlesim::msg::Pose::SharedPtr msg) const
        {
            //RCLCPP_INFO(this->get_logger(), "I heard: '%f.%f'", msg->x, msg->y);
            geometry_msgs::msg::TransformStamped t;
            t.header.stamp = this->get_clock()->now();
            t.header.frame_id = "world";
            t.child_frame_id = turtlename_.c_str();
        }
        void timer_callback()
        {
            std::string my_param = this->get_parameter("my_parameter").as_string();
            if( my_param == "world" ){
                auto message = std_msgs::msg::String();
                message.data = "Hello, world! " + std::to_string(count_++) + " w/ " + my_param;
                RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
                publisher_->publish(message);
            }
            else {
                auto message = test_msgs::msg::Num();
                message.num = this->count_++;
                RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: '" << message.num << "'");
                publisher2_->publish(message);
            }
        }
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
        rclcpp::Publisher<test_msgs::msg::Num>::SharedPtr   publisher2_;
        rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        std::string turtlename_;
        size_t count_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
}
