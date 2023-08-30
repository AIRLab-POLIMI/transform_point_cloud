#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals;

class TransformPointCloud : public rclcpp::Node {
public:
    TransformPointCloud() : Node("transform_point_cloud_node") {
        state_publisher_active_ = false;
        cloud_publisher_active_ = false;

        cloud_buffer_ = {};

        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "/ouster/points",
                rclcpp::QoS(rclcpp::SensorDataQoS()),
                std::bind(&TransformPointCloud::topic_callback, this, _1));

        scan_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("registered_scan", 5);
        state_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("state_estimation", 10);
        global_frame_ = this->declare_parameter<std::string>("global_frame", "map");
        base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        cloud_timer_ = this->create_wall_timer(200ms, std::bind(&TransformPointCloud::cloud_timer_callback, this));
        state_timer_ = this->create_wall_timer(10ms, std::bind(&TransformPointCloud::state_timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Setup complete");
    }

private:
    void cloud_timer_callback() {
        try {
            geometry_msgs::msg::TransformStamped transformStamped;
            transformStamped =
                    tf_buffer_->lookupTransform(global_frame_, cloud_frame_, rclcpp::Time(0));

            std::lock_guard<std::mutex> guard(cloud_buffer_mutex_);

            auto cloud = std::lower_bound(cloud_buffer_.begin(),
                                             cloud_buffer_.end(),
                                             rclcpp::Time(transformStamped.header.stamp),
                                             [](const sensor_msgs::msg::PointCloud2 &cloud, const rclcpp::Time &value) {
                                                 return rclcpp::Time(cloud.header.stamp) < value;
                                             });

            if(cloud != cloud_buffer_.end() && cloud != cloud_buffer_.begin()) {
                cloud--;
                transformStamped =
                        tf_buffer_->lookupTransform(global_frame_, cloud_frame_, cloud->header.stamp);

                sensor_msgs::msg::PointCloud2 transformed_cloud;
                tf2::doTransform(*cloud, transformed_cloud, transformStamped);

                // Publish the transformed point cloud
                scan_pub_->publish(transformed_cloud);

                cloud_buffer_.clear();

                if (!cloud_publisher_active_) {
                    cloud_publisher_active_ = true;
                    RCLCPP_INFO(this->get_logger(), "Publishing the point cloud");
                }
            }

        } catch (const tf2::TransformException &ex) {
            if (cloud_publisher_active_) {
                cloud_publisher_active_ = false;
                RCLCPP_INFO(this->get_logger(), "point cloud publisher interrupted");
                RCLCPP_INFO(this->get_logger(), "%s", ex.what());

            } else {
                RCLCPP_INFO(this->get_logger(), "%s", ex.what());
            }
        }
    }

    void state_timer_callback() {
        try {
            geometry_msgs::msg::TransformStamped transformStamped;
            transformStamped = tf_buffer_->lookupTransform(base_frame_, global_frame_,
                                                           rclcpp::Time(0));
            nav_msgs::msg::Odometry odom_msg;
            odom_msg.header.stamp = transformStamped.header.stamp;
            odom_msg.header.frame_id = global_frame_;
            odom_msg.child_frame_id = base_frame_;
            odom_msg.pose.pose.position.x = transformStamped.transform.translation.x;
            odom_msg.pose.pose.position.y = transformStamped.transform.translation.y;
            odom_msg.pose.pose.position.z = transformStamped.transform.translation.z;
            odom_msg.pose.pose.orientation = transformStamped.transform.rotation;

            state_pub_->publish(odom_msg);

            if (!state_publisher_active_) {
                state_publisher_active_ = true;
                RCLCPP_INFO(this->get_logger(), "Publishing the state estimation");
            }
        } catch (const tf2::TransformException &ex) {
            if (state_publisher_active_) {
                state_publisher_active_ = false;
                RCLCPP_INFO(this->get_logger(), "State estimation publisher interrupted");
            } else {
                RCLCPP_INFO(this->get_logger(), "%s", ex.what());
            }
        }
    }

    void topic_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) {
        std::lock_guard<std::mutex> guard(cloud_buffer_mutex_);

        if(cloud_buffer_.size() > 15) {
            cloud_buffer_.clear();
        }

        cloud_buffer_.push_back(*msg);
        cloud_frame_ = msg->header.frame_id;
    }

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr scan_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr state_pub_;
    std::string global_frame_, base_frame_, cloud_frame_;
    rclcpp::TimerBase::SharedPtr cloud_timer_, state_timer_;
    std::list<sensor_msgs::msg::PointCloud2> cloud_buffer_;
    bool state_publisher_active_, cloud_publisher_active_;

    std::mutex cloud_buffer_mutex_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(std::make_shared<TransformPointCloud>());
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
