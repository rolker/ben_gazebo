// Gazebo helm node: converts Helm/cmd_vel commands to Gazebo thruster outputs.
//
// Replaces asv_helm for the Gazebo simulation path. Outputs thrust in Newtons
// and steering deflection in radians (matching Gz thruster/joint plugins).

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <control_toolbox/pid_ros.hpp>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>
#include <marine_interfaces/msg/helm.hpp>
#include <marine_interfaces/msg/heartbeat.hpp>
#include <marine_interfaces/msg/key_value.hpp>

using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class GazeboHelm : public rclcpp_lifecycle::LifecycleNode
{
public:
  GazeboHelm(const rclcpp::NodeOptions & options)
  : rclcpp_lifecycle::LifecycleNode("gazebo_helm", options)
  {
    // Declare parameters with defaults matching BEN's Gz model
    declare_parameter("max_forward_thrust", 1800.0);
    declare_parameter("max_reverse_thrust", 500.0);
    declare_parameter("max_deflection", 0.524);  // ~30 degrees
    declare_parameter("max_speed", 2.75);
    declare_parameter("max_yaw_speed", 0.5);
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    max_forward_thrust_ = get_parameter("max_forward_thrust").as_double();
    max_reverse_thrust_ = get_parameter("max_reverse_thrust").as_double();
    max_deflection_ = get_parameter("max_deflection").as_double();
    max_speed_ = get_parameter("max_speed").as_double();
    max_yaw_speed_ = get_parameter("max_yaw_speed").as_double();

    // Publishers — thrust in Newtons, pos in radians
    thrust_pub_ = create_publisher<std_msgs::msg::Float64>(
        "thrusters/main/thrust", 1);
    pos_pub_ = create_publisher<std_msgs::msg::Float64>(
        "thrusters/main/pos", 1);
    heartbeat_pub_ = create_publisher<marine_interfaces::msg::Heartbeat>(
        "marine/status/helm", 1);

    // Subscribers
    helm_sub_ = create_subscription<marine_interfaces::msg::Helm>(
        "helm", 1,
        [this](marine_interfaces::msg::Helm::ConstSharedPtr msg) {
          helmCallback(msg);
        });
    twist_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
        "cmd_vel", 10,
        [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr msg) {
          twistCallback(msg);
        });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "odom", 5,
        [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
          latest_odom_ = *msg;
          have_odom_ = true;
        });

    // Speed PID via control_toolbox
    speed_pid_ = std::make_unique<control_toolbox::PidROS>(
        get_node_base_interface(),
        get_node_logging_interface(),
        get_node_parameters_interface(),
        get_node_topics_interface(),
        "speed_pid.",
        "speed_pid_state",
        true);
    speed_pid_->initialize_from_ros_parameters();

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {
    thrust_pub_.reset();
    pos_pub_.reset();
    heartbeat_pub_.reset();
    helm_sub_.reset();
    twist_sub_.reset();
    odom_sub_.reset();
    speed_pid_.reset();
    return CallbackReturn::SUCCESS;
  }

private:
  void helmCallback(marine_interfaces::msg::Helm::ConstSharedPtr msg)
  {
    // Direct throttle/rudder → thrust/pos conversion
    double throttle = std::clamp(static_cast<double>(msg->throttle), -1.0, 1.0);
    double rudder = std::clamp(static_cast<double>(msg->rudder), -1.0, 1.0);
    publishCommands(throttle, rudder);
    publishHeartbeat();
  }

  void twistCallback(geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
  {
    double throttle;

    if (have_odom_) {
      auto now = get_clock()->now();
      auto odom_age = now - rclcpp::Time(latest_odom_.header.stamp, get_clock()->get_clock_type());
      if (odom_age.seconds() < 1.0) {
        double error = msg->twist.linear.x - latest_odom_.twist.twist.linear.x;
        rclcpp::Duration dt(0, 0);
        if (last_twist_time_.nanoseconds() > 0) {
          dt = now - last_twist_time_;
        }
        last_twist_time_ = now;

        if (dt.seconds() > 0.0 && dt.seconds() < 5.0) {
          throttle = speed_pid_->compute_command(error, dt);
          throttle = std::clamp(throttle, -1.0, 1.0);
        } else {
          throttle = msg->twist.linear.x / max_speed_;
          speed_pid_->reset();
        }
      } else {
        throttle = msg->twist.linear.x / max_speed_;
      }
    } else {
      throttle = msg->twist.linear.x / max_speed_;
    }

    double rudder = -msg->twist.angular.z / max_yaw_speed_;
    rudder = std::clamp(rudder, -1.0, 1.0);

    // Minimum throttle for steering authority (same logic as asv_helm)
    double min_throttle = 0.5 * std::abs(rudder);
    if (msg->twist.linear.x > 0.0) {
      throttle = std::max(throttle, min_throttle);
    } else if (msg->twist.linear.x < 0.0) {
      throttle = std::min(throttle, -min_throttle);
    }

    publishCommands(throttle, rudder);
    publishHeartbeat();
  }

  void publishCommands(double throttle, double rudder)
  {
    std_msgs::msg::Float64 thrust_msg;
    if (throttle >= 0.0) {
      thrust_msg.data = throttle * max_forward_thrust_;
    } else {
      thrust_msg.data = throttle * max_reverse_thrust_;
    }
    thrust_pub_->publish(thrust_msg);

    std_msgs::msg::Float64 pos_msg;
    pos_msg.data = rudder * max_deflection_;
    pos_pub_->publish(pos_msg);
  }

  void publishHeartbeat()
  {
    marine_interfaces::msg::Heartbeat hb;
    hb.header.stamp = get_clock()->now();

    marine_interfaces::msg::KeyValue kv;
    kv.key = "sim";
    kv.value = "gazebo";
    hb.values.push_back(kv);

    heartbeat_pub_->publish(hb);
  }

  // Parameters
  double max_forward_thrust_{1800.0};
  double max_reverse_thrust_{500.0};
  double max_deflection_{0.524};
  double max_speed_{2.75};
  double max_yaw_speed_{0.5};

  // Publishers
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr
      thrust_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr
      pos_pub_;
  rclcpp_lifecycle::LifecyclePublisher<marine_interfaces::msg::Heartbeat>::SharedPtr
      heartbeat_pub_;

  // Subscribers
  rclcpp::Subscription<marine_interfaces::msg::Helm>::SharedPtr helm_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // PID
  std::unique_ptr<control_toolbox::PidROS> speed_pid_;

  // State
  nav_msgs::msg::Odometry latest_odom_;
  bool have_odom_{false};
  rclcpp::Time last_twist_time_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GazeboHelm>(rclcpp::NodeOptions());
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
