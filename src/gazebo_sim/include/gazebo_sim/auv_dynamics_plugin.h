#ifndef AUV_DYNAMICS_PLUGIN_H
#define AUV_DYNAMICS_PLUGIN_H

#include <google/protobuf/message.h>

#include <gazebo/common/Plugin.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/bool.hpp> // For the reset topic
#include <gazebo_msgs/msg/entity_state.hpp> // For the state publisher
#include <mutex>
#include <thread>
#include "gazebo_sim/vehicle_model.h"

// Forward declare gazebo classes
namespace gazebo {
    namespace physics {
        class Model;
        class World;
    }
    namespace event {
        class Connection;
    }
}

// Type aliases for Eigen
using Vector6d = Eigen::Matrix<double, 6, 1>;

namespace gazebo
{
  class AuvDynamicsPlugin : public ModelPlugin
  {
  public:
    AuvDynamicsPlugin() = default;
    virtual ~AuvDynamicsPlugin();
    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

  private:
    void OnUpdate();
    void OnThrusterCmd(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void OnResetCmd(const std_msgs::msg::Bool::SharedPtr msg);
    void ResetState();

    physics::ModelPtr model_;
    physics::WorldPtr world_;
    event::ConnectionPtr update_connection_;
    double last_update_time_{0.0};

    rclcpp::Node::SharedPtr ros_node_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr thruster_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reset_sub_; // Reset subscriber
    rclcpp::Publisher<gazebo_msgs::msg::EntityState>::SharedPtr state_pub_; // State publisher
    std::unique_ptr<std::thread> ros_thread_;

    std::unique_ptr<VehicleModel> vehicle_model_;
    Vector6d eta_, nu_;
    std::mutex data_mutex_;
    Eigen::Vector<double, 8> thruster_forces_;
  };
}

#endif // AUV_DYNAMICS_PLUGIN_H