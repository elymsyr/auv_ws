#include <google/protobuf/message.h>

#include "gazebo_sim/auv_dynamics_plugin.h"
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/common/Events.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>

#include <gazebo_msgs/msg/entity_state.hpp>

namespace gazebo
{
  // Destructor
  AuvDynamicsPlugin::~AuvDynamicsPlugin()
  {
    this->update_connection_.reset();
    if (this->ros_thread_) {
        this->ros_thread_->join();
    }
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  // Load function
  void AuvDynamicsPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {
    this->model_ = _model;
    this->world_ = _model->GetWorld();

    // --- 1. Load Configuration ---
    if (!_sdf->HasElement("config_file")) {
      gzerr << "AUV Dynamics Plugin missing <config_file> element." << std::endl;
      return;
    }
    std::string config_path = _sdf->Get<std::string>("config_file");
    
    try {
      this->vehicle_model_ = std::make_unique<VehicleModel>(config_path);
    } catch (const std::exception& e) {
      gzerr << "Failed to initialize VehicleModel: " << e.what() << std::endl;
      return;
    }

    // --- 2. Initialize ROS 2 ---
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    this->ros_node_ = rclcpp::Node::make_shared("auv_dynamics_plugin");
    
    // Create Subscribers
    this->thruster_sub_ = this->ros_node_->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/ucat/thruster_cmd", 10,
      std::bind(&AuvDynamicsPlugin::OnThrusterCmd, this, std::placeholders::_1));
      
    this->reset_sub_ = this->ros_node_->create_subscription<std_msgs::msg::Bool>(
      "/ucat/reset", 10,
      std::bind(&AuvDynamicsPlugin::OnResetCmd, this, std::placeholders::_1));

    // Create Publisher
    this->state_pub_ = this->ros_node_->create_publisher<gazebo_msgs::msg::EntityState>("/ucat/state", 10);

    // Spin the ROS node in a separate thread
    this->ros_thread_ = std::make_unique<std::thread>([&]() {
      rclcpp::spin(this->ros_node_);
    });
    
    // --- 3. Connect to Gazebo's World Update Event ---
    this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&AuvDynamicsPlugin::OnUpdate, this));
      
    // --- 4. Initialize State ---
    this->ResetState();
    gzmsg << "AUV Dynamics Plugin Loaded for model [" << _model->GetName() << "]" << std::endl;
  }

  // Reset function
  void AuvDynamicsPlugin::ResetState()
  {
    std::lock_guard<std::mutex> lock(this->data_mutex_);
    this->eta_ = Vector6d::Zero();
    this->nu_ = Vector6d::Zero();
    this->eta_(2) = 5.0; // Start 5m deep (NED convention, Z is down)
    this->thruster_forces_.setZero();
    this->last_update_time_ = this->world_->SimTime().Double();
    
    // Instantly move the model in Gazebo to the reset pose
    ignition::math::Pose3d reset_pose(0, 0, -5.0, 0, 0, 0);
    this->model_->SetWorldPose(reset_pose);
    this->model_->SetLinearVel(ignition::math::Vector3d::Zero);
    this->model_->SetAngularVel(ignition::math::Vector3d::Zero);
  }
  
  // OnResetCmd callback
  void AuvDynamicsPlugin::OnResetCmd(const std_msgs::msg::Bool::SharedPtr msg)
  {
      if (msg->data) {
          RCLCPP_INFO(this->ros_node_->get_logger(), "Reset command received.");
          this->ResetState();
      }
  }

  // OnUpdate (main loop)
  void AuvDynamicsPlugin::OnUpdate()
  {
    double current_time = this->world_->SimTime().Double();
    double dt = current_time - this->last_update_time_;
    if (dt <= 0) return;
    this->last_update_time_ = current_time;

    Eigen::Vector<double, 8> current_thruster_forces;
    {
      std::lock_guard<std::mutex> lock(this->data_mutex_);
      current_thruster_forces = this->thruster_forces_;
    }

    // --- Physics Calculation ---
    auto [eta_dot, nu_dot] = this->vehicle_model_->dynamics(this->eta_, this->nu_, current_thruster_forces);
    this->eta_ += eta_dot * dt;
    this->nu_ += nu_dot * dt;

    // --- Gazebo Update ---
    ignition::math::Pose3d new_pose;
    new_pose.Pos().X(this->eta_(0));
    new_pose.Pos().Y(-this->eta_(1)); // NED to ENU
    new_pose.Pos().Z(-this->eta_(2)); // NED to ENU
    
    ignition::math::Quaterniond q;
    q.Euler(this->eta_(3), -this->eta_(4), -this->eta_(5)); // NED to ENU
    new_pose.Rot() = q;
    this->model_->SetWorldPose(new_pose);

    // --- Publish State ---
    gazebo_msgs::msg::EntityState state_msg;
    state_msg.name = this->model_->GetName();
    state_msg.pose.position.x = new_pose.Pos().X();
    state_msg.pose.position.y = new_pose.Pos().Y();
    state_msg.pose.position.z = new_pose.Pos().Z();
    state_msg.pose.orientation.x = new_pose.Rot().X();
    state_msg.pose.orientation.y = new_pose.Rot().Y();
    state_msg.pose.orientation.z = new_pose.Rot().Z();
    state_msg.pose.orientation.w = new_pose.Rot().W();
    
    // Body-frame velocities
    state_msg.twist.linear.x = this->nu_(0);
    state_msg.twist.linear.y = this->nu_(1);
    state_msg.twist.linear.z = this->nu_(2);
    state_msg.twist.angular.x = this->nu_(3);
    state_msg.twist.angular.y = this->nu_(4);
    state_msg.twist.angular.z = this->nu_(5);
    
    this->state_pub_->publish(state_msg);
  }

  // OnThrusterCmd callback
  void AuvDynamicsPlugin::OnThrusterCmd(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    if (msg->data.size() == 8) {
      std::lock_guard<std::mutex> lock(this->data_mutex_);
      for (size_t i = 0; i < 8; ++i) { this->thruster_forces_(i) = msg->data[i]; }
    } else {
      RCLCPP_WARN_THROTTLE(this->ros_node_->get_logger(), *this->ros_node_->get_clock(), 1000,
        "Received thruster command with %zu elements, expected 8.", msg->data.size());
    }
  }

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(AuvDynamicsPlugin)
}