#include "CMGPlugin.hh"
#include "Util.hh"
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Matrix3.hh>
#include <ros/ros.h>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <memory>
#include <string>

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(CMGPlugin)

CMGPlugin::CMGPlugin() : ModelPlugin()
{
    // Initialize ROS 2 node
    if (!rclcpp::ok()) {
        rclcpp::init(0, nullptr);
    }
    
    node_ = std::make_shared<rclcpp::Node>("cmg_plugin");
    
    // Initialize CMG parameters
    max_torque_ = 1000.0;  // Nm
    max_angular_velocity_ = 100.0;  // rad/s
    moment_of_inertia_ = 50.0;  // kg*m^2
    
    // Initialize CMG states
    for (int i = 0; i < 4; ++i) {
        cmg_states_[i].angular_velocity = 0.0;
        cmg_states_[i].gimbal_angle = 0.0;
        cmg_states_[i].torque_output = ignition::math::Vector3d::Zero;
        cmg_states_[i].status = CMGStatus::READY;
    }
    
    // Initialize control variables
    target_attitude_ = ignition::math::Quaterniond::Identity;
    current_attitude_ = ignition::math::Quaterniond::Identity;
    attitude_error_ = ignition::math::Vector3d::Zero;
    angular_velocity_ = ignition::math::Vector3d::Zero;
    
    // Initialize PID controllers for attitude control
    pid_roll_.init(10.0, 0.1, 1.0, 0.0, max_torque_);
    pid_pitch_.init(10.0, 0.1, 1.0, 0.0, max_torque_);
    pid_yaw_.init(10.0, 0.1, 1.0, 0.0, max_torque_);
    
    RCLCPP_INFO(node_->get_logger(), "CMG Plugin initialized");
}

CMGPlugin::~CMGPlugin()
{
    if (node_) {
        node_.reset();
    }
}

void CMGPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    // Store the model pointer
    model_ = _model;
    
    // Get the world
    world_ = _model->GetWorld();
    
    // Get the base link
    base_link_ = _model->GetLink();
    if (!base_link_) {
        gzerr << "CMG Plugin: Could not find base link" << std::endl;
        return;
    }
    
    // Load CMG configuration from SDF
    loadCMGConfiguration(_sdf);
    
    // Initialize CMG links and joints
    initializeCMGComponents();
    
    // Set up ROS 2 publishers and subscribers
    setupROS2Communication();
    
    // Connect to the world update event
    update_connection_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&CMGPlugin::OnUpdate, this));
        
    RCLCPP_INFO(node_->get_logger(), "CMG Plugin loaded successfully");
}

void CMGPlugin::loadCMGConfiguration(sdf::ElementPtr _sdf)
{
    if (_sdf->HasElement("max_torque")) {
        max_torque_ = _sdf->Get<double>("max_torque");
    }
    
    if (_sdf->HasElement("max_angular_velocity")) {
        max_angular_velocity_ = _sdf->Get<double>("max_angular_velocity");
    }
    
    if (_sdf->HasElement("moment_of_inertia")) {
        moment_of_inertia_ = _sdf->Get<double>("moment_of_inertia");
    }
    
    if (_sdf->HasElement("pid_gains")) {
        auto pid_element = _sdf->GetElement("pid_gains");
        double kp = pid_element->Get<double>("kp");
        double ki = pid_element->Get<double>("ki");
        double kd = pid_element->Get<double>("kd");
        
        pid_roll_.init(kp, ki, kd, 0.0, max_torque_);
        pid_pitch_.init(kp, ki, kd, 0.0, max_torque_);
        pid_yaw_.init(kp, ki, kd, 0.0, max_torque_);
    }
    
    RCLCPP_INFO(node_->get_logger(), "CMG Configuration loaded - Max Torque: %.1f Nm, Max Angular Velocity: %.1f rad/s", 
                max_torque_, max_angular_velocity_);
}

void CMGPlugin::initializeCMGComponents()
{
    // Find CMG links and joints
    for (int i = 0; i < 4; ++i) {
        std::string cmg_name = "cmg_" + std::to_string(i + 1);
        
        // Find CMG flywheel link
        cmg_links_[i] = model_->GetLink(cmg_name + "_flywheel");
        if (!cmg_links_[i]) {
            gzwarn << "Could not find CMG flywheel link: " << cmg_name + "_flywheel" << std::endl;
            continue;
        }
        
        // Find CMG gimbal joint
        cmg_joints_[i] = model_->GetJoint(cmg_name + "_gimbal");
        if (!cmg_joints_[i]) {
            gzwarn << "Could not find CMG gimbal joint: " << cmg_name + "_gimbal" << std::endl;
            continue;
        }
        
        // Set initial CMG orientation based on position
        setInitialCMGOrientation(i);
        
        RCLCPP_INFO(node_->get_logger(), "CMG %d initialized successfully", i + 1);
    }
}

void CMGPlugin::setInitialCMGOrientation(int cmg_index)
{
    // Set initial CMG orientations for a 4-CMG pyramid configuration
    switch (cmg_index) {
        case 0: // CMG 1: +X direction
            cmg_states_[cmg_index].initial_orientation = ignition::math::Vector3d(1.0, 0.0, 0.0);
            break;
        case 1: // CMG 2: -X direction
            cmg_states_[cmg_index].initial_orientation = ignition::math::Vector3d(-1.0, 0.0, 0.0);
            break;
        case 2: // CMG 3: +Y direction
            cmg_states_[cmg_index].initial_orientation = ignition::math::Vector3d(0.0, 1.0, 0.0);
            break;
        case 3: // CMG 4: -Y direction
            cmg_states_[cmg_index].initial_orientation = ignition::math::Vector3d(0.0, -1.0, 0.0);
            break;
    }
}

void CMGPlugin::setupROS2Communication()
{
    // Publishers
    attitude_pub_ = node_->create_publisher<geometry_msgs::msg::Quaternion>(
        "/gnc/pose_est", 10);
    angular_velocity_pub_ = node_->create_publisher<geometry_msgs::msg::Vector3>(
        "/gnc/angular_velocity", 10);
    cmg_status_pub_ = node_->create_publisher<std_msgs::msg::String>(
        "/gnc/cmg_status", 10);
    torque_pub_ = node_->create_publisher<geometry_msgs::msg::Vector3>(
        "/gnc/applied_torque", 10);
    
    // Subscribers
    attitude_cmd_sub_ = node_->create_subscription<geometry_msgs::msg::Quaternion>(
        "/gnc/pose_ref",
        10,
        std::bind(&CMGPlugin::attitudeCommandCallback, this, std::placeholders::_1));
        
    attitude_control_sub_ = node_->create_subscription<geometry_msgs::msg::Vector3>(
        "/gnc/attitude_control",
        10,
        std::bind(&CMGPlugin::attitudeControlCallback, this, std::placeholders::_1));
        
    cmg_control_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/gnc/cmg_control",
        10,
        std::bind(&CMGPlugin::cmgControlCallback, this, std::placeholders::_1));
    
    RCLCPP_INFO(node_->get_logger(), "ROS 2 communication setup complete");
}

void CMGPlugin::attitudeCommandCallback(const geometry_msgs::msg::Quaternion::SharedPtr msg)
{
    target_attitude_ = ignition::math::Quaterniond(msg->w, msg->x, msg->y, msg->z);
    RCLCPP_DEBUG(node_->get_logger(), "Received attitude command: w=%.3f, x=%.3f, y=%.3f, z=%.3f", 
                 msg->w, msg->x, msg->y, msg->z);
}

void CMGPlugin::attitudeControlCallback(const geometry_msgs::msg::Vector3::SharedPtr msg)
{
    // Direct torque command
    ignition::math::Vector3d torque_cmd(msg->x, msg->y, msg->z);
    applyTorqueCommand(torque_cmd);
}

void CMGPlugin::cmgControlCallback(const std_msgs::msg::String::SharedPtr msg)
{
    if (msg->data == "START") {
        startCMGs();
    } else if (msg->data == "STOP") {
        stopCMGs();
    } else if (msg->data == "RESET") {
        resetCMGs();
    }
}

void CMGPlugin::OnUpdate()
{
    // Update current attitude and angular velocity
    updateState();
    
    // Execute attitude control
    executeAttitudeControl();
    
    // Update CMG states
    updateCMGStates();
    
    // Publish status
    publishStatus();
    
    // Execute ROS 2 callbacks
    rclcpp::spin_some(node_);
}

void CMGPlugin::updateState()
{
    if (!base_link_) return;
    
    // Get current pose and angular velocity
    ignition::math::Pose3d pose = base_link_->WorldPose();
    current_attitude_ = pose.Rot();
    
    ignition::math::Vector3d angular_vel = base_link_->WorldAngularVel();
    angular_velocity_ = angular_vel;
    
    // Calculate attitude error
    ignition::math::Quaterniond error_quat = target_attitude_ * current_attitude_.Inverse();
    attitude_error_ = error_quat.Euler();
}

void CMGPlugin::executeAttitudeControl()
{
    // Calculate control torques using PID controllers
    double roll_torque = pid_roll_.compute(attitude_error_.X(), 0.0, world_->SimTime().Double());
    double pitch_torque = pid_pitch_.compute(attitude_error_.Y(), 0.0, world_->SimTime().Double());
    double yaw_torque = pid_yaw_.compute(attitude_error_.Z(), 0.0, world_->SimTime().Double());
    
    ignition::math::Vector3d control_torque(roll_torque, pitch_torque, yaw_torque);
    
    // Apply torque through CMGs
    applyTorqueCommand(control_torque);
}

void CMGPlugin::applyTorqueCommand(const ignition::math::Vector3d& torque_cmd)
{
    // Limit torque to maximum capability
    ignition::math::Vector3d limited_torque = torque_cmd;
    double torque_magnitude = limited_torque.Length();
    
    if (torque_magnitude > max_torque_) {
        limited_torque = limited_torque * (max_torque_ / torque_magnitude);
    }
    
    // Calculate CMG gimbal rates using pseudo-inverse method
    std::vector<double> gimbal_rates = calculateGimbalRates(limited_torque);
    
    // Apply gimbal rates to CMG joints
    for (int i = 0; i < 4; ++i) {
        if (cmg_joints_[i] && cmg_states_[i].status == CMGStatus::READY) {
            double current_rate = cmg_joints_[i]->GetVelocity(0);
            double target_rate = gimbal_rates[i];
            
            // Apply rate limit
            double max_rate_change = 1.0; // rad/s^2
            double rate_change = target_rate - current_rate;
            if (std::abs(rate_change) > max_rate_change) {
                rate_change = (rate_change > 0) ? max_rate_change : -max_rate_change;
            }
            
            double new_rate = current_rate + rate_change;
            cmg_joints_[i]->SetVelocity(0, new_rate);
            
            // Update CMG state
            cmg_states_[i].gimbal_angle += new_rate * world_->Physics()->GetMaxStepSize();
            cmg_states_[i].torque_output = calculateCMGTorque(i, new_rate);
        }
    }
    
    // Apply the torque to the base link
    base_link_->AddTorque(limited_torque);
}

std::vector<double> CMGPlugin::calculateGimbalRates(const ignition::math::Vector3d& desired_torque)
{
    // Simplified CMG Jacobian matrix for 4-CMG pyramid configuration
    // This is a simplified version - in practice, you'd use a more sophisticated algorithm
    
    std::vector<double> gimbal_rates(4, 0.0);
    
    // Calculate gimbal rates based on desired torque and current CMG orientations
    for (int i = 0; i < 4; ++i) {
        if (cmg_states_[i].status == CMGStatus::READY) {
            // Simple proportional control for each CMG
            ignition::math::Vector3d cmg_axis = cmg_states_[i].initial_orientation;
            double gimbal_angle = cmg_states_[i].gimbal_angle;
            
            // Rotate CMG axis by gimbal angle
            ignition::math::Matrix3d rotation_matrix;
            rotation_matrix.SetFromAxis(ignition::math::Vector3d(0, 0, 1), gimbal_angle);
            ignition::math::Vector3d rotated_axis = rotation_matrix * cmg_axis;
            
            // Calculate contribution to desired torque
            double contribution = rotated_axis.Dot(desired_torque);
            gimbal_rates[i] = contribution * 0.1; // Proportional gain
        }
    }
    
    return gimbal_rates;
}

ignition::math::Vector3d CMGPlugin::calculateCMGTorque(int cmg_index, double gimbal_rate)
{
    if (cmg_index < 0 || cmg_index >= 4) {
        return ignition::math::Vector3d::Zero;
    }
    
    // Calculate CMG torque based on flywheel angular momentum and gimbal rate
    double flywheel_angular_momentum = cmg_states_[cmg_index].angular_velocity * moment_of_inertia_;
    ignition::math::Vector3d cmg_axis = cmg_states_[cmg_index].initial_orientation;
    
    // Torque = H × ω_gimbal
    ignition::math::Vector3d torque = cmg_axis.Cross(ignition::math::Vector3d(0, 0, gimbal_rate)) * flywheel_angular_momentum;
    
    return torque;
}

void CMGPlugin::updateCMGStates()
{
    for (int i = 0; i < 4; ++i) {
        if (cmg_links_[i] && cmg_joints_[i]) {
            // Update flywheel angular velocity
            cmg_states_[i].angular_velocity = cmg_links_[i]->RelativeAngularVel().Z();
            
            // Check for CMG saturation or failure conditions
            if (std::abs(cmg_states_[i].angular_velocity) > max_angular_velocity_) {
                cmg_states_[i].status = CMGStatus::SATURATED;
                RCLCPP_WARN(node_->get_logger(), "CMG %d saturated at %.1f rad/s", i + 1, cmg_states_[i].angular_velocity);
            } else if (cmg_states_[i].status == CMGStatus::SATURATED) {
                cmg_states_[i].status = CMGStatus::READY;
            }
        }
    }
}

void CMGPlugin::startCMGs()
{
    for (int i = 0; i < 4; ++i) {
        if (cmg_links_[i]) {
            // Set initial flywheel angular velocity
            cmg_states_[i].angular_velocity = max_angular_velocity_ * 0.8; // 80% of max
            cmg_links_[i]->SetAngularVel(ignition::math::Vector3d(0, 0, cmg_states_[i].angular_velocity));
            cmg_states_[i].status = CMGStatus::READY;
        }
    }
    RCLCPP_INFO(node_->get_logger(), "All CMGs started");
}

void CMGPlugin::stopCMGs()
{
    for (int i = 0; i < 4; ++i) {
        if (cmg_links_[i]) {
            cmg_links_[i]->SetAngularVel(ignition::math::Vector3d::Zero);
            cmg_states_[i].angular_velocity = 0.0;
            cmg_states_[i].status = CMGStatus::STOPPED;
        }
    }
    RCLCPP_INFO(node_->get_logger(), "All CMGs stopped");
}

void CMGPlugin::resetCMGs()
{
    for (int i = 0; i < 4; ++i) {
        if (cmg_joints_[i]) {
            cmg_joints_[i]->Reset();
            cmg_states_[i].gimbal_angle = 0.0;
            cmg_states_[i].torque_output = ignition::math::Vector3d::Zero;
        }
    }
    RCLCPP_INFO(node_->get_logger(), "All CMGs reset");
}

void CMGPlugin::publishStatus()
{
    // Publish current attitude
    auto attitude_msg = geometry_msgs::msg::Quaternion();
    attitude_msg.w = current_attitude_.W();
    attitude_msg.x = current_attitude_.X();
    attitude_msg.y = current_attitude_.Y();
    attitude_msg.z = current_attitude_.Z();
    attitude_pub_->publish(attitude_msg);
    
    // Publish angular velocity
    auto angular_vel_msg = geometry_msgs::msg::Vector3();
    angular_vel_msg.x = angular_velocity_.X();
    angular_vel_msg.y = angular_velocity_.Y();
    angular_vel_msg.z = angular_velocity_.Z();
    angular_velocity_pub_->publish(angular_vel_msg);
    
    // Publish CMG status
    std::string status_str = "CMG Status: ";
    for (int i = 0; i < 4; ++i) {
        status_str += "CMG" + std::to_string(i + 1) + "=";
        switch (cmg_states_[i].status) {
            case CMGStatus::READY:
                status_str += "READY";
                break;
            case CMGStatus::SATURATED:
                status_str += "SATURATED";
                break;
            case CMGStatus::STOPPED:
                status_str += "STOPPED";
                break;
            case CMGStatus::FAILED:
                status_str += "FAILED";
                break;
        }
        if (i < 3) status_str += ", ";
    }
    
    auto cmg_status_msg = std_msgs::msg::String();
    cmg_status_msg.data = status_str;
    cmg_status_pub_->publish(cmg_status_msg);
    
    // Publish applied torque
    auto torque_msg = geometry_msgs::msg::Vector3();
    ignition::math::Vector3d total_torque = ignition::math::Vector3d::Zero;
    for (int i = 0; i < 4; ++i) {
        total_torque += cmg_states_[i].torque_output;
    }
    torque_msg.x = total_torque.X();
    torque_msg.y = total_torque.Y();
    torque_msg.z = total_torque.Z();
    torque_pub_->publish(torque_msg);
}