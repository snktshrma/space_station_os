#ifndef THERMAL_SOLVER_NODE_HPP_
#define THERMAL_SOLVER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <urdf/model.h>
#include <unordered_map>
#include <vector>
#include <string>
#include <memory>
#include <thread>
#include <chrono>
#include "std_msgs/msg/string.hpp"
#include <random>
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "space_station_thermal_control/msg/thermal_node_data_array.hpp"
#include "space_station_thermal_control/msg/thermal_node_data.hpp"
#include "space_station_thermal_control/msg/thermal_link_flows_array.hpp"
#include "space_station_thermal_control/msg/thermal_link_flows.hpp"
#include "space_station_thermal_control/msg/solar_panels_q.hpp"
#include "space_station_thermal_control/srv/node_heat_flow.hpp"
#include <chrono>

using namespace std::chrono_literals;
struct ThermalNode
{
  std::string name;
  double temperature;
  double heat_capacity;
  double internal_power;
};

struct ThermalLink
{
  std::string from;
  std::string to;
  double conductance;
  std::string joint_name;
};

class ThermalSolverNode : public rclcpp::Node
{
public:
  ThermalSolverNode();
  ~ThermalSolverNode();

private:
  void parseURDF(const std::string &urdf_string);
  void updateSimulation();
  double compute_dTdt(const std::string &name, const std::unordered_map<std::string, double> &temps);
  void coolingCallback();
  void solarHeatCallback(const space_station_thermal_control::msg::SolarPanelsQ::SharedPtr msg);

  std::unordered_map<std::string, ThermalNode> thermal_nodes_;
  std::vector<ThermalLink> thermal_links_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr urdf_sub_;
  rclcpp::Publisher<space_station_thermal_control::msg::ThermalNodeDataArray>::SharedPtr node_pub_;
  rclcpp::Publisher<space_station_thermal_control::msg::ThermalLinkFlowsArray>::SharedPtr link_pub_;
  rclcpp::Client<space_station_thermal_control::srv::NodeHeatFlow>::SharedPtr cooling_client_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr diag_pub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_;
  bool enable_failure_ = false;
  bool enable_cooling_ = true;

  bool cooling_active_ = false;
  double cooling_rate_ = 10.0;
  double cooling_trigger_threshold_ = 330.0;
  double max_temp_threshold_ = 420.0;
  double thermal_update_dt_ = 0.5;
  double sink_temperature_ = 293.15;

  double init_temp_low_ = 290.0;
  double init_temp_high_ = 310.0;
  double capacity_low_ = 500.0;
  double capacity_high_ = 1500.0;
  double power_low_ = 30.0;
  double power_high_ = 60.0;
  double conductance_low_ = 0.05;
  double conductance_high_ = 2.0;

  double avg_temperature_ = 0.0;
  double avg_internal_power_ = 0.0;
  std::unordered_map<std::string, double> initial_temperatures_;

  std::unordered_map<std::string, std::string> panel_node_map_;
  std::unordered_map<std::string, double> solar_heat_input_;
  rclcpp::Subscription<space_station_thermal_control::msg::SolarPanelsQ>::SharedPtr solar_sub_;

  std::default_random_engine rng_;
};

#endif  // THERMAL_SOLVER_NODE_HPP_
