#ifndef GLOBAL_PLANNER__PLANNER_HPP_
#define GLOBAL_PLANNER__PLANNER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/global_position_target.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/float64.hpp>
#include <airside_interfaces/msg/waypoint.hpp>
#include "theta_star.hpp"

namespace GlobalPlanner {

class PlannerNode : public rclcpp::Node {
  public:
      explicit PlannerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  private:
      rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr       octomap_sub_;
      rclcpp::Subscription<airside_interfaces::msg::Waypoint>::SharedPtr goal_sub_;
      rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr      gps_sub_;
      rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr           rel_alt_sub_;
      rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr          state_sub_;

      rclcpp::Publisher<mavros_msgs::msg::GlobalPositionTarget>::SharedPtr setpoint_pub_;
      rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr             path_pub_;

      rclcpp::TimerBase::SharedPtr setpoint_timer_;
      rclcpp::TimerBase::SharedPtr replan_timer_;

      std::shared_ptr<octomap::OcTree> map_;
      ThetaStar planner_;

      sensor_msgs::msg::NavSatFix current_gps_;
      double current_rel_alt_m_ = 0.0;
      bool has_valid_origin_ = false;
      double origin_lat_deg_ = 0.0;
      double origin_lon_deg_ = 0.0;
      double origin_rel_alt_m_ = 0.0;
      mavros_msgs::msg::State         mavros_state_;
      octomap::point3d                goal_local_{0.0f, 0.0f, 0.0f};
      double goal_lat_deg_ = 0.0;
      double goal_lon_deg_ = 0.0;
      double goal_rel_alt_m_ = 0.0;
      std::vector<octomap::point3d>   current_path_;

      bool has_gps_ = false;
      bool has_rel_alt_ = false;
      bool has_map_  = false;
      bool has_goal_ = false;
      rclcpp::Time last_plan_time_{0, 0, RCL_ROS_TIME};

      double lookahead_dist_;
      double acceptance_radius_;
      double replan_cooldown_;
      ThetaStar::Config planner_config_;

      void octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg);
      void goalCallback(const airside_interfaces::msg::Waypoint::SharedPtr msg);
      void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
      void relAltCallback(const std_msgs::msg::Float64::SharedPtr msg);
      void stateCallback(const mavros_msgs::msg::State::SharedPtr msg);

      void planPath();
      void setpointTimerCallback();
      void replanTimerCallback();
      bool isPathValid() const;
      void publishSetpoint(const octomap::point3d & target);
      void publishPath();
      octomap::point3d currentPosAsPoint() const;
      octomap::point3d gpsToLocal(double lat_deg, double lon_deg, double rel_alt_m) const;
      void localToGps(const octomap::point3d & local, double & lat_deg, double & lon_deg, double & rel_alt_m) const;
  };
}

#endif