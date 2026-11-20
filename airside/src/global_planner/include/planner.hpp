#ifndef GLOBAL_PLANNER__PLANNER_HPP_
#define GLOBAL_PLANNER__PLANNER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <airside_interfaces/msg/waypoint.hpp>
#include "global_planner/theta_star.hpp"

namespace GlobalPlanner {

class PlannerNode : public rclcpp::Node {
  public:
      explicit PlannerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  private:
      rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr       octomap_sub_;
      rclcpp::Subscription<airside_interfaces::msg::Waypoint>::SharedPtr goal_sub_;
      rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr  pose_sub_;
      rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr          state_sub_;

      rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr setpoint_pub_;
      rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr             path_pub_;

      rclcpp::TimerBase::SharedPtr setpoint_timer_;
      rclcpp::TimerBase::SharedPtr replan_timer_;

      std::shared_ptr<octomap::OcTree> map_;
      ThetaStar planner_;

      geometry_msgs::msg::PoseStamped current_pose_;
      mavros_msgs::msg::State         mavros_state_;
      octomap::point3d                goal_local_{0.0f, 0.0f, 0.0f};
      std::vector<octomap::point3d>   current_path_;

      bool has_pose_ = false;
      bool has_map_  = false;
      bool has_goal_ = false;
      rclcpp::Time last_plan_time_{0, 0, RCL_ROS_TIME};

      double lookahead_dist_;
      double acceptance_radius_;
      double replan_cooldown_;
      ThetaStar::Config planner_config_;

      void octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg);
      void goalCallback(const airside_interfaces::msg::Waypoint::SharedPtr msg);
      void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
      void stateCallback(const mavros_msgs::msg::State::SharedPtr msg);

      void planPath();
      void setpointTimerCallback();
      void replanTimerCallback();
      bool isPathValid() const;
      void publishSetpoint(const octomap::point3d & target);
      void publishPath();
      octomap::point3d currentPosAsPoint() const;
  };
}

#endif