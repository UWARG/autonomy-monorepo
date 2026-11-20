#include <chrono>
#include <cmath>
#include <algorithm>
#include <octomap_msgs/conversions.h>
#include "global_planner/planner.hpp"

namespace GlobalPlanner{
    PlannerNode::PlannerNode(const rclcpp::NodeOptions & options): rclcpp::Node("global_planner", options){
        this->declare_parameter("step_size", rclcpp::ParameterValue{});
        this->declare_parameter("safety_margin", rclcpp::ParameterValue{});
        this->declare_parameter("min_altitude", rclcpp::ParameterValue{});
        this->declare_parameter("max_altitude", rclcpp::ParameterValue{});
        this->declare_parameter("max_planning_time", rclcpp::ParameterValue{});
        this->declare_parameter("lookahead_dist", rclcpp::ParameterValue{});
        this->declare_parameter("acceptance_radius", rclcpp::ParameterValue{});
        this->declare_parameter("replan_cooldown", rclcpp::ParameterValue{});
        this->declare_parameter("setpoint_rate", rclcpp::ParameterValue{});

        planner_config_.step_size = this->get_parameter("step_size").as_double();
        planner_config_.safety_margin = this->get_parameter("safety_margin").as_double();
        planner_config_.min_altitude = this->get_parameter("min_altitude").as_double();
        planner_config_.max_altitude = this->get_parameter("max_altitude").as_double();
        planner_config_.max_planning_time = this->get_parameter("max_planning_time").as_double();
        lookahead_dist_ = this->get_parameter("lookahead_dist").as_double();
        acceptance_radius_ = this->get_parameter("acceptance_radius").as_double();
        replan_cooldown_ = this->get_parameter("replan_cooldown").as_double();

        planner_.setConfig(planner_config_);

        auto reliable_qos = rclcpp::QoS(1).reliable();
        auto besteffort_qos = rclcpp::QoS(1).best_effort();

        octomap_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>("/rtabmap/octomap_binary", reliable_qos, std::bind(&PlannerNode::octomapCallback, this, std::placeholders::_1));
        goal_sub_ = this->create_subscription<airside_interfaces::msg::Waypoint>("/position_controller/goal", reliable_qos, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/mavros/local_position/pose", besteffort_qos, std::bind(&PlannerNode::poseCallback, this, std::placeholders::_1));
        state_sub_ = this->create_subscription<mavros_msgs::msg::State>("/mavros/state", reliable_qos, std::bind(&PlannerNode::stateCallback, this, std::placeholders::_1));

        setpoint_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/mavros/setpoint_position/local", reliable_qos);

        auto transient_qos = rclcpp::QoS(1).reliable().transient_local();
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/global_planner/path", transient_qos);

        const double rate_hz = this->get_parameter("setpoint_rate").as_double();
        const auto setpoint_period = std::chrono::duration<double>(1.0 / rate_hz);
        setpoint_timer_ = this->create_wall_timer(setpoint_period, std::bind(&PlannerNode::setpointTimerCallback, this));
        replan_timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&PlannerNode::replanTimerCallback, this));

        RCLCPP_INFO(this->get_logger(), "GlobalPlanner node initialized.");
    }

    void PlannerNode::poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
        current_pose_ = *msg;
        has_pose_ = true;
    }

    void PlannerNode::stateCallback(const mavros_msgs::msg::State::SharedPtr msg){
        mavros_state_ = *msg;
    }

    void PlannerNode::goalCallback(const airside_interfaces::msg::Waypoint::SharedPtr msg){
        goal_local_ = octomap::point3d(static_cast<float>(msg->pose.position.x), static_cast<float>(msg->pose.position.y), static_cast<float>(msg->pose.position.z));
        has_goal_ = true;

        RCLCPP_INFO(this->get_logger(), "New goal received: (%.2f, %.2f, %.2f)", goal_local_.x(), goal_local_.y(), goal_local_.z());

        if(has_pose_ && has_map_){
            planPath();
        }
    }

    void PlannerNode::octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg){
        if(msg->data.empty()){
            RCLCPP_WARN(this->get_logger(), "Received empty OctoMap message.");
            return;
        }

        octomap::AbstractOcTree* abstract_tree = octomap_msgs::msgToMap(*msg);
        auto* octree = dynamic_cast<octomap::OcTree*>(abstract_tree);
        if(!octree){
            delete abstract_tree;
            RCLCPP_WARN(this->get_logger(), "Failed to decode OctoMap message.");
            return;
        }

        map_ = std::shared_ptr<octomap::OcTree>(octree);
        has_map_ = true;

        if(!current_path_.empty() && !isPathValid()){
            RCLCPP_INFO(this->get_logger(), "New obstacle blocks current path. Replanning.");
            planPath();
        }

        if(has_goal_ && has_pose_ && current_path_.empty())
            planPath();
    }

    void PlannerNode::planPath(){
        const octomap::point3d start = currentPosAsPoint();

        RCLCPP_INFO(this->get_logger(),
            "Planning from (%.2f, %.2f, %.2f) to (%.2f, %.2f, %.2f)", start.x(), start.y(), start.z(), goal_local_.x(), goal_local_.y(), goal_local_.z());

        auto new_path = planner_.plan(start, goal_local_, *map_);

        if(new_path.empty()){
            RCLCPP_WARN(this->get_logger(), "Theta* found no valid path. Will retry on next map update.");
            return;
        }

        current_path_ = std::move(new_path);
        last_plan_time_ = this->now();

        RCLCPP_INFO(this->get_logger(),
            "Path found: %zu waypoints.", current_path_.size());

        publishPath();
    }

    void PlannerNode::setpointTimerCallback(){
        if(!has_pose_)
            return;

        const octomap::point3d current_pt = currentPosAsPoint();

        if(current_path_.empty()){
            publishSetpoint(current_pt);
            return;
        }

        const octomap::point3d & goal_pt = current_path_.back();
        if(current_pt.distance(goal_pt) <= acceptance_radius_){
            publishSetpoint(goal_pt);
            return;
        }

        octomap::point3d target = goal_pt;
        for(const auto & wp:current_path_){
            if(current_pt.distance(wp) >= lookahead_dist_){
                target = wp;
                break;
            }
        }

        publishSetpoint(target);
    }

    void PlannerNode::replanTimerCallback(){
        if(current_path_.empty() || !has_pose_ || !has_map_)
            return;

        const double elapsed = (this->now() - last_plan_time_).seconds();
        if(elapsed < replan_cooldown_)
            return;

        if(!isPathValid()){
            RCLCPP_INFO(this->get_logger(), "Obstacle blocks path. Re-planning.");
            planPath();
        }
    }

    bool PlannerNode::isPathValid() const{
        if(current_path_.size() < 2 || !map_)
            return true;

        const octomap::point3d current_pt = currentPosAsPoint();
        size_t start_idx = 0;
        for(size_t i = 0; i < current_path_.size(); ++i){
            if(current_pt.distance(current_path_[i]) >= lookahead_dist_){
                if(i > 0)
                    start_idx = i-1;
                else
                    start_idx = 0;

                break;
            }
        }

        for(size_t i = start_idx; i + 1 < current_path_.size(); ++i){
            if(!planner_.lineOfSight(current_path_[i], current_path_[i + 1], *map_))
                return false;
        }

        return true;
    }

    void PlannerNode::publishSetpoint(const octomap::point3d & target){
        geometry_msgs::msg::PoseStamped sp;
        sp.header.stamp= this->now();
        sp.header.frame_id = "map";
        sp.pose.position.x = target.x();
        sp.pose.position.y = target.y();
        sp.pose.position.z = target.z();
        sp.pose.orientation.w = 1.0;
        setpoint_pub_->publish(sp);
    }

    void PlannerNode::publishPath(){
        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp= this->now();
        path_msg.header.frame_id = "map";
        for(const auto & pt:current_path_){
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path_msg.header;
            pose.pose.position.x = pt.x();
            pose.pose.position.y = pt.y();
            pose.pose.position.z = pt.z();
            pose.pose.orientation.w = 1.0;
            path_msg.poses.push_back(pose);
        }
        path_pub_->publish(path_msg);
    }

    octomap::point3d PlannerNode::currentPosAsPoint() const{
        return octomap::point3d(static_cast<float>(current_pose_.pose.position.x), static_cast<float>(current_pose_.pose.position.y), static_cast<float>(current_pose_.pose.position.z));
    }
}

int main(int argc, char ** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GlobalPlanner::PlannerNode>());
    rclcpp::shutdown();
    return 0;
}
