#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "hybrid_a_star/hybrid_a_star.h"
#include "hybrid_a_star/node3d.h"

class HybridAStarNode : public rclcpp::Node {
public:
  HybridAStarNode() : Node("hybrid_a_star_node") {
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
      "planned_path", 10);

    marker_pub_ = this->create_publisher<
      visualization_msgs::msg::MarkerArray>(
        "debug_markers", 10);

    start_sub_ = this->create_subscription<
      geometry_msgs::msg::PoseStamped>(
        "start", 10,
        std::bind(&HybridAStarNode::startCallback, this, std::placeholders::_1));

    goal_sub_ = this->create_subscription<
      geometry_msgs::msg::PoseStamped>(
        "goal", 10,
        std::bind(&HybridAStarNode::goalCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Hybrid A* ROS2 node started");
  }

private:
  HybridAStar planner_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr start_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;

  geometry_msgs::msg::PoseStamped start_, goal_;
  bool has_start_{false}, has_goal_{false};

  void startCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    start_ = *msg;
    has_start_ = true;
    tryPlan();
  }

  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    goal_ = *msg;
    has_goal_ = true;
    tryPlan();
  }

  void tryPlan() {
    if (!has_start_ || !has_goal_) return;

    Node3D start_node(
      start_.pose.position.x,
      start_.pose.position.y,
      tf2::getYaw(start_.pose.orientation));

    Node3D goal_node(
      goal_.pose.position.x,
      goal_.pose.position.y,
      tf2::getYaw(goal_.pose.orientation));

    auto path_nodes = planner_.Search(start_node, goal_node);

    publishPath(path_nodes);
  }

  void publishPath(const std::vector<Node3D>& nodes) {
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    path.header.stamp = this->now();

    for (const auto& n : nodes) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = n.getX();
      pose.pose.position.y = n.getY();
      pose.pose.orientation =
        tf2::toMsg(tf2::Quaternion(0, 0, sin(n.getYaw()/2), cos(n.getYaw()/2)));
      path.poses.push_back(pose);
    }

    path_pub_->publish(path);
  }
};
  
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HybridAStarNode>());
  rclcpp::shutdown();
  return 0;
}
