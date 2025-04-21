#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class FrontierNavigationNode : public rclcpp::Node
{
public:
  FrontierNavigationNode() : Node("frontier_navigation")
  {
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", 10,
      std::bind(&FrontierNavigationNode::map_callback, this, std::placeholders::_1));

    goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/frontier_goal", 10);
  }

private:
  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr map_msg)
  {
    int width = map_msg->info.width;
    bool found = false;
    int frontier_idx = -1;

    // Simple frontier detection: find first cell with value -1 (unknown)
    for (size_t i = 0; i < map_msg->data.size(); ++i)
    {
      if (map_msg->data[i] == -1)
      {
        frontier_idx = i;
        found = true;
        break;
      }
    }

    if (!found)
    {
      RCLCPP_INFO(this->get_logger(), "No frontier found");
      return;
    }

    int x_idx = frontier_idx % width;
    int y_idx = frontier_idx / width;
    double x = map_msg->info.origin.position.x + x_idx * map_msg->info.resolution;
    double y = map_msg->info.origin.position.y + y_idx * map_msg->info.resolution;

    geometry_msgs::msg::PoseStamped goal;
    goal.header = map_msg->header;
    goal.pose.position.x = x;
    goal.pose.position.y = y;
    goal.pose.position.z = 0.0;
    goal.pose.orientation.w = 1.0;

    RCLCPP_INFO(this->get_logger(), "Frontier found at (%.2f, %.2f)", x, y);
    goal_pub_->publish(goal);
  }

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FrontierNavigationNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

