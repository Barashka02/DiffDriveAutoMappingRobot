#include "behaviortree_ros2/bt_service_node.hpp"
#include "slam_toolbox/srv/serialize_pose_graph.hpp"
#include "rclcpp/rclcpp.hpp"
#include <string>

using SerializePoseGraph = slam_toolbox::srv::SerializePoseGraph;

class SaveMapAction : public BT::RosServiceNode<SerializePoseGraph>
{
public:
  SaveMapAction(const std::string & name,
                const BT::NodeConfig & config,
                const BT::RosNodeParams & params)
    : RosServiceNode<SerializePoseGraph>(name, config, params)
  {}

  static BT::PortsList providedPorts()
  {
    // Provide an input port "filename" (defaults to "final_map.yaml")
    return providedBasicPorts({ BT::InputPort<std::string>("filename") });
  }

  // For services, we implement setRequest to fill in the request message.
  bool setRequest(typename RosServiceNode<SerializePoseGraph>::Request::SharedPtr & request) override
  {
    std::string filename;
    if(getInput("filename", filename))
      request->filename = filename;
    else
      request->filename = "final_map.yaml";
    return true;
  }

  BT::NodeStatus onResponseReceived(const typename RosServiceNode<SerializePoseGraph>::Response::SharedPtr & response) override
  {
    auto node_ptr = node_.lock();
    if(node_ptr)
    {
      if(response->success)
      {
        RCLCPP_INFO(node_ptr->get_logger(), "Map saved successfully.");
        return BT::NodeStatus::SUCCESS;
      }
      else
      {
        RCLCPP_ERROR(node_ptr->get_logger(), "Map save reported failure.");
        return BT::NodeStatus::FAILURE;
      }
    }
    return BT::NodeStatus::SUCCESS;
  }

  BT::NodeStatus onFailure(BT::ServiceNodeErrorCode error) override
  {
    auto node_ptr = node_.lock();
    if(node_ptr)
      RCLCPP_ERROR(node_ptr->get_logger(), "SaveMap action failed: %d", error);
    return BT::NodeStatus::FAILURE;
  }
};

