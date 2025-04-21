/*
This is a demo for BehaviorTree.CPP BehaviorTree.ROS2 
first you must ros2 run the fibonacci and add_two_ints servers 
the run demo and it will go through the behaviour tree
*/
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_ros2/bt_service_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/srv/get_map.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include <cmath>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/utils.hpp"
#include "ros_interfaces/srv/get_pose.hpp"
#include <queue>
#include <set>
#include <tf2/utils.h>
#include <visualization_msgs/msg/marker.hpp>
#include "slam_toolbox/srv/serialize_pose_graph.hpp"

namespace chr = std::chrono;
using namespace BT;
using PoseMsg = geometry_msgs::msg::Pose;
using Point = geometry_msgs::msg::Point;
using OccupancyGrid = nav_msgs::msg::OccupancyGrid;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GetMap = nav_msgs::srv::GetMap;
using ComputePathToPose = nav2_msgs::action::ComputePathToPose;
using Pose = geometry_msgs::msg::Pose;
using GetPose = ros_interfaces::srv::GetPose;
using Path = nav_msgs::msg::Path;\
// using GetPose = ros_interfaces::srv::GetPose;

// Simple function that return a NodeStatus
NodeStatus CheckBattery()
{
    std::cout << "[ Battery: OK ]" << std::endl;
    return NodeStatus::SUCCESS;
}

// SyncActionNode (synchronous action) with an input port.
/*
   a SyncActionNode can only return SUCCESS or FAILURE
   it cannot return RUNNING. so it is only good for simple, fast operations
 */
// SyncActionNode (synchronous action) with an input port.
class SaySomething : public SyncActionNode
{
    public:
        // If your Node has ports, you must use this constructor signature 
        SaySomething(const std::string& name, const NodeConfig& config)
            : SyncActionNode(name, config)
        { }

        // It is mandatory to define this STATIC method.
        static PortsList providedPorts()
        {
            // This action has a single input port called "message"
            return { InputPort<std::string>("message") };
        }

        // Override the virtual function tick()
        NodeStatus tick() override
        {
            Expected<std::string> msg = getInput<std::string>("message");
            // Check if expected is valid. If not, throw its error
            if (!msg)
            {
                throw RuntimeError("missing required input [message]: ", 
                        msg.error() );
            }
            // use the method value() to extract the valid message.
            std::cout << "Robot says: " << msg.value() << std::endl;
            return NodeStatus::SUCCESS;
        }
};
class Wait : public StatefulActionNode
{
    public:
        Wait(const std::string& name, const NodeConfiguration& config)
            : StatefulActionNode(name, config)
        {}
        static PortsList providedPorts()
        {
            return{ InputPort<unsigned>("seconds") };
        }
        NodeStatus onStart() override;
        NodeStatus onRunning() override;
        void onHalted() override;
    private:
    	unsigned _seconds;
        chr::system_clock::time_point _completion_time;
};
NodeStatus Wait::onStart()
{
    if ( !getInput<unsigned>("seconds", _seconds))
    {
        throw RuntimeError("missing required input [seconds]");
    }
    printf("[ Wait: ] seconds = %d\n",_seconds);
    _completion_time = chr::system_clock::now() + chr::milliseconds(_seconds*1000);
    return NodeStatus::RUNNING;
}

NodeStatus Wait::onRunning()
{
    std::this_thread::sleep_for(chr::milliseconds(1000));
    if(chr::system_clock::now() >= _completion_time)
    {
        std::cout << "[ Wait: FINISHED ]" << std::endl;
        return NodeStatus::SUCCESS;
    }
    return NodeStatus::RUNNING;
}
void Wait::onHalted()
{
    printf("[ Wait: ABORTED ]");
}


class NavigateToPoseAction: public RosActionNode<NavigateToPose>
{
public:
    NavigateToPoseAction(const std::string& name,
                         const NodeConfig& conf,
                         const RosNodeParams& params)
        : RosActionNode<NavigateToPose>(name, conf, params)
    {}

    static PortsList providedPorts()
    {
        return providedBasicPorts({
            InputPort<Pose>("pose")  // Accept full pose now
        });
    }

    bool setGoal(RosActionNode::Goal& goal) override
    {
        Pose target_pose;
        if (!getInput("pose", target_pose))
        {
            throw RuntimeError("Missing required input [pose]");
        }

        goal.pose.header.frame_id = "map";
        goal.pose.pose = target_pose;
        return true;
    }

    NodeStatus onResultReceived(const WrappedResult& wr) override
    {
        std::cout << "NavigateToPose Goal reached! " << wr.result << "\n";
        return NodeStatus::SUCCESS;
    }

    NodeStatus onFailure(ActionNodeErrorCode error) override
    {
        std::cout << "Error in NavigateToPose: " << error << "\n";
        return NodeStatus::FAILURE;
    }

    NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback)
    {
        std::cout << "NavigateToPose feedback: " << feedback << "\n";
        return NodeStatus::RUNNING;
    }
};





class MapService: public RosServiceNode<GetMap>
{
    public:
        MapService(const std::string& name,
                const NodeConfig& conf,
                const RosNodeParams& params)
            : RosServiceNode<GetMap>(name, conf, params)
        {}
        static PortsList providedPorts()
        {
            return providedBasicPorts({OutputPort<OccupancyGrid>("map")});
        }
        bool setRequest(Request::SharedPtr& request) override
        {
            (void)request;
            return true;
        }
        NodeStatus onResponseReceived(const Response::SharedPtr& response) override
        {
            setOutput("map",response->map);
            (void)response;
            return NodeStatus::SUCCESS;
        }
        virtual NodeStatus onFailure(ServiceNodeErrorCode error) override
        {
            (void)error;
            return NodeStatus::FAILURE;
        }
};

class PoseService : public RosServiceNode<GetPose>
{
public:
    PoseService(const std::string& name,
                const NodeConfig& conf,
                const RosNodeParams& params)
        : RosServiceNode<GetPose>(name, conf, params)
    {}

    static PortsList providedPorts()
    {
        return providedBasicPorts({ OutputPort<Pose>("pose") });
    }

    bool setRequest(GetPose::Request::SharedPtr& request) override
    {
        (void)request;
        return true;
    }

    NodeStatus onResponseReceived(const GetPose::Response::SharedPtr& response) override
    {
        setOutput("pose", response->pose);
        return NodeStatus::SUCCESS;
    }

    NodeStatus onFailure(ServiceNodeErrorCode /*error*/) override
    {
        return NodeStatus::FAILURE;
    }
};





class MapFinished : public SyncActionNode
{
    public:
        // If your Node has ports, you must use this constructor signature 
        MapFinished(const std::string& name, const NodeConfig& config)
            : SyncActionNode(name, config)
        { }

        // It is mandatory to define this STATIC method.
        static PortsList providedPorts()
        {
            // This action has a single input port called "message"
            return { InputPort<OccupancyGrid>("map") };
        }

        // Override the virtual function tick()
        NodeStatus tick() override
        {
            Expected<OccupancyGrid> msg
                = getInput<OccupancyGrid>("map");
            // Check if expected is valid. If not, throw its error
            if (!msg)
            {
                throw RuntimeError("missing required input [map]: ", 
                        msg.error() );
            }
            OccupancyGrid map=msg.value();
            unsigned n=map.info.width*map.info.height;
            unsigned i;
            unsigned cnt=0;
            std::cout<<"Map info: "<< map.info.origin.position.x << ", " 
                << map.info.origin.position.x << ","<<n<<std::endl;
            for(i=0;i<n;++i) {
                if(map.data[i] == -1) {
                    cnt++;
                }
            }
            float percent_done = (float)cnt/(float)n;
            std::cout << "Map is "<< percent_done << " known"<<std::endl;
            if(percent_done > 0.5) {
                std::cout << "Map is finished!\n";
                return NodeStatus::SUCCESS;
            } else {
                return NodeStatus::FAILURE;
            }
        }
};

class ComputePathToPoseAction: public RosActionNode<ComputePathToPose>
{
    public:
        ComputePathToPoseAction(const std::string& name,
                const NodeConfig& conf,
                const RosNodeParams& params)
            : RosActionNode<ComputePathToPose>(name, conf, params)
        {}
        static PortsList providedPorts()
        {
            return {
               InputPort<PoseMsg>("goal_pose"),
               InputPort<PoseMsg>("start_pose"),
               OutputPort<uint16_t>("error_code"),
               OutputPort<Path>("path")
            };
        }
        bool setGoal(RosActionNode::Goal& goal) override
        {
            getInput("goal_pose", goal.goal.pose);
            getInput("start_pose", goal.start.pose);
            goal.goal.header.frame_id="map";
            goal.start.header.frame_id="map";
            return true;
        }
        NodeStatus onResultReceived(const WrappedResult& wr) override
        {
            //RCLCPP_INFO(node_->get_logger(), "NavigateToPose Goal reached!");
                std::cout << "ComputePathToPose result " << wr.result->error_code << " " << wr.result->error_msg << "\n";
            setOutput("path", wr.result->path);
            return NodeStatus::SUCCESS;
        }
        virtual NodeStatus onFailure(ActionNodeErrorCode error) override
        {
            (void)error;
            return NodeStatus::FAILURE;
        }
        NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback)
        {
                std::cout<< "ComputePathToPose feedback Recieved:" <<  feedback <<"\n";
            return NodeStatus::RUNNING;
        }
};

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>  // Required for tf2::getYaw


// class MapFinished : public SyncActionNode
// {
//     public:
//         // If your Node has ports, you must use this constructor signature 
//         MapFinished(const std::string& name, const NodeConfig& config)
//             : SyncActionNode(name, config)
//         { }

//         // It is mandatory to define this STATIC method.
//         static PortsList providedPorts()
//         {
//             // This action has a single input port called "message"
//             return { InputPort<OccupancyGrid>("map") };
//         }

//         // Override the virtual function tick()
//         NodeStatus tick() override
//         {
//             Expected<OccupancyGrid> msg
//                 = getInput<OccupancyGrid>("map");
//             // Check if expected is valid. If not, throw its error
//             if (!msg)
//             {
//                 throw RuntimeError("missing required input [map]: ", 
//                         msg.error() );
//             }
//             OccupancyGrid map=msg.value();
//             unsigned n=map.info.width*map.info.height;
//             unsigned i;
//             unsigned cnt=0;
//             std::cout<<"Map info: "<< map.info.origin.position.x << ", " 
//                 << map.info.origin.position.x << ","<<n<<std::endl;
//             for(i=0;i<n;++i) {
//                 if(map.data[i] == -1) {
//                     cnt++;
//                 }
//             }
//             float percent_done = (float)cnt/(float)n;
//             std::cout << "Map is "<< percent_done << " known"<<std::endl;
//             if(percent_done > 0.9) {
//                 std::cout << "Map is finished!\n";
//                 return NodeStatus::SUCCESS;
//             } else {
//                 return NodeStatus::FAILURE;
//             }
//         }
// };



class FindFrontier : public SyncActionNode {
  public:
    FindFrontier(const std::string& name, const BT::NodeConfig& config)
      : BT::SyncActionNode(name, config)
    {
      RCLCPP_INFO(rclcpp::get_logger("FindFrontier"), "Constructed.");
      rcl_node_ = rclcpp::Node::make_shared("frontier_node");
      marker_pub_ = rcl_node_->create_publisher<visualization_msgs::msg::Marker>("goal_marker", 10);
      spin_thread_ = std::thread([this]() { rclcpp::spin(rcl_node_); });
    }
  
    ~FindFrontier() override {
      rcl_node_->get_node_base_interface()->get_context()->shutdown("frontier_node shutdown");
      if (spin_thread_.joinable())
        spin_thread_.join();
    }
  
    static BT::PortsList providedPorts() {
      return {
        BT::InputPort<Pose>("current_pose"),
        BT::InputPort<OccupancyGrid>("map"),
        BT::OutputPort<Pose>("goal_pose")
      };
    }
  
    NodeStatus tick() override {
      // Retrieve current pose and map from the blackboard.
      if (!getInput("current_pose", current_pose_) || !getInput("map", map_)) {
        RCLCPP_ERROR(rcl_node_->get_logger(), "Missing required input");
        return NodeStatus::FAILURE;
      }
      
      // Get the robot's current heading (yaw).
      yaw_ = tf2::getYaw(current_pose_.orientation);
  
      Pose best_pose;
      double best_score = -1.0;
      int width = map_.info.width;
      int height = map_.info.height;
      
      // Loop over the occupancy grid (skip outer border).
      for (int y = 1; y < height - 1; ++y) {
        for (int x = 1; x < width - 1; ++x) {
          int idx = y * width + x;
          // Only consider unknown cells (value == -1).
          if (map_.data[idx] != -1)
            continue;
          // Check that the candidate cell is part of a cluster.
          if (!hasUnknownGroupNearby(x, y))
            continue;
          
          // Convert grid indices to world coordinates.
          auto [wx, wy] = gridToWorld(x, y);
      
          // Check 1: Candidate is ahead (using a 35° cone).
          if (!isAhead(wx, wy))
            continue;
      
          // Construct a candidate pose.
          Pose candidate;
          candidate.position.x = wx;
          candidate.position.y = wy;
          candidate.position.z = 0.0;
          double dx = wx - current_pose_.position.x;
          double dy = wy - current_pose_.position.y;
          double target_heading = std::atan2(dy, dx);
          tf2::Quaternion q;
          q.setRPY(0, 0, target_heading);
          candidate.orientation = tf2::toMsg(q);
      
          // Check 2: 5x5 block around candidate is at least 90% known.
          if (!fiveByFiveKnown(candidate, 0.87))
            continue;
      
          // if (!farAhead(candidate, 2.0))
          //   continue;
          // Check 3: Candidate must be within 0.1 m of a wall.
          // if (!isCloseToWall(candidate, 2.0))
            // continue;
      
          // Compute Euclidean distance from current pose.
          double dist = distance(current_pose_.position.x, current_pose_.position.y, wx, wy);
          if (dist > best_score) {
            best_score = dist;
            best_pose = candidate;
          }
        }
      }
      
      if (best_score < 0) {
        RCLCPP_WARN(rcl_node_->get_logger(), "No valid frontiers found.");
        return NodeStatus::FAILURE;
      }
      
      publishMarker(best_pose);
      setOutput("goal_pose", best_pose);
      RCLCPP_INFO(rcl_node_->get_logger(), "Selected frontier: (%.2f, %.2f)", best_pose.position.x, best_pose.position.y);
      return NodeStatus::SUCCESS;
    }
  
  private:
    // Member variables.
    Pose current_pose_;
    OccupancyGrid map_;
    double yaw_;
    rclcpp::Node::SharedPtr rcl_node_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    std::thread spin_thread_;
  
    // Check if grid indices are in bounds.
    bool inBounds(int x, int y) {
      return x >= 0 && y >= 0 && x < static_cast<int>(map_.info.width) &&
             y < static_cast<int>(map_.info.height);
    }
  
    // Convert grid indices to world coordinates.
    std::pair<double, double> gridToWorld(int x, int y) {
      double wx = map_.info.origin.position.x + (x + 0.5) * map_.info.resolution;
      double wy = map_.info.origin.position.y + (y + 0.5) * map_.info.resolution;
      return {wx, wy};
    }
  
    // Convert world coordinates to grid indices.
    std::pair<int, int> worldToGrid(double wx, double wy) {
      int x = static_cast<int>((wx - map_.info.origin.position.x) / map_.info.resolution);
      int y = static_cast<int>((wy - map_.info.origin.position.y) / map_.info.resolution);
      return {x, y};
    }
  
    // Euclidean distance between two points.
    double distance(double x1, double y1, double x2, double y2) {
      return std::hypot(x2 - x1, y2 - y1);
    }
  
    // Normalized angular difference between two angles (radians).
    double angleDifference(double a, double b) {
      double diff = std::fmod(b - a + M_PI, 2 * M_PI) - M_PI;
      return diff < -M_PI ? diff + 2 * M_PI : diff;
    }
  
    // Check if there are at least 5 unknown cells in a 5x5 neighborhood.
    bool hasUnknownGroupNearby(int x, int y) {
      int count = 0;
      for (int dy = -5; dy <= 5; ++dy) {
        for (int dx = -5; dx <= 5; ++dx) {
          int nx = x + dx, ny = y + dy;
          if (!inBounds(nx, ny))
            continue;
          if (map_.data[ny * map_.info.width + nx] == -1)
            count++;
        }
      }
      return count >= 20;
    }
  
    // Check if candidate is ahead: within a 35° cone relative to the robot’s heading.
    bool isAhead(double wx, double wy) {
      double dx = wx - current_pose_.position.x;
      double dy = wy - current_pose_.position.y;
      double target_heading = std::atan2(dy, dx);
      double rel_heading = std::fabs(angleDifference(yaw_, target_heading));
      double threshold = 45.0 * M_PI / 180.0;
      return (rel_heading < threshold);
    }
  
    // Check that the 5x5 grid around the candidate is mostly known.
    // 'threshold_fraction' is the minimum fraction of cells (0 to 1) that must be known.
    bool fiveByFiveKnown(const Pose &p, double threshold_fraction) {
      auto [gx, gy] = worldToGrid(p.position.x, p.position.y);
      int known = 0, total = 0;
      // Define a 5x5 block centered on candidate (change block size if needed).
      for (int dy = -3; dy <= 3; ++dy) {
        for (int dx = -3; dx <= 3; ++dx) {
          int cx = gx + dx, cy = gy + dy;
          if (!inBounds(cx, cy))
            continue;
          total++;
          int idx = cy * map_.info.width + cx;
          if (map_.data[idx] != -1)
            known++;
        }
      }
      return (total > 0) && ((double)known / total >= threshold_fraction);
    }
  
    bool farAhead(const Pose &candidate, double min_distance) {
      double dist = distance(current_pose_.position.x,
                             current_pose_.position.y,
                             candidate.position.x,
                             candidate.position.y);
      return (dist <= min_distance);
    }
    
  
    // Publish an RViz marker (an arrow) to visualize the candidate frontier.
    void publishMarker(const Pose &pose) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = rcl_node_->now();
      marker.ns = "goal";
      marker.id = 0;
      marker.type = visualization_msgs::msg::Marker::ARROW;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose = pose;
      marker.scale.x = 0.5;
      marker.scale.y = 0.1;
      marker.scale.z = 0.1;
      marker.color.r = 0.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
      marker.color.a = 1.0f;
      marker.lifetime = rclcpp::Duration::from_seconds(0);
      marker_pub_->publish(marker);
    }
  };


  class AtHomeAction : public BT::SyncActionNode
  {
  public:
      AtHomeAction(const std::string& name, const BT::NodeConfig& config)
          : BT::SyncActionNode(name, config), has_left_home_(false)
      {}
  
      static BT::PortsList providedPorts()
      {
          return {
              BT::InputPort<Pose>("current_pose"),
              BT::InputPort<Pose>("home_pose"),
              BT::InputPort<double>("tolerance", 1.5, "How close to home (m)"),
              BT::InputPort<double>("leave_threshold", 2.0, "Min distance to mark robot as 'left home'")
          };
      }
  
      BT::NodeStatus tick() override
      {
          auto curOpt  = getInput<Pose>("current_pose");
          auto homeOpt = getInput<Pose>("home_pose");
          auto tolOpt  = getInput<double>("tolerance");
          auto leaveOpt = getInput<double>("leave_threshold");
  
          if (!curOpt || !homeOpt || !tolOpt || !leaveOpt)
          {
              throw BT::RuntimeError("AtHomeAction missing input: ",
                  (!curOpt ? curOpt.error() :
                  !homeOpt ? homeOpt.error() :
                  !tolOpt ? tolOpt.error() :
                  leaveOpt.error()));
          }
  
          const auto& cur = curOpt.value();
          const auto& home = homeOpt.value();
          double tol = tolOpt.value();
          double leave_thresh = leaveOpt.value();
  
          double dx = cur.position.x - home.position.x;
          double dy = cur.position.y - home.position.y;
          double dist = std::hypot(dx, dy);
  
          RCLCPP_INFO(rclcpp::get_logger("AtHomeAction"),
                      "AtHomeAction: Distance to home = %.2fm (Tolerance = %.2fm, LeaveThresh = %.2fm, has_left_home = %s)",
                      dist, tol, leave_thresh, has_left_home_ ? "true" : "false");
  
          if (!has_left_home_ && dist > leave_thresh)
          {
              has_left_home_ = true;
              RCLCPP_INFO(rclcpp::get_logger("AtHomeAction"),
                          "Robot has officially left home.");
          }
  
          if (has_left_home_ && dist <= tol)
          {
              RCLCPP_INFO(rclcpp::get_logger("AtHomeAction"),
                          "AtHomeAction: SUCCESS! Robot is home.");
              return BT::NodeStatus::SUCCESS;
          }
  
          return BT::NodeStatus::FAILURE;
      }
  
  private:
      bool has_left_home_;
  };
  
  
  
  class PrintPose : public BT::SyncActionNode
  {
  public:
      PrintPose(const std::string& name, const BT::NodeConfig& config)
          : BT::SyncActionNode(name, config) {}
  
      static BT::PortsList providedPorts()
      {
          return { BT::InputPort<Pose>("pose") };
      }
  
      BT::NodeStatus tick() override
      {
          auto pose_opt = getInput<Pose>("pose");
          if (!pose_opt)
          {
              throw BT::RuntimeError("PrintPose missing input: ", pose_opt.error());
          }
          const Pose& pose = pose_opt.value();
          RCLCPP_INFO(rclcpp::get_logger("PrintPose"),
                      "Home Pose: x=%.2f, y=%.2f, yaw=%.2f",
                      pose.position.x, pose.position.y, tf2::getYaw(pose.orientation));
          return BT::NodeStatus::SUCCESS;
      }
  };
  

  class SerializeMapAction : public BT::SyncActionNode
  {
  public:
      SerializeMapAction(const std::string& name, const BT::NodeConfig& config)
      : BT::SyncActionNode(name, config)
      {
          node_   = rclcpp::Node::make_shared("serialize_map_bt_client");
          client_ = node_->create_client<slam_toolbox::srv::SerializePoseGraph>(
                        "/slam_toolbox/serialize_map");
      }
  
      static BT::PortsList providedPorts()
      {
          return {
              BT::InputPort<std::string>("map_filename",
                  "full path (without extension) where the map & graph will be saved")
          };
      }
  
      BT::NodeStatus tick() override
      {
          // 1) Read filename
          auto filename_opt = getInput<std::string>("map_filename");
          if (!filename_opt)
          {
              RCLCPP_ERROR(node_->get_logger(),
                           "SerializeMapAction: missing required input [map_filename]: %s",
                           filename_opt.error().c_str());
              return BT::NodeStatus::FAILURE;
          }
          const std::string filename = filename_opt.value();
  
          // 2) Wait for service
          if (!client_->wait_for_service(std::chrono::seconds(3)))
          {
              RCLCPP_ERROR(node_->get_logger(),
                           "SerializeMapAction: /slam_toolbox/serialize_map not available");
              return BT::NodeStatus::FAILURE;
          }
  
          // 3) Build request
          auto request = std::make_shared<slam_toolbox::srv::SerializePoseGraph::Request>();
          request->filename = filename;
  
          // 4) Call service and wait up to 5s
          auto future = client_->async_send_request(request);
          if (future.wait_for(std::chrono::seconds(5)) != std::future_status::ready)
          {
              RCLCPP_ERROR(node_->get_logger(),
                           "SerializeMapAction: service call timed out");
              return BT::NodeStatus::FAILURE;
          }
  
          // 5) Check response->result (0 means success)
          auto response = future.get();
          if (response->result == 0)
          {
              RCLCPP_INFO(node_->get_logger(),
                          "SerializeMapAction: map+graph saved to '%s'", filename.c_str());
              return BT::NodeStatus::SUCCESS;
          }
          else
          {
              RCLCPP_ERROR(node_->get_logger(),
                           "SerializeMapAction: failed with result code %d", response->result);
              return BT::NodeStatus::FAILURE;
          }
      }
  
  private:
      rclcpp::Node::SharedPtr node_;
      rclcpp::Client<slam_toolbox::srv::SerializePoseGraph>::SharedPtr client_;
  };
  
static const char* xml_text = R"(
<root BTCPP_format="4">
  <BehaviorTree ID="MainTree">
    <Sequence>
      <RetryUntilSuccessful num_attempts="120">
        <PoseService name="get_pose" pose="{home_pose}" />
      </RetryUntilSuccessful>

      <PrintPose pose="{home_pose}" />

      <RetryUntilSuccessful num_attempts="120">
        <Sequence>
          <MapService map="{map}" />
          <PoseService pose="{current_pose}" />
          <FindFrontier current_pose="{current_pose}" map="{map}" goal_pose="{target_pose}" />
          <ForceSuccess>
            <NavigateToPose pose="{target_pose}" />
          </ForceSuccess>
          <Wait seconds="0.5" />
          <SaySomething message="about to check if we're home..." />
          <PoseService name="get_pose" pose="{current_pose}" />
      <AtHomeAction
        current_pose="{current_pose}"
        home_pose="{home_pose}"
        tolerance="4.0"
        leave_threshold="4.0" />

        </Sequence>
      </RetryUntilSuccessful>

      <SaySomething message="finished mapping" />
      <ForceSuccess>
        <NavigateToPose pose="{home_pose}" />
      </ForceSuccess>
      <Wait seconds="5" />
      <SaySomething message="mission completed! Saving map..." />
      <SerializeMapAction map_filename="/home/prunov/ros2_ws/src/ece3730/robot/map/map" />
        <SaySomething message="Serialization complete!" />

    </Sequence>
  </BehaviorTree>
</root>

 )";




 int main(int argc, char **argv)
 {
     rclcpp::init(argc, argv);
 
     BT::BehaviorTreeFactory factory;
 
     // Register simple condition and action nodes
     factory.registerSimpleCondition("BatteryOK", std::bind(CheckBattery));
     factory.registerNodeType<Wait>("Wait");
     factory.registerNodeType<SaySomething>("SaySomething");
     factory.registerNodeType<AtHomeAction>("AtHomeAction");
     factory.registerNodeType<PrintPose>("PrintPose");
     factory.registerNodeType<SerializeMapAction>("SerializeMapAction");

     // Register sync action node for frontier finding
     factory.registerNodeType<FindFrontier>("FindFrontier");
    //  factory.registerNodeType<PrintMappingPercentage>("PrintMappingPercentage");


 
     // NavigateToPose action
    // this is the ROS2 action client node so it needs some extra parameters
    auto nav_node = std::make_shared<rclcpp::Node>("navigate_client");
    // provide the ROS node and the name of the action service
    RosNodeParams nav_params;
    nav_params.nh = nav_node;
    nav_params.default_port_value = "navigate_to_pose";
    factory.registerNodeType<NavigateToPoseAction>("NavigateToPose", nav_params);
 
     // MapService node
     auto map_node = std::make_shared<rclcpp::Node>("map_client");
     RosNodeParams map_params;
     map_params.nh = map_node;
     map_params.default_port_value = "slam_toolbox/dynamic_map";
     factory.registerNodeType<MapService>("MapService", map_params);
 
     // PoseService node
     auto pose_node = std::make_shared<rclcpp::Node>("pose_client");
     RosNodeParams pose_params;
     pose_params.nh = pose_node;
     pose_params.default_port_value = "get_pose";
     factory.registerNodeType<PoseService>("PoseService", pose_params);
 
     // ComputePathToPose action
     auto path_node = std::make_shared<rclcpp::Node>("compute_path_client");
     RosNodeParams path_params;
     path_params.nh = path_node;
     path_params.default_port_value = "compute_path_to_pose";
     factory.registerNodeType<ComputePathToPoseAction>("ComputePathToPose", path_params);
 
     // MapFinished check node (sync action)
     factory.registerNodeType<MapFinished>("MapFinished");
 
     // Build tree from XML
     auto tree = factory.createTreeFromText(xml_text);
 
     // Tick loop
     NodeStatus status = tree.tickOnce();
 
     while (status == NodeStatus::RUNNING)
     {
         tree.sleep(std::chrono::milliseconds(100));
         status = tree.tickOnce();
     }
 
     rclcpp::shutdown();
     return 0;
 }

 

