
#include <erl_astar/astar_nx.h>
#include <erl_conversions/erl_msg_utils.h>
#include <erl_msgs/GridMap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/duration.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <erl_astar/environments/planning_2d.h>

// # TODOABG update tf to tf2 package, why odom and map frame only differ in x, y ?
// # TODOABG publish path using Path2D.msg

/**
 * @brief ErlAstar2dOnline The ErlAstar2dOnline class is responsible for reading in all configuration
 * parameters for 2d of A* planner that can be called, as well as managing necessary
 * subscribers and publishers to generate a 2d path with the ERL Astar package.
 */

class ErlAstar2dOnline {
 public:
  /**
   * @brief Constructs the ErlAstar2dOnline.
   * @param nh The ROS nodehandle.
   */
  void init() {
    // Load parameters
    if (!readParameters()) {
      ROS_ERROR("ErlAstar2dOnline parameter loading failed!");
      ros::requestShutdown();
    } else {
      ROS_INFO("ErlAstar2dOnline parameter loading successfully");
    }

    // A* planner attach
    erl::ARAStar<std::vector<int>> AA;
    this->AA = std::make_shared<erl::ARAStar<std::vector<int>>>(AA);
  }

  explicit ErlAstar2dOnline(ros::NodeHandle &nh) : nh(nh) {
    init();
    // Setup Publishers and then Subscribers
    path_pub = nh.advertise<nav_msgs::Path>(path_topic, 1, 1);
    rviz_goal_sub = nh.subscribe<geometry_msgs::PoseStamped>(goal_topic, 1, &ErlAstar2dOnline::goalCallback, this);
    map_sub = nh.subscribe<nav_msgs::OccupancyGrid>(map_topic, 1, &ErlAstar2dOnline::gridMapCallback, this);
    odom_sub = nh.subscribe<nav_msgs::Odometry>(odom_topic, 1, &ErlAstar2dOnline::odomCallback, this);

  }  // end of constructor

 private:
  using dim = uint8_t;
  // ----------------------- ROS -------------------------------
  ros::NodeHandle nh;
  // subscribers
  ros::Subscriber odom_sub;       // Odometry Subscriber.
  ros::Subscriber map_sub;        // GridMap Subscriber from a mapper.
  ros::Subscriber rviz_goal_sub;  // Goal Subscriber

  std::string odom_topic = "/odom";
  std::string map_topic = "/map";
  std::string goal_topic = "/move_base_simple/goal";

  // publishers
  ros::Publisher path_pub;  // Path Publisher
  std::string planning_frame = "map"; // planning frame computationally grid_msg frame is the best choice
  std::string path_topic = "/path";

  // status flags
  bool goal_received{false};
  bool map_received{false};

  // transform for planning in map frame, robot state from odometry in odom frame
  tf::TransformListener start_listener;
  tf::StampedTransform start_transform;

  tf::TransformListener goal_listener;
  tf::StampedTransform goal_transform;

  // ----------------------- A* planning --------------------------
  // grid map
  std::shared_ptr<erl::GridMap<dim>> grid_map;

  // A* core
  std::shared_ptr<erl::Planning2D<dim>> planning_env;
  std::shared_ptr<erl::ARAStar<std::vector<int>>> AA;

  // robot circumscribed radius http://wiki.ros.org/costmap_2d
  double inflation_value{0.0};

  // Astar Goal
  double goal_x{0.0};
  double goal_y{0.0};

  // Astar start
  double start_x{0.0};
  double start_y{0.0};

  // ----------------------- FUNCTIONS --------------------------

  /**
   * Update goal from rviz
   */
  void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    goal_received = true;
    goal_x = msg->pose.position.x;
    goal_y = msg->pose.position.y;

    // transform goal in planning frame, assume 2d, frame orientation are the same
    if (msg->header.frame_id != planning_frame){
      ROS_WARN("planning_frame [%s] is different from occupancy msg frame_id [%s]", planning_frame.c_str(), msg->header.frame_id.c_str());
      try {
        goal_listener.waitForTransform(planning_frame, msg->header.frame_id, ros::Time(0), ros::Duration(1.0));
        goal_listener.lookupTransform(planning_frame, msg->header.frame_id, ros::Time(0), goal_transform);
      } catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
      }
      goal_x += goal_transform.getOrigin().x();
      goal_y += goal_transform.getOrigin().y();
    }
    callAStar(start_x, start_y);
  }

  /**
   * Call A* after the robot pose is updated.
   */
  void callAStar(double start_x, double start_y) {
    if (!goal_received) return;
    // Update the goal cell as it might have been changed due to the expanding map
    this->planning_env->goal_coord = this->planning_env->map->meters2cells({goal_x, goal_y});
    // Call A* to update path
    auto start_cell = planning_env->toState({start_x, start_y});  // Convert metric to cell state.

    if (planning_frame != "map"){
      ROS_INFO("planning in [%s] frame", planning_frame.c_str());  
    }

    auto goal_cell = this->planning_env->toState({goal_x, goal_y});
    ROS_INFO("Begin Astar\nStart 2D: %6.2f %6.2f\nGoal  2D: %6.2f %6.2f", start_x, start_y, goal_x, goal_y);
    ROS_INFO("\nStart 2D cell: %5d %5d\nGoal  2D cell: %5d %5d", start_cell[0], start_cell[1], goal_cell[0], goal_cell[1]);

    auto output = AA->Astar(start_cell, *planning_env, 1.0);
    ROS_INFO("Finished Astar: Path 2D Length (in cell): %d\t\t Cost: %.2f\n", output.action_idx.size(), output.pcost);

    std::vector<std::vector<double>> metric_path;
    for (const auto &state : output.path) {
      // metric_path contains 2d location in planning frame (map frame)
      metric_path.push_back(planning_env->toMetric(state));
    }
    // Publish Path Message
    publishPathMessage(metric_path);
  }

  /**
   * Callback for Odometry messages. Writes the robot current state to the class members.
   * @param msg The incoming Odometry message.
   */
  void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    // Pose below is in Odom frame
    auto pose_odom = msg->pose.pose;

    if (!map_received || !goal_received) {
      return;
    }

    // start_x, start_y in msg transform it to planning_frame
    start_x = pose_odom.position.x; 
    start_y = pose_odom.position.y;

    // transform goal in planning frame, assume 2d, frame orientation are the same
    if (msg->header.frame_id != planning_frame){
      try {
        start_listener.waitForTransform(planning_frame, msg->header.frame_id, ros::Time(0), ros::Duration(1.0));
        start_listener.lookupTransform(planning_frame, msg->header.frame_id, ros::Time(0), start_transform);
      } catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
      }
      start_x += start_transform.getOrigin().x();
      start_y += start_transform.getOrigin().y();
    }
    callAStar(start_x, start_y);
  }

  /**
   * Callback for Odometry messages. Writes the robot current state to the class members.
   * @param msg The incoming Odometry message.
   */
  void gridMapCallback(const nav_msgs::OccupancyGrid::ConstPtr &gridmap_msg) {
    // Convert ROS Occupancy Grid Map to erl::GridMap for planning
    // fromROS prototype used from #include <erl_conversions/erl_msg_utils.h>:
    // inline erl::GridMap<uint8_t> fromROS(const nav_msgs::OccupancyGrid &gridmap_msg, bool copy_data = true, //bool binary_map = false, int threshold = 50) 
    auto gm = erl::fromROS(*gridmap_msg, true, true);
    bool result = gm.setMap(erl::inflateMap2D(gm, inflation_value));  // Inflate Map
    ROS_DEBUG("Inflating Map by %f meters. Success = %d", inflation_value, result);
    grid_map = std::make_shared<erl::GridMap<dim>>(gm);

    // check on planning_frame consistency, 
    auto msg_frame_id = gridmap_msg->header.frame_id;
    if (planning_frame != msg_frame_id){
      ROS_ERROR("planning_frame [%s] is different from occupancy msg frame_id [%s]", planning_frame.c_str(), msg_frame_id.c_str());
    }

    // Generate new map parameter the first time, only update map contents next update.
    if (!map_received) {
      erl::Environment2D<dim> curr_env(*grid_map);
      erl::Planning2D<dim> planning_env(curr_env, {goal_x, goal_y});
      this->planning_env = std::make_shared<erl::Planning2D<dim>>(planning_env);
      map_received = true;
    }
    // Update map
    this->planning_env->map = grid_map;
  }

  /**
   * Utils function. transform 2d location to 3d pose ros msg
   * @param x robot location coord x in map frame
   * @param y robot location coord y in map frame
   */
  geometry_msgs::PoseStamped getRos3DPoseFromAstar2DLocation(double x, double y) {
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.w = 1.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    return pose;
  }

  /**
   * Publish output of A as ros path msg.
   * @param metric_path A* output, path 2d
   */
  void publishPathMessage(const std::vector<std::vector<double>> &metric_path) {
    nav_msgs::Path path_msg;
    path_msg.header.frame_id = planning_frame;
    for (const auto &loc2d : metric_path) {
      path_msg.poses.push_back(getRos3DPoseFromAstar2DLocation(loc2d[0], loc2d[1]));
    }
    path_pub.publish(path_msg);
  }

  /**
   * @brief Read parameters from ROS parameter server
   */

  bool readParameters() {
    // read parameters for scan subscriber
    // for odom topic
    ROS_INFO("ErlAstar2d Reading Parameter...");
    if (!nh.getParam("odom_topic", odom_topic)) {
      ROS_ERROR("Could not find [odom_topic] parameter");
      return false;
    } else {
      ROS_INFO_STREAM("Read odom_topic:" << odom_topic);
    }
    // for map topic
    if (!nh.getParam("map_topic", map_topic)) {
      ROS_ERROR("Could not find [map_topic] parameter");
      return false;
    } else {
      ROS_INFO_STREAM("Read map_topic:" << map_topic);
    }

    if (!nh.getParam("planning_frame", planning_frame)) {
      ROS_ERROR("Planning_frame is not specified ! %s", planning_frame.c_str());
      return false;
    } else {
      ROS_INFO_STREAM("Read planning_frame:" << planning_frame);
    }

    // for goal topic
    if (!nh.getParam("goal_topic", goal_topic)) {
      ROS_ERROR("Could not find [goal_topic] parameter");
      return false;
    } else {
      ROS_INFO_STREAM("Read goal_topic:" << goal_topic);
    }

    // for A* path2d publish
    if (!nh.getParam("path_topic", path_topic)) {
      ROS_ERROR("Could not find [path_topic] parameter");
      return false;
    } else {
      ROS_INFO_STREAM("Read path_topic:" << path_topic);
    }

    // for A* map inflation
    if (!nh.getParam("inflation_value", inflation_value)) {
      ROS_ERROR("Could not find [inflation_value] parameter");
      return false;
    } else {
      ROS_INFO_STREAM("Read inflation_value:" << inflation_value);
    }

    return true;
  }

  /**
   * @brief Init all parameters. 1) from parameter server 2) attach A* planner
   */
};
