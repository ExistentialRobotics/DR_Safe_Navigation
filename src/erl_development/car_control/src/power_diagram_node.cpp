#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <Eigen/Geometry>
#include <dd_control/DDControl.h>
#include <tf/transform_datatypes.h>

class PowerDiagramControlNode
{
 public:
    PowerDiagramControlNode() :
      position_cmd_updated_(false),
      position_cmd_active_(false),
      des_yaw_(0),
      des_yaw_dot_(0),
      have_odom_(false)
  {
    // Initialize the Node on Construction.
    onInit();
  }

  void onInit(void);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW; // Need this since we have SO3Control which needs aligned pointer

 private:
  void publishDDCommand(void);
  void position_cmd_callback(const geometry_msgs::PoseStamped::ConstPtr &cmd);
  void pose_callback(const geometry_msgs::PoseStamped::ConstPtr &pose);
  void odom_callback(const nav_msgs::Odometry::ConstPtr &odom);

  PowerDiagramControl controller_;
  ros::Publisher dd_command_pub;
  ros::Subscriber odom_sub_, pose_sub_, position_cmd_sub_;

  bool position_cmd_updated_ {false}, position_cmd_active_ {false};
  std::string frame_id_;

  Eigen::Vector3d des_pos_, des_vel_, des_acc_, des_jrk_;
  double des_yaw_, des_yaw_dot_;
  // Gains
  double ka, kl;
  // Max Velocities
  double vmax, wmax;
  bool have_odom_;
};


void PowerDiagramControlNode::publishDDCommand(void)
{
  if (!have_odom_)
  {
    ROS_WARN("No odometry! Not publishing DDCommand.");
    return;
  }


  // Eigen::Vector3f ki = Eigen::Vector3f::Zero();

  controller_.calculateControl(des_pos_[0], des_pos_[1], ka, kl, vmax, wmax);
  const double v = controller_.getLinearVelocity();
  const double w = controller_.getAngularVelocity();
  // Setup Twist Message here.
  geometry_msgs::Twist::Ptr dd_command(new geometry_msgs::Twist);
  dd_command->linear.x = v;
  dd_command->angular.z = w;
  ROS_INFO_THROTTLE(2.0, "Publishing DD Command %f %f\n", v, w);

  // Publish Twist Message.
  dd_command_pub.publish(dd_command);
}

void PowerDiagramControlNode::position_cmd_callback(const geometry_msgs::PoseStamped::ConstPtr &cmd)
{
  des_pos_ = Eigen::Vector3d(cmd->pose.position.x, cmd->pose.position.y, cmd->pose.position.z);
//  des_vel_ = Eigen::Vector3f(cmd->velocity.x, cmd->velocity.y, cmd->velocity.z);

  des_yaw_ = tf::getYaw(cmd->pose.orientation);
//  des_yaw_dot_ = cmd->yaw_dot;
  position_cmd_updated_ = true;
  position_cmd_active_ = true;
  ROS_INFO("Received position command %f %f %f\n", des_pos_[0], des_pos_[1], des_yaw_);
  publishDDCommand();
}

void PowerDiagramControlNode::odom_callback(const nav_msgs::Odometry::ConstPtr &odom)
{
  have_odom_ = true;

  const Eigen::Vector3d position(odom->pose.pose.position.x,
                                 odom->pose.pose.position.y,
                                 odom->pose.pose.position.z);
  const Eigen::Vector3d velocity(odom->twist.twist.linear.x,
                                 odom->twist.twist.linear.y,
                                 odom->twist.twist.linear.z);

  double current_yaw_ = tf::getYaw(odom->pose.pose.orientation);

  controller_.setPosition(position);
  controller_.setHeading(current_yaw_);

  if(position_cmd_active_)
  {
    // We set position_cmd_updated_ = false and expect that the
    // position_cmd_callback would set it to true since typically a position_cmd
    // message would follow an odom message. If not, the position_cmd_callback
    // hasn't been called and we publish the DD command ourselves
    if(!position_cmd_updated_)
      publishDDCommand(); 
    position_cmd_updated_ = false;

    position_cmd_active_ = !controller_.checkReachedGoal(des_pos_, des_yaw_);
  }
}

void PowerDiagramControlNode::pose_callback(const geometry_msgs::PoseStamped::ConstPtr &pose)
{
  have_odom_ = true;

  const Eigen::Vector3d position(pose->pose.position.x,
                                 pose->pose.position.y,
                                 pose->pose.position.z);
  double current_yaw_ = tf::getYaw(pose->pose.orientation);

  controller_.setPosition(position);
  controller_.setHeading(current_yaw_);

  if(position_cmd_active_)
  {
    // We set position_cmd_updated_ = false and expect that the
    // position_cmd_callback would set it to true since typically a position_cmd
    // message would follow an odom message. If not, the position_cmd_callback
    // hasn't been called and we publish the DD command ourselves
    if(!position_cmd_updated_)
      publishDDCommand(); 
    position_cmd_updated_ = false;

    position_cmd_active_ = !controller_.checkReachedGoal(des_pos_, des_yaw_);
  }
}

void PowerDiagramControlNode::onInit(void) {
  ros::NodeHandle n("~");
  std::string robot_name;
  // Setup Subscribers
  odom_sub_ = n.subscribe("odom", 10, &PowerDiagramControlNode::odom_callback, this, ros::TransportHints().tcpNoDelay());
  pose_sub_ = n.subscribe("pose", 10, &PowerDiagramControlNode::pose_callback, this, ros::TransportHints().tcpNoDelay());
  position_cmd_sub_ = n.subscribe("position_cmd", 10, &PowerDiagramControlNode::position_cmd_callback, this,
                                  ros::TransportHints().tcpNoDelay());
  // Setup Publisher
  dd_command_pub = n.advertise<geometry_msgs::Twist>("commands", 10);

  // Read in Gains Config.
  n.param("gains/ka", ka, 0.5);
  n.param("gains/kl", kl, 0.5);

  n.param("vmax", vmax, 1.0);
  n.param("wmax", wmax, 2.0);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "power_diagram_node");
  PowerDiagramControlNode node;

  ros::spin();
}
