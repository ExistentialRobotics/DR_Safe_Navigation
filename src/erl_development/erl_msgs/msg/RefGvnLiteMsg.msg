# This represents a simplified version of reference governor state information
# rbt_pose2D: Robot position in 2D
# gvn_pose2D: Reference governor position in 2D
# gvn_status: Governor running status

Header header
geometry_msgs/Pose2D rbt_pose2D
geometry_msgs/Pose2D gvn_pose2D
float32 search_radius
int32 gvn_status
