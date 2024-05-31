#ifndef __ERL_TRIANGLE_MSG_GENERATOR_
#define __ERL_TRIANGLE_MSG_GENERATOR_

#include <visualization_msgs/Marker.h>

namespace erl {
class TriangleMsgGenerator {
  visualization_msgs::Marker::Ptr msg;
 public:

  TriangleMsgGenerator() {};

  TriangleMsgGenerator(std::string frame_id)
      : msg(new visualization_msgs::Marker) {
    
    msg->header.frame_id = frame_id;
    msg->ns = "triangles_visualization";
    msg->id = 0;
    msg->type = visualization_msgs::Marker::TRIANGLE_LIST;
    msg->action = visualization_msgs::Marker::ADD;
    msg->pose.position.x = 0.0;
    msg->pose.position.y = 0.0;
    msg->pose.position.z = 0.0;
    msg->pose.orientation.x = 0.0;
    msg->pose.orientation.y = 0.0;
    msg->pose.orientation.z = 0.0;
    msg->pose.orientation.w = 1.0;
    msg->scale.x = 1.0;
    msg->scale.y = 1.0;
    msg->scale.z = 1.0;
    msg->color.r = 0.0;
    msg->color.g = 1.0;
    msg->color.b = 1.0;
    msg->color.a = 1.0;
  }

  void insert_triangle(float p1x, float p1y, float p1z,
                       float p2x, float p2y, float p2z,
                       float p3x, float p3y, float p3z) {
    geometry_msgs::Point p1;
    p1.x = p1x;
    p1.y = p1y;
    p1.z = p1z;

    geometry_msgs::Point p2;
    p2.x = p2x;
    p2.y = p2y;
    p2.z = p2z;

    geometry_msgs::Point p3;
    p3.x = p3x;
    p3.y = p3y;
    p3.z = p3z;

    msg->points.push_back(p1);
    msg->points.push_back(p2);
    msg->points.push_back(p3);
  }

  void insert_color_triangle(float p1x, float p1y, float p1z,
                             float p2x, float p2y, float p2z,
                             float p3x, float p3y, float p3z,
                             float c1r, float c1g, float c1b,
                             float c2r, float c2g, float c2b,
                             float c3r, float c3g, float c3b) {
    insert_triangle(p1x, p1y, p1z, p2x, p2y, p2z, p3x, p3y, p3z);

    std_msgs::ColorRGBA c1;
    c1.r = c1r;
    c1.g = c1g;
    c1.b = c1b;

    std_msgs::ColorRGBA c2;
    c2.r = c2r;
    c2.g = c2g;
    c2.b = c2b;

    std_msgs::ColorRGBA c3;
    c3.r = c3r;
    c3.g = c3g;
    c3.b = c3b;

    msg->colors.push_back(c1);
    msg->colors.push_back(c2);
    msg->colors.push_back(c3);
  }

  void clear() {
    msg->points.clear();
    msg->colors.clear();
  }

  visualization_msgs::Marker::Ptr get_msg() const {
    msg->header.stamp = ros::Time::now();
    return msg;
  }
}; // End Class
} // End namespace

#endif
