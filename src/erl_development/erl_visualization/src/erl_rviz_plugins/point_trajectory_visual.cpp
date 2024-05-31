#include "point_trajectory_visual.h"

namespace erl_rviz_plugins
{
  void PointTrajectoryVisual::setMessage(const erl::aligned_vector<Eigen::Vector3f> &point_trajectory)
  {
    nodes_.clear();
    lines_.clear();
    //line_.reset(new rviz::BillboardLine(scene_manager_, frame_node_));

    if (point_trajectory.empty())
      return;

    nodes_.resize(point_trajectory.size());
    lines_.resize(point_trajectory.size() - 1);

    for (auto &it : nodes_)
      it.reset(new rviz::Shape(rviz::Shape::Type::Sphere, scene_manager_,
                               frame_node_));
    for (auto &it : lines_)
      it.reset(new rviz::BillboardLine(scene_manager_, frame_node_));

    for (int i = 0; i < (int) point_trajectory.size(); i++) {
      Ogre::Vector3 pos(point_trajectory[i](0), point_trajectory[i](1), point_trajectory[i](2));
      if(i < (int) point_trajectory.size() - 1) {
        Ogre::Vector3 pos2(point_trajectory[i+1](0), point_trajectory[i+1](1), point_trajectory[i+1](2));
        lines_[i]->addPoint(pos);
        lines_[i]->addPoint(pos2);
      }
      nodes_[i]->setPosition(pos);
    }
  }

  void PointTrajectoryVisual::addMessage(const erl::aligned_vector<Eigen::Vector3f> &point_trajectory)
  {
    if (point_trajectory.empty())
      return;

    unsigned int nodes_prev_size = nodes_.size();
    nodes_.resize(nodes_prev_size + point_trajectory.size());

    for (unsigned int i = nodes_prev_size; i < nodes_.size(); i++)
      nodes_[i].reset(new rviz::Shape(rviz::Shape::Type::Sphere, scene_manager_,
                               frame_node_));

    unsigned int lines_prev_size = lines_.size();
    lines_.resize(lines_prev_size + point_trajectory.size() - 1);

    for (unsigned int i = lines_prev_size; i < lines_prev_size + point_trajectory.size() - 1; i++) {
      lines_[i].reset(new rviz::BillboardLine(scene_manager_, frame_node_));
    }

    for (int i = 0; i < (int) point_trajectory.size(); i++) {
      Ogre::Vector3 pos(point_trajectory[i](0), point_trajectory[i](1), point_trajectory[i](2));
      if(i < (int) point_trajectory.size() - 1) {
        Ogre::Vector3 pos2(point_trajectory[i+1](0), point_trajectory[i+1](1), point_trajectory[i+1](2));
        lines_[i + lines_prev_size]->addPoint(pos);
        lines_[i + lines_prev_size]->addPoint(pos2);
      }
      nodes_[i + nodes_prev_size]->setPosition(pos);
    }
  }
}

