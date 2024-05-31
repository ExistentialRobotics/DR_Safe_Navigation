#ifndef POINT_TRAJECTORY_VISUAL_H
#define POINT_TRAJECTORY_VISUAL_H

#include <erl_utilities/erl_types.h>
#include <erl_msgs/PointTrajectory.h>

#include <OGRE/OgreVector3.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <rviz/ogre_helpers/billboard_line.h>
#include <rviz/ogre_helpers/shape.h>

namespace erl_rviz_plugins
{
  class PointTrajectoryVisual
  {
    public:
      PointTrajectoryVisual(Ogre::SceneManager *scene_manager, Ogre::SceneNode *parent_node)
      {
        scene_manager_ = scene_manager;
        frame_node_ = parent_node->createChildSceneNode();
      }
      ~PointTrajectoryVisual(){ scene_manager_->destroySceneNode(frame_node_); }

      void setMessage(const erl::aligned_vector<Eigen::Vector3f> &point_trajectory);
      void addMessage(const erl::aligned_vector<Eigen::Vector3f> &point_trajectory);

      void setFramePosition(const Ogre::Vector3 &position) { frame_node_->setPosition(position); }
      void setFrameOrientation(const Ogre::Quaternion &orientation) { frame_node_->setOrientation(orientation); }

      void setLineColor(float r, float g, float b, float a) { for(auto& it: lines_) it->setColor(r, g, b, a); }
      void setNodeColor(float r, float g, float b, float a) { for (auto &it : nodes_) it->setColor(r, g, b, a); }
      void setLineScale(float s) { for (auto &it : lines_) it->setLineWidth(s); }
      void setNodeScale(float s) { for (auto &it : nodes_) it->setScale(Ogre::Vector3(s, s, s)); }
  
    private:
      std::vector<std::unique_ptr<rviz::BillboardLine>> lines_;
      std::vector<std::unique_ptr<rviz::Shape>> nodes_;

      Ogre::SceneNode *frame_node_;
      Ogre::SceneManager *scene_manager_;
  };
}

#endif
