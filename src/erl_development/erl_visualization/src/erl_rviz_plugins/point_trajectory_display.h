#ifndef POINT_TRAJECTORY_DISPLAY_H
#define POINT_TRAJECTORY_DISPLAY_H

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <rviz/visualization_manager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/frame_manager.h>

#include <rviz/load_resource.h>
#include <rviz/message_filter_display.h>

#include <erl_utilities/erl_types.h>
#include <erl_msgs/PointTrajectory.h>

#include "point_trajectory_visual.h"

namespace erl_rviz_plugins
{
  class PointTrajectoryDisplay : public rviz::MessageFilterDisplay<erl_msgs::PointTrajectory>
  {
    Q_OBJECT
    public:
      PointTrajectoryDisplay();
      ~PointTrajectoryDisplay() {}

    protected:
      void onInitialize() {MFDClass::onInitialize();}
      void reset() { MFDClass::reset(); visual_ = nullptr; }

      private Q_SLOTS:
      void updateLineColorAndAlpha();
      void updateNodeColorAndAlpha();
      void updateLineScale();
      void updateNodeScale();

    private:
      void processMessage(const erl_msgs::PointTrajectory::ConstPtr &msg);
      void visualizeMessage();

      std::shared_ptr<PointTrajectoryVisual> visual_;

      rviz::ColorProperty *line_color_property_;
      rviz::ColorProperty *node_color_property_;
      rviz::FloatProperty *line_scale_property_;
      rviz::FloatProperty *node_scale_property_;

      Ogre::Vector3 position_;
      Ogre::Quaternion orientation_;

      erl::aligned_vector<Eigen::Vector3f> point_trajectory_;
  };
 
}
#endif
