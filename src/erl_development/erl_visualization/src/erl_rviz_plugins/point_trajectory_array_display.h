#ifndef POINT_TRAJECTORY_ARRAY_DISPLAY_H
#define POINT_TRAJECTORY_ARRAY_DISPLAY_H

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <rviz/visualization_manager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/frame_manager.h>

#include <rviz/load_resource.h>
#include <rviz/message_filter_display.h>

#include <erl_msgs/PointTrajectoryArray.h>
#include "point_trajectory_visual.h"

namespace erl_rviz_plugins
{
  class PointTrajectoryArrayDisplay : public rviz::MessageFilterDisplay<erl_msgs::PointTrajectoryArray>
  {
    Q_OBJECT
    public:
      PointTrajectoryArrayDisplay();
      ~PointTrajectoryArrayDisplay() {}

    protected:
      void onInitialize() {MFDClass::onInitialize();}
      void reset() { MFDClass::reset(); visual_ = nullptr; }

      private Q_SLOTS:
      void updateLineColorAndAlpha();
      void updateNodeColorAndAlpha();
      void updateLineScale();
      void updateNodeScale();
      void updateID();

    private:
      void processMessage(const erl_msgs::PointTrajectoryArray::ConstPtr &msg);
      void visualizeMessage(int state);

      std::shared_ptr<PointTrajectoryVisual> visual_;

      rviz::ColorProperty *line_color_property_;
      rviz::ColorProperty *node_color_property_;
      rviz::FloatProperty *line_scale_property_;
      rviz::FloatProperty *node_scale_property_;
      rviz::EnumProperty *id_property_;

      Ogre::Vector3 position_;
      Ogre::Quaternion orientation_;

      erl_msgs::PointTrajectoryArray point_trajectories_;
  };
}
#endif
