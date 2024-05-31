#ifndef PRIMITIVE_TRAJECTORY_DISPLAY_H
#define PRIMITIVE_TRAJECTORY_DISPLAY_H

#include <boost/circular_buffer.hpp>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <rviz/visualization_manager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/bool_property.h>
#include <rviz/frame_manager.h>

#include <rviz/load_resource.h>

#include <erl_msgs/PrimitiveTrajectory.h>
#include <rviz/message_filter_display.h>

#include "primitive_trajectory_visual.h"

namespace erl_rviz_plugins {
  class PrimitiveTrajectoryDisplay
    : public rviz::MessageFilterDisplay<erl_msgs::PrimitiveTrajectory> {
      Q_OBJECT
      public:
        PrimitiveTrajectoryDisplay();
        virtual ~PrimitiveTrajectoryDisplay();

      protected:
        virtual void onInitialize();

        virtual void reset();

        private Q_SLOTS:
        void updatePosColorAndAlpha();
        void updateVelColorAndAlpha();
        void updateAccColorAndAlpha();
        void updateJrkColorAndAlpha();
        void updatePosScale();
        void updateVelScale();
        void updateAccScale();
        void updateJrkScale();
        void updateVelVis();
        void updateAccVis();
        void updateJrkVis();
        void updateHistoryLength();
        void updateNum();

      private:
        void processMessage(const erl_msgs::PrimitiveTrajectory::ConstPtr &msg);
        void visualizeMessage();

        boost::circular_buffer<boost::shared_ptr<PrimitiveTrajectoryVisual>> visuals_;

        rviz::ColorProperty *pos_color_property_;
        rviz::ColorProperty *vel_color_property_;
        rviz::ColorProperty *acc_color_property_;
        rviz::ColorProperty *jrk_color_property_;
        rviz::FloatProperty *pos_scale_property_;
        rviz::FloatProperty *vel_scale_property_;
        rviz::FloatProperty *acc_scale_property_;
        rviz::FloatProperty *jrk_scale_property_;
        rviz::BoolProperty *vel_vis_property_;
        rviz::BoolProperty *acc_vis_property_;
        rviz::BoolProperty *jrk_vis_property_;
        rviz::IntProperty *history_length_property_;
        rviz::IntProperty *num_property_;

        Ogre::Vector3 position_;
        Ogre::Quaternion orientation_;

        erl_msgs::PrimitiveTrajectory primitive_trajectory_;
    };

}
#endif
