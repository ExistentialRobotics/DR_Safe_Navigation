#include "point_trajectory_array_display.h"
#include <erl_conversions/erl_msg_utils.h>

namespace erl_rviz_plugins
{

  PointTrajectoryArrayDisplay::PointTrajectoryArrayDisplay()
  {
    line_color_property_ =
      new rviz::ColorProperty("LineColor", QColor(204, 51, 204), "Color to draw the line.",
                              this, SLOT(updateLineColorAndAlpha()));
    node_color_property_ =
      new rviz::ColorProperty("NodeColor", QColor(85, 85, 255), "Color to draw the node.",
                              this, SLOT(updateNodeColorAndAlpha()));
    line_scale_property_ =
      new rviz::FloatProperty("LineScale", 0.1, "0.1 is the default value.",
                              this, SLOT(updateLineScale()));
    node_scale_property_ =
      new rviz::FloatProperty("NodeScale", 0.15, "0.15 is the default value.",
                              this, SLOT(updateNodeScale()));
    id_property_ =
      new rviz::EnumProperty("ID", "All", "Visualize individual path using its id, All is the default",
                             this, SLOT(updateID()));
  }

  void PointTrajectoryArrayDisplay::updateLineColorAndAlpha() {
    Ogre::ColourValue color = line_color_property_->getOgreColor();
    if(visual_)
      visual_->setLineColor(color.r, color.g, color.b, 1);
  }

  void PointTrajectoryArrayDisplay::updateNodeColorAndAlpha() {
    Ogre::ColourValue color = node_color_property_->getOgreColor();
    if(visual_)
      visual_->setNodeColor(color.r, color.g, color.b, 1);
  }

  void PointTrajectoryArrayDisplay::updateLineScale() {
    float s = line_scale_property_->getFloat();
    if(visual_)
      visual_->setLineScale(s);
  }

  void PointTrajectoryArrayDisplay::updateNodeScale() {
    float s = node_scale_property_->getFloat();
    if(visual_)
      visual_->setNodeScale(s);
  }

  void PointTrajectoryArrayDisplay::processMessage(const erl_msgs::PointTrajectoryArray::ConstPtr &msg)
  {
    if (!context_->getFrameManager()->getTransform(msg->header.frame_id, msg->header.stamp, position_, orientation_)) {
      ROS_DEBUG("Error transforming from frame '%s' to frame '%s'",
                msg->header.frame_id.c_str(), qPrintable(fixed_frame_));
      return;
    }

    point_trajectories_ = *msg;

    id_property_->clearOptions();
    id_property_->addOption("All", -1);
    for (int i = 0; i < (int) point_trajectories_.trajectories.size(); i++) {
      if(point_trajectories_.trajectories[i].name.empty())
        id_property_->addOption(QString::number(i), i);
      else
        id_property_->addOption(QString(point_trajectories_.trajectories[i].name.c_str()), i);
    }

    int id = id_property_->getOptionInt();
    visualizeMessage(id);
  }


  void PointTrajectoryArrayDisplay::visualizeMessage(int id)
  {
    if (id >= (int) point_trajectories_.trajectories.size())
      return;

    std::shared_ptr<PointTrajectoryVisual> visual;
    visual.reset(new PointTrajectoryVisual(context_->getSceneManager(), scene_node_));

    bool set = false;
    if(id >= 0) {
      set = true;
      visual->setMessage(erl::fromROS(point_trajectories_.trajectories[id]));
      visual->setFramePosition(position_);
      visual->setFrameOrientation(orientation_);
    }
    else {
      for (int i = 0; i < (int)point_trajectories_.trajectories.size(); i++)
      {
        if(i == 0) {
          set = true;
          visual->setMessage(erl::fromROS(point_trajectories_.trajectories[i]));
          visual->setFramePosition(position_);
          visual->setFrameOrientation(orientation_);
          continue;
        }
        visual->addMessage(erl::fromROS(point_trajectories_.trajectories[i]));
      }
    }

    if(set){
      float line_scale = line_scale_property_->getFloat();
      visual->setLineScale(line_scale);
      float node_scale = node_scale_property_->getFloat();
      visual->setNodeScale(node_scale);

      Ogre::ColourValue line_color = line_color_property_->getOgreColor();
      visual->setLineColor(line_color.r, line_color.g, line_color.b, 1);
      Ogre::ColourValue node_color = node_color_property_->getOgreColor();
      visual->setNodeColor(node_color.r, node_color.g, node_color.b, 1);
    }
    visual_ = visual;
  }

  void PointTrajectoryArrayDisplay::updateID() {
    int id = id_property_->getOptionInt();
    visualizeMessage(id);
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(erl_rviz_plugins::PointTrajectoryArrayDisplay, rviz::Display)

