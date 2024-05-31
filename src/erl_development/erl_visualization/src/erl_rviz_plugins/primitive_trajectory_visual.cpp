#include "primitive_trajectory_visual.h"

namespace erl_rviz_plugins {

PrimitiveTrajectoryVisual::PrimitiveTrajectoryVisual(Ogre::SceneManager *scene_manager,
                       Ogre::SceneNode *parent_node) {
  scene_manager_ = scene_manager;
  frame_node_ = parent_node->createChildSceneNode();
}

PrimitiveTrajectoryVisual::~PrimitiveTrajectoryVisual() { scene_manager_->destroySceneNode(frame_node_); }

template <typename T>
std::vector<T> linspace(T a, T b, size_t N) {
  T h = (b - a) / static_cast<T>(N-1);
  std::vector<T> xs(N);
  typename std::vector<T>::iterator x;
  T val;
  for (x = xs.begin(), val = a; x != xs.end(); ++x, val += h)
    *x = val;
  return xs;
}

void PrimitiveTrajectoryVisual::setMessage(const erl_msgs::PrimitiveTrajectory &msg) {
  poss_.clear();
  vels_.clear();
  accs_.clear();
  jrks_.clear();

  if (msg.primitives.empty() || num_ == 0)
    return;

  poss_.resize(num_);
  if(vel_vis_)
    vels_.resize(num_);
  if(acc_vis_)
    accs_.resize(num_);
  if(jrk_vis_)
    jrks_.resize(num_);

  for (int i = 0; i < num_; i++) {
    poss_[i].reset(new rviz::BillboardLine(scene_manager_, frame_node_));
    if(vel_vis_)
      vels_[i].reset(new rviz::BillboardLine(scene_manager_, frame_node_));
    if(acc_vis_)
      accs_[i].reset(new rviz::BillboardLine(scene_manager_, frame_node_));
    if(jrk_vis_)
      jrks_[i].reset(new rviz::BillboardLine(scene_manager_, frame_node_));
  }


  trajectories::PiecewisePolynomial<double> trajectory = erl::fromROS(msg);
  auto velocity_traj = trajectory.derivative();
  auto accel_traj = velocity_traj.derivative();
  auto jrk_traj = accel_traj.derivative();

  double theta = M_PI / 2;
  Eigen::Matrix3d R;
  R << cos(theta), -sin(theta), 0,
      sin(theta), cos(theta), 0,
      0, 0, 1;

  auto tspan = linspace(trajectory.start_time(), trajectory.end_time(), num_);
  for (int i = 1; i < tspan.size(); i++) {
    Eigen::Vector3d p1 = trajectory.value(tspan[i-1]);
    Eigen::Vector3d p2 = trajectory.value(tspan[i]);

    Ogre::Vector3 pos1(p1(0, 0), p1(1, 0), p1(2, 0));
    Ogre::Vector3 pos2(p2(0, 0), p2(1, 0), p2(2, 0));
    poss_[i - 1]->addPoint(pos1);
    poss_[i - 1]->addPoint(pos2);

    if(vel_vis_){
      Eigen::Vector3d vel = velocity_traj.value(tspan[i]);
      Eigen::Vector3d p3 = p2 + R * vel;
      Ogre::Vector3 pos3(p3(0), p3(1), p3(2));
      vels_[i-1]->addPoint(pos2);
      vels_[i-1]->addPoint(pos3);
    }

    if(acc_vis_){
      Eigen::Vector3d acc = accel_traj.value(tspan[i]);
      Eigen::Vector3d p4 = p2 + R * acc;
      Ogre::Vector3 pos4(p4(0), p4(1), p4(2));
      accs_[i-1]->addPoint(pos2);
      accs_[i-1]->addPoint(pos4);
    }

    if(jrk_vis_){
      Eigen::Vector3d jrk = jrk_traj.value(tspan[i]);
      Eigen::Vector3d p5 = p2 + R * jrk;
      Ogre::Vector3 pos5(p5(0), p5(1), p5(2));
      jrks_[i-1]->addPoint(pos2);
      jrks_[i-1]->addPoint(pos5);
    }
  }
}


void PrimitiveTrajectoryVisual::addMessage(const erl_msgs::PrimitiveTrajectory &msg) {
  int prev_size = poss_.size();
  poss_.resize(num_ + prev_size);
  if(vel_vis_)
    vels_.resize(num_ + prev_size);
  if(acc_vis_)
    accs_.resize(num_ + prev_size);
 if(jrk_vis_)
    jrks_.resize(num_ + prev_size);
  
  for (int i = prev_size; i < (int)poss_.size(); i++) {
    poss_[i].reset(new rviz::BillboardLine(scene_manager_, frame_node_));
    if(vel_vis_)
      vels_[i].reset(new rviz::BillboardLine(scene_manager_, frame_node_));
    if(acc_vis_)
      accs_[i].reset(new rviz::BillboardLine(scene_manager_, frame_node_));
    if(jrk_vis_)
      jrks_[i].reset(new rviz::BillboardLine(scene_manager_, frame_node_));
  }

  trajectories::PiecewisePolynomial<double> trajectory = erl::fromROS(msg);
  auto velocity_traj = trajectory.derivative();
  auto accel_traj = velocity_traj.derivative();
  auto jrk_traj = accel_traj.derivative();

  decimal_t theta = M_PI / 2;
  Eigen::Matrix3d R;
  R << cos(theta), -sin(theta), 0,
      sin(theta), cos(theta), 0,
      0, 0, 1;

  auto tspan = linspace(trajectory.start_time(), trajectory.end_time(), num_);
  for (int i = 1; i < tspan.size(); i++) {
    Eigen::Vector3d p1 = trajectory.value(tspan[i-1]);
    Eigen::Vector3d p2 = trajectory.value(tspan[i]);

    Ogre::Vector3 pos1(p1(0, 0), p1(1, 0), p1(2, 0));
    Ogre::Vector3 pos2(p2(0, 0), p2(1, 0), p2(2, 0));
    poss_[i - 1]->addPoint(pos1);
    poss_[i - 1]->addPoint(pos2);

    if(vel_vis_){
      Eigen::Vector3d vel = velocity_traj.value(tspan[i]);
      Eigen::Vector3d p3 = p2 + R * vel;
      Ogre::Vector3 pos3(p3(0), p3(1), p3(2));
      vels_[i-1 + prev_size]->addPoint(pos2);
      vels_[i-1 + prev_size]->addPoint(pos3);
    }
    if(acc_vis_){
      Eigen::Vector3d acc = accel_traj.value(tspan[i]);
      Eigen::Vector3d p4 = p2 + R * acc;
      Ogre::Vector3 pos4(p4(0), p4(1), p4(2));
      accs_[i-1 + prev_size]->addPoint(pos2);
      accs_[i-1 + prev_size]->addPoint(pos4);
    }

    if(jrk_vis_){
      Eigen::Vector3d jrk = jrk_traj.value(tspan[i]);
      Eigen::Vector3d p5 = p2 + R * jrk;
      Ogre::Vector3 pos5(p5(0), p5(1), p5(2));
      jrks_[i-1 + prev_size]->addPoint(pos2);
      jrks_[i-1 + prev_size]->addPoint(pos5);
    }
  }
}

void PrimitiveTrajectoryVisual::setNum(int n) {
  num_ = n;
}

void PrimitiveTrajectoryVisual::setVelVis(bool vis) {
  vel_vis_ = vis;
}

void PrimitiveTrajectoryVisual::setAccVis(bool vis) {
  acc_vis_ = vis;
}

void PrimitiveTrajectoryVisual::setJrkVis(bool vis) {
  jrk_vis_ = vis;
}

void PrimitiveTrajectoryVisual::setFramePosition(const Ogre::Vector3 &position) {
  frame_node_->setPosition(position);
}

void PrimitiveTrajectoryVisual::setFrameOrientation(const Ogre::Quaternion &orientation) {
  frame_node_->setOrientation(orientation);
}

void PrimitiveTrajectoryVisual::setPosColor(float r, float g, float b, float a) {
  for (auto &it : poss_)
    it->setColor(r, g, b, a);
}

void PrimitiveTrajectoryVisual::setVelColor(float r, float g, float b, float a) {
  for (auto &it : vels_)
    it->setColor(r, g, b, a);
}

void PrimitiveTrajectoryVisual::setAccColor(float r, float g, float b, float a) {
  for (auto &it : accs_)
    it->setColor(r, g, b, a);
}

void PrimitiveTrajectoryVisual::setJrkColor(float r, float g, float b, float a) {
  for (auto &it : jrks_)
    it->setColor(r, g, b, a);
}

void PrimitiveTrajectoryVisual::setPosScale(float s) {
  for (auto &it : poss_)
    it->setLineWidth(s);
}

void PrimitiveTrajectoryVisual::setVelScale(float s) {
  for (auto &it : vels_)
    it->setLineWidth(s);
}

void PrimitiveTrajectoryVisual::setAccScale(float s) {
  for (auto &it : accs_)
    it->setLineWidth(s);
}

void PrimitiveTrajectoryVisual::setJrkScale(float s) {
  for (auto &it : jrks_)
    it->setLineWidth(s);
}

}

