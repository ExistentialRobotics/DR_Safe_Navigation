//
// Created by daizhirui on 7/10/23.
//

// 1. http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29
// 2. http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv#Creating_a_srv
// 3. http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28c%2B%2B%29
// 4. http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages

#include "erl_common/eigen.hpp"
#include "erl_geometry/occupancy_quadtree_drawer.hpp"  // must make sure the tree is included before the drawer
#include "erl_sdf_mapping/gp_occ_surface_mapping_2d.hpp"
#include "erl_sdf_mapping/gp_sdf_mapping_2d.hpp"
#define BOOST_BIND_GLOBAL_PLACEHOLDERS
#include <cv_bridge/cv_bridge.h>
#include <erl_sdf_mapping/PredictSdf.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>

#include <cmath>
#include <filesystem>
#include <mutex>

static const std::string kRosNodeName = "erl_sdf_mapping_node";
static const std::filesystem::path kRosNodeParamRoot = "/erl_sdf_mapping_node";

class RosNode {

    struct Parameters {
        double frequency = 0.0;
        int lidar_data_queue_size = 10;
        std::string lidar_data_topic_path = "/front/scan";
        std::string world_frame_name = "map";
        std::string lidar_frame_name = "front_laser";
        std::string surface_mapping_config_path;
        std::string sdf_mapping_config_path;
        bool visualize_quadtree = false;
        int visualize_frequency_divider = 10;
        std::string visualize_quadtree_config_path;
    };

    std::shared_ptr<ros::NodeHandle> m_node_handle_ = std::make_shared<ros::NodeHandle>();
    Parameters m_params_;
    std::shared_ptr<ros::AsyncSpinner> m_spinner_ = std::make_shared<ros::AsyncSpinner>(0);
    std::shared_ptr<ros::Subscriber> m_lidar_scan_sub_ = nullptr;
    std::shared_ptr<ros::ServiceServer> m_predict_sdf_server_ = nullptr;
    std::shared_ptr<ros::Publisher> m_quadtree_viz_pub_ = nullptr;
    tf2_ros::Buffer m_tf_buffer_;
    tf2_ros::TransformListener m_tf_listener_{m_tf_buffer_};

    std::shared_ptr<erl::sdf_mapping::GpOccSurfaceMapping2D::Setting> m_surface_mapping_setting_ = nullptr;
    std::shared_ptr<erl::sdf_mapping::GpOccSurfaceMapping2D> m_surface_mapping_ = nullptr;
    std::shared_ptr<erl::sdf_mapping::GpSdfMapping2D::Setting> m_sdf_mapping_setting_ = nullptr;
    std::shared_ptr<erl::sdf_mapping::GpSdfMapping2D> m_sdf_mapping_ = nullptr;

    using OccupancyQuadtreeDrawer = erl::geometry::OccupancyQuadtreeDrawer<erl::sdf_mapping::SurfaceMappingQuadtree>;
    std::shared_ptr<OccupancyQuadtreeDrawer::Setting> m_quadtree_drawer_setting_ = nullptr;
    std::shared_ptr<OccupancyQuadtreeDrawer> m_quadtree_drawer_ = nullptr;
    bool m_drawer_connected_ = false;
    std::vector<std::pair<cv::Point, cv::Point>> m_arrowed_lines_;
    cv::Mat m_cv_image_;
    int m_visualize_counter_ = 0;
    std::mutex m_quadtree_mutex_{};

public:
    explicit RosNode(std::shared_ptr<ros::NodeHandle> node_handle = nullptr)
        : m_node_handle_(std::move(node_handle)) {
        if (m_node_handle_ == nullptr) { m_node_handle_ = std::make_shared<ros::NodeHandle>(kRosNodeName); }

        (void) ros::Duration(1.0).sleep();
        LoadParameters();  // load parameters from ros parameter server

        m_surface_mapping_ = std::make_shared<erl::sdf_mapping::GpOccSurfaceMapping2D>(m_surface_mapping_setting_);
        m_sdf_mapping_ = std::make_shared<erl::sdf_mapping::GpSdfMapping2D>(m_surface_mapping_, m_sdf_mapping_setting_);
        if (m_params_.visualize_quadtree) {
            m_quadtree_drawer_ = std::make_shared<OccupancyQuadtreeDrawer>(m_quadtree_drawer_setting_);
            m_quadtree_drawer_->SetDrawTreeCallback(
                [&](const OccupancyQuadtreeDrawer *self, cv::Mat &img, erl::sdf_mapping::SurfaceMappingQuadtree::TreeIterator &it) {
                    unsigned int cluster_depth = m_surface_mapping_->GetQuadtree()->GetTreeDepth() - m_surface_mapping_->GetClusterLevel();
                    auto grid_map_info = self->GetGridMapInfo();
                    if (it->GetDepth() == cluster_depth) {
                        Eigen::Vector2i position_px = grid_map_info->MeterToPixelForPoints(Eigen::Vector2d(it.GetX(), it.GetY()));
                        cv::Point position_px_cv(position_px[0], position_px[1]);
                        cv::circle(img, position_px_cv, 2, cv::Scalar(0, 0, 255, 255), -1);  // draw surface point
                        return;
                    }
                    if (it->GetSurfaceData() == nullptr) { return; }
                    Eigen::Vector2i position_px = grid_map_info->MeterToPixelForPoints(it->GetSurfaceData()->position);
                    cv::Point position_px_cv(position_px[0], position_px[1]);
                    cv::circle(img, cv::Point(position_px[0], position_px[1]), 1, cv::Scalar(0, 0, 255, 255), -1);  // draw surface point
                    Eigen::Vector2i normal_px = grid_map_info->MeterToPixelForVectors(it->GetSurfaceData()->normal * 0.5);
                    cv::Point arrow_end_px(position_px[0] + normal_px[0], position_px[1] + normal_px[1]);
                    m_arrowed_lines_.emplace_back(position_px_cv, arrow_end_px);
                });
            m_quadtree_drawer_->SetDrawLeafCallback(
                [&](const OccupancyQuadtreeDrawer *self, cv::Mat &img, erl::sdf_mapping::SurfaceMappingQuadtree::LeafIterator &it) {
                    if (it->GetSurfaceData() == nullptr) { return; }
                    auto grid_map_info = self->GetGridMapInfo();
                    Eigen::Vector2i position_px = grid_map_info->MeterToPixelForPoints(it->GetSurfaceData()->position);
                    cv::Point position_px_cv(position_px[0], position_px[1]);
                    cv::circle(img, position_px_cv, 1, cv::Scalar(0, 0, 255, 255), -1);  // draw surface point
                    Eigen::Vector2i normal_px = grid_map_info->MeterToPixelForVectors(it->GetSurfaceData()->normal * 0.5);
                    cv::Point arrow_end_px(position_px[0] + normal_px[0], position_px[1] + normal_px[1]);
                    m_arrowed_lines_.emplace_back(position_px_cv, arrow_end_px);
                });
            auto grid_map_info = m_quadtree_drawer_->GetGridMapInfo();
            m_cv_image_ = cv::Mat(grid_map_info->Height(), grid_map_info->Width(), CV_8UC4, cv::Scalar(128, 128, 128, 255));
        }

        m_spinner_->start();

        m_lidar_scan_sub_ = std::make_shared<ros::Subscriber>(
            m_node_handle_->subscribe(m_params_.lidar_data_topic_path, m_params_.lidar_data_queue_size, &RosNode::SubCallbackLidarScan, this));
        (void) m_lidar_scan_sub_;

        m_predict_sdf_server_ = std::make_shared<ros::ServiceServer>(m_node_handle_->advertiseService("predict_sdf", &RosNode::SrvCallbackPredictSdf, this));
        (void) m_predict_sdf_server_;

        m_quadtree_viz_pub_ = std::make_shared<ros::Publisher>(m_node_handle_->advertise<sensor_msgs::Image>("quadtree_viz", 1));
    }

    static void
    Run() {
        ROS_INFO("Node %s is running", kRosNodeName.c_str());
        ros::waitForShutdown();
        // ros::Rate loop_rate(m_params_.frequency);
        // ros::Duration(5.0).sleep();
        // while (ros::ok()) {
        //     ros::spinOnce();
        //     loop_rate.sleep();
        // }
    }

private:
    // enable if T is not std::string
    template<typename T>
    std::enable_if_t<!std::is_same_v<T, std::string>, void>
    LoadParameter(const std::string &param_path, T &param, const bool must_exist = false) {
        if (m_node_handle_->hasParam(param_path)) {
            m_node_handle_->getParam(param_path, param);
            ROS_INFO("Load parameter %s: %s", param_path.c_str(), std::to_string(param).c_str());
        } else {
            if (must_exist) { ROS_ERROR("Parameter %s not found", param_path.c_str()); }
            ROS_INFO("Parameter %s not found, use default value: %s", param_path.c_str(), std::to_string(param).c_str());
        }
    }

    void
    LoadParameter(const std::string &param_path, std::string &param, const bool must_exist = false) const {
        if (m_node_handle_->hasParam(param_path)) {
            m_node_handle_->getParam(param_path, param);
            ROS_INFO("Load parameter %s: %s", param_path.c_str(), param.c_str());
        } else {
            if (must_exist) { ROS_ERROR("Parameter %s not found", param_path.c_str()); }
            ROS_INFO("Parameter %s not found, use default value: %s", param_path.c_str(), param.c_str());
        }
    }

#define LOAD_REQUIRED_PARAMETER(param_name) LoadParameter((kRosNodeParamRoot / #param_name).string(), m_params_.param_name, true)
#define LOAD_PARAMETER(param_name)          LoadParameter((kRosNodeParamRoot / #param_name).string(), m_params_.param_name)

    void
    LoadParameters() {
        LOAD_REQUIRED_PARAMETER(frequency);
        LOAD_PARAMETER(lidar_data_queue_size);
        LOAD_PARAMETER(lidar_data_topic_path);
        LOAD_PARAMETER(world_frame_name);
        LOAD_PARAMETER(lidar_frame_name);
        LOAD_PARAMETER(surface_mapping_config_path);
        LOAD_PARAMETER(sdf_mapping_config_path);
        m_surface_mapping_setting_ = std::make_shared<erl::sdf_mapping::GpOccSurfaceMapping2D::Setting>();
        if (!m_params_.surface_mapping_config_path.empty()) { m_surface_mapping_setting_->FromYamlFile(m_params_.surface_mapping_config_path); }
        m_sdf_mapping_setting_ = std::make_shared<erl::sdf_mapping::GpSdfMapping2D::Setting>();
        if (!m_params_.sdf_mapping_config_path.empty()) { m_sdf_mapping_setting_->FromYamlFile(m_params_.sdf_mapping_config_path); }
        LOAD_PARAMETER(visualize_quadtree);
        if (m_params_.visualize_quadtree) {
            LOAD_PARAMETER(visualize_frequency_divider);
            LOAD_REQUIRED_PARAMETER(visualize_quadtree_config_path);
            m_quadtree_drawer_setting_ = std::make_shared<OccupancyQuadtreeDrawer::Setting>();
            m_quadtree_drawer_setting_->FromYamlFile(m_params_.visualize_quadtree_config_path);
        }
    }

#undef LOAD_REQUIRED_PARAMETER
#undef LOAD_PARAMETER

    Eigen::Matrix23d
    GetLidarPose(const ros::Time &target_time) const {
        geometry_msgs::TransformStamped transform_stamped;
        while (true) {
            try {
                transform_stamped = m_tf_buffer_.lookupTransform(m_params_.world_frame_name, m_params_.lidar_frame_name, target_time, ros::Duration(5.0));
                break;
            } catch (tf2::TransformException &ex) {
                ROS_WARN("%s", ex.what());
                (void) ros::Duration(1.0).sleep();
            }
        }
        Eigen::Matrix4d eigen_transform_matrix = tf2::transformToEigen(transform_stamped).matrix();
        Eigen::Matrix23d pose = eigen_transform_matrix({0, 1}, {0, 1, 3});
        return pose;
    }

    void
    SubCallbackLidarScan(const sensor_msgs::LaserScanConstPtr &laser_scan) {
        auto t0 = std::chrono::high_resolution_clock::now();
        auto num_lines = static_cast<long>(laser_scan->ranges.size());
        Eigen::VectorXd angles = Eigen::VectorXd::LinSpaced(num_lines, laser_scan->angle_min, laser_scan->angle_max);
        Eigen::VectorXd ranges = Eigen::Map<const Eigen::VectorXf>((const float *) (laser_scan->ranges.data()), num_lines).cast<double>();
        auto lidar_pose = GetLidarPose(laser_scan->header.stamp);
        bool success = m_sdf_mapping_->Update(angles, ranges, lidar_pose);
        if (!success) {
            ROS_WARN("SDF mapping update failed");
            return;
        }

        if (m_params_.visualize_quadtree && m_visualize_counter_ == 0 && m_surface_mapping_->GetQuadtree()) {
            if (!m_drawer_connected_) {
                m_quadtree_drawer_->SetQuadtree(m_surface_mapping_->GetQuadtree());
                m_drawer_connected_ = true;
            }
            bool update_occupancy = m_surface_mapping_->GetSetting()->update_occupancy;
            m_cv_image_.setTo(cv::Scalar(128, 128, 128, 255));
            if (update_occupancy) {
                m_quadtree_drawer_->DrawLeaves(m_cv_image_);
            } else {
                m_quadtree_drawer_->DrawTree(m_cv_image_);
            }
            for (auto &[position_px_cv, arrow_end_px]: m_arrowed_lines_) {
                cv::arrowedLine(m_cv_image_, position_px_cv, arrow_end_px, cv::Scalar(0, 0, 255, 255), 1, 8, 0, 0.1);
            }
            cv_bridge::CvImage out_msg(laser_scan->header, sensor_msgs::image_encodings::BGRA8, m_cv_image_);
            m_quadtree_viz_pub_->publish(out_msg.toImageMsg());
            m_visualize_counter_ = (m_visualize_counter_ + 1) % m_params_.visualize_frequency_divider;
        }
        auto t1 = std::chrono::high_resolution_clock::now();
        double dt = std::chrono::duration<double, std::milli>(t1 - t0).count();
        ROS_INFO("Subscriber callback takes time: %f ms", dt);
    }

    bool
    SrvCallbackPredictSdf(erl_sdf_mapping::PredictSdf::Request &request, erl_sdf_mapping::PredictSdf::Response &response) {
        try {
            auto t0 = std::chrono::high_resolution_clock::now();
            auto num_queries = static_cast<long>(request.x.size());
            Eigen::Matrix2Xd positions_in(2, num_queries);
            for (long i = 0; i < num_queries; ++i) { positions_in.col(i) << request.x[i], request.y[i]; }
            Eigen::VectorXd sdf_out;
            Eigen::Matrix2Xd gradient_out;
            Eigen::Matrix3Xd variance_out, covariance_out;
            bool success = m_sdf_mapping_->Test(positions_in, sdf_out, gradient_out, variance_out, covariance_out);
            if (!success) {
                ROS_ERROR("SDF mapping test failed");
                return false;
            }
            response.sdf.resize(num_queries);
            response.gradient_x.resize(num_queries);
            response.gradient_y.resize(num_queries);
            response.var_sdf.resize(num_queries);
            response.var_gradient_x.resize(num_queries);
            response.var_gradient_y.resize(num_queries);
            if (covariance_out.cols() > 0) {
                response.cov_sdf_gradient_x.resize(num_queries);
                response.cov_sdf_gradient_y.resize(num_queries);
                response.cov_gradient_x_gradient_y.resize(num_queries);
            }
            for (long i = 0; i < num_queries; ++i) {
                response.sdf[i] = sdf_out[i];
                response.gradient_x[i] = gradient_out(0, i);
                response.gradient_y[i] = gradient_out(1, i);
                response.var_sdf[i] = variance_out(0, i);
                response.var_gradient_x[i] = variance_out(1, i);
                response.var_gradient_y[i] = variance_out(2, i);
                if (covariance_out.cols() > 0) {
                    response.cov_sdf_gradient_x[i] = covariance_out(0, i);
                    response.cov_sdf_gradient_y[i] = covariance_out(1, i);
                    response.cov_gradient_x_gradient_y[i] = covariance_out(2, i);
                }
            }
            auto t1 = std::chrono::high_resolution_clock::now();
            double dt = std::chrono::duration<double, std::milli>(t1 - t0).count();
            ROS_INFO("Service callback takes time: %f ms", dt);
            return true;
        } catch (const std::exception &e) {
            ROS_ERROR("Service callback failed: %s", e.what());
            return false;
        }
    }
};

int
main(int argc, char *argv[]) {
    ros::init(argc, argv, kRosNodeName);
    RosNode node;
    RosNode::Run();
    return 0;
}
