
#ifndef __ERL_MSG_UTILS_H_
#define __ERL_MSG_UTILS_H_
#include <erl_map/grid_map.h>
#include <erl_map/mesh_map.h>
#include <erl_utilities/erl_types.h>
#include <erl_utilities/trajectories/piecewise_polynomial.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pcl_msgs/PolygonMesh.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <shape_msgs/Mesh.h>

#include <Eigen/Geometry>
#include <unordered_map>
// #include <erl_estimation/filters/tracking/multi_target_filter.h>
// #include <erl_estimation/filters/tracking/landmark_filter_3d.h>

#include <erl_msgs/GridMap.h>
#include <erl_msgs/MultiTargetBelief.h>
#include <erl_msgs/PointTrajectory.h>
#include <erl_msgs/PointTrajectoryArray.h>
#include <erl_msgs/PrimitiveTrajectory.h>

namespace erl {

template <size_t Rows, size_t Cols>
inline void boostArrayToEigen(const boost::array<double, Rows * Cols> &array,
                              Eigen::Matrix<double, Rows, Cols> &matrix) {
  // assume that the boost array is row major, as ROS provides
  matrix = Eigen::Map<const Eigen::Matrix<double, Rows, Cols, Eigen::RowMajor>>(array.elems);
}

template <size_t Rows, size_t Cols>
inline void eigenToBoostArray(const Eigen::Matrix<double, Rows, Cols> &matrix,
                              boost::array<double, Rows * Cols> &array) {
  // copy in row major order
  for (size_t i = 0; i < Rows; i++) {
    for (size_t j = 0; j < Cols; j++) {
      array[i * Cols + j] = matrix(i, j);
    }
  }
}

// This function has not been tested...
inline void cloud2image(const sensor_msgs::PointCloud2 &cloud, sensor_msgs::Image &image) {
  int rgb_index = -1;
  // Get the index we need
  for (size_t d = 0; d < cloud.fields.size(); ++d)
    if (cloud.fields[d].name == "rgb") {
      rgb_index = static_cast<int>(d);
      break;
    }
  if (rgb_index == -1) return;
  if (cloud.width == 0 && cloud.height == 0) return;

  image.header = cloud.header;
  image.height = cloud.height;
  image.width = cloud.width;
  image.encoding = "bgr8";
  image.is_bigendian = cloud.is_bigendian;
  image.step = image.width * sizeof(uint8_t) * 3;
  image.data.resize(image.step * image.height);
  int rgb_offset = cloud.fields[rgb_index].offset;
  int point_step = cloud.point_step;
  for (size_t y = 0; y < cloud.height; ++y)
    for (size_t x = 0; x < cloud.width; ++x, rgb_offset += point_step) {
      uint8_t *pixel = &(image.data[y * image.step + x * 3]);
      memcpy(pixel, &(cloud.data[rgb_offset]), 3 * sizeof(uint8_t));
    }
}

inline void fromROS(const geometry_msgs::PoseWithCovariance &msg, Eigen::Vector3d &position,
                    Eigen::Quaterniond &orientation, Eigen::Matrix<double, 6, 6> &covariance) {
  position << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;

  orientation.x() = msg.pose.orientation.x;
  orientation.y() = msg.pose.orientation.y;
  orientation.z() = msg.pose.orientation.z;
  orientation.w() = msg.pose.orientation.w;

  boostArrayToEigen<6, 6>(msg.covariance, covariance);
}

inline void fromROS(const sensor_msgs::Imu &msg, Eigen::Vector3d &omega, Eigen::Matrix3d &omega_cov,
                    Eigen::Vector3d &accel, Eigen::Matrix3d &accel_cov) {
  omega << msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z;
  accel << msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z;

  boostArrayToEigen<3, 3>(msg.angular_velocity_covariance, omega_cov);
  boostArrayToEigen<3, 3>(msg.linear_acceleration_covariance, accel_cov);
}

inline erl_msgs::PointTrajectory toROS(const aligned_vector<Eigen::Vector3f> &point_trajectory) {
  erl_msgs::PointTrajectory msg;
  for (const auto &itt : point_trajectory) {
    geometry_msgs::Point pt;
    pt.x = itt(0);
    pt.y = itt(1);
    pt.z = itt(2);
    msg.points.push_back(pt);
  }
  return msg;
}

inline aligned_vector<Eigen::Vector3f> fromROS(const erl_msgs::PointTrajectory &msg) {
  aligned_vector<Eigen::Vector3f> point_trajectory;
  for (const auto &it : msg.points) point_trajectory.push_back(Eigen::Vector3f(it.x, it.y, it.z));
  return point_trajectory;
}

inline erl_msgs::PointTrajectoryArray toROS(const aligned_vector<aligned_vector<Eigen::Vector3f>> &point_trajectories) {
  erl_msgs::PointTrajectoryArray msg;
  for (const auto &it : point_trajectories) {
    erl_msgs::PointTrajectory pt_traj_msg;
    for (const auto &itt : it) {
      geometry_msgs::Point pt;
      pt.x = itt(0);
      pt.y = itt(1);
      pt.z = itt(2);
      pt_traj_msg.points.push_back(pt);
    }
    msg.trajectories.push_back(pt_traj_msg);
  }
  return msg;
}

inline erl_msgs::PointTrajectoryArray toROS(
    const std::vector<std::pair<std::string, aligned_vector<Eigen::Vector3f>>> &point_trajectories) {
  erl_msgs::PointTrajectoryArray msg;
  for (const auto &it : point_trajectories) {
    erl_msgs::PointTrajectory pt_traj_msg;
    pt_traj_msg.name = it.first;
    for (const auto &itt : it.second) {
      geometry_msgs::Point pt;
      pt.x = itt(0);
      pt.y = itt(1);
      pt.z = itt(2);
      pt_traj_msg.points.push_back(pt);
    }
    msg.trajectories.push_back(pt_traj_msg);
  }
  return msg;
}

inline erl_msgs::PrimitiveTrajectory toROS(const trajectories::PiecewisePolynomial<double> &erl_traj) {
  erl_msgs::PrimitiveTrajectory msg;
  const auto seg_times = erl_traj.get_segment_times();

  for (int i = 0; i < erl_traj.get_number_of_segments(); ++i) {
    // Construct One Primitive at a Time.
    erl_msgs::Primitive prim;

    const auto c_x = erl_traj.getPolynomial(i, 0, 0).GetCoefficients();
    const auto c_y = erl_traj.getPolynomial(i, 1, 0).GetCoefficients();
    const auto c_z = erl_traj.getPolynomial(i, 2, 0).GetCoefficients();

    // Copy Coefficients
    for (int n = 0; n < c_x.rows(); n++) {
      unsigned c_idx = n;  // Compute Coefficient Index.
      prim.cx.push_back(c_x(c_idx, 0));
      prim.cy.push_back(c_y(c_idx, 0));
      prim.cz.push_back(c_z(c_idx, 0));
    }

    prim.t = seg_times[i + 1] - seg_times[i];  // Write Time
    // Push the Primitive
    msg.primitives.push_back(prim);
  }
  return msg;
}

inline erl_msgs::PrimitiveTrajectory toROS(const trajectories::PiecewisePolynomial<double> &erl_traj, const double z) {
  erl_msgs::PrimitiveTrajectory msg;
  const auto seg_times = erl_traj.get_segment_times();

  for (int i = 0; i < erl_traj.get_number_of_segments(); ++i) {
    // Construct One Primitive at a Time.
    erl_msgs::Primitive prim;

    const auto c_x = erl_traj.getPolynomial(i, 0, 0).GetCoefficients();
    const auto c_y = erl_traj.getPolynomial(i, 1, 0).GetCoefficients();

    // Copy Coefficients
    for (int n = 0; n < c_x.rows(); n++) {
      unsigned c_idx = n;  // Compute Coefficient Index.
      prim.cx.push_back(c_x(c_idx, 0));
      prim.cy.push_back(c_y(c_idx, 0));
      prim.cz.push_back(0);
    }

    prim.cz[0] = z;  // Assign the Fixed Z height.

    prim.t = seg_times[i + 1] - seg_times[i];  // Write Time
    // Push the Primitive
    msg.primitives.push_back(prim);
  }
  return msg;
}

inline trajectories::PiecewisePolynomial<double> fromROS(const erl_msgs::PrimitiveTrajectory &ros_traj) {
  const unsigned n_prims = ros_traj.primitives.size();
  std::vector<double> seg_times = {0};
  const unsigned N = ros_traj.primitives[0].cx.size();
  typedef Eigen::Matrix<trajectories::Polynomial<double>, Eigen::Dynamic, Eigen::Dynamic> PolynomialVector;
  std::vector<PolynomialVector> polynomials(n_prims, PolynomialVector(3, 1));  // 3D
  Eigen::MatrixXd coeff(N, 1);

  for (unsigned i = 0; i < n_prims; ++i) {
    // Store the Coefficients in a temporary vector
    std::vector<std::vector<double>> msg_coefs = {ros_traj.primitives[i].cx, ros_traj.primitives[i].cy,
                                                  ros_traj.primitives[i].cz};
    for (int d = 0; d < 3; ++d) {
      // Construct One Polynomial at a Time.
      for (unsigned n = 0; n < N; ++n) coeff(n, 0) = msg_coefs[d][n];

      polynomials[i](d) = trajectories::Polynomial<double>(coeff);
    }
    seg_times.push_back(seg_times[i] + ros_traj.primitives[i].t);
  }
  return trajectories::PiecewisePolynomial<double>(polynomials, seg_times);
}

/**
 * Converts a gridmap_msg ROS message to an erl::GridMap object.
 * @param gridmap_msg The ROS message containing the Grid Map data.
 * @param copy_data A flag indicating whether the data needs to be copied into the map.
 * @return The resulting erl::GridMap.
 */
inline erl::GridMap<int8_t> fromROS(const erl_msgs::GridMap &gridmap_msg, bool copy_data = true) {
  std::vector<int> dim;
  std::vector<double> origin;
  std::vector<double> res;

  std::copy(gridmap_msg.dim.begin(), gridmap_msg.dim.end(), std::back_inserter(dim));
  std::copy(gridmap_msg.origin.begin(), gridmap_msg.origin.end(), std::back_inserter(origin));
  std::copy(gridmap_msg.resolution.begin(), gridmap_msg.resolution.end(), std::back_inserter(res));

  erl::GridMap<int8_t> grid_map(dim, origin, res, gridmap_msg.rowmajor);
  if (copy_data)  // Copy the Map data only if requested.
  {
    grid_map.setMap(gridmap_msg.data);
  }

  return grid_map;
}

/**
 * Converts an erl::GridMap to a GridMap message for publishing over ROS.
 * @tparam T The type of the map data.
 * @param gridmap The erl::GridMap to be converted.
 * @return The resulting message type.
 */
template <typename T>
inline erl_msgs::GridMap toROS(const erl::GridMap<T> &gridmap) {
  erl_msgs::GridMap gridmap_msg;

  std::copy(gridmap.size().begin(), gridmap.size().end(), std::back_inserter(gridmap_msg.dim));
  std::copy(gridmap.origin().begin(), gridmap.origin().end(), std::back_inserter(gridmap_msg.origin));
  std::copy(gridmap.res().begin(), gridmap.res().end(), std::back_inserter(gridmap_msg.resolution));
  gridmap_msg.rowmajor = gridmap.rowmajor();
  for (const auto &item : gridmap.map()) {
    if (std::is_same<T, char>::value)
      gridmap_msg.data.push_back(item - 48);  // '0' is numerically 48
    else
      gridmap_msg.data.push_back(item);
  }

  return gridmap_msg;
}

/**
 * Converts a nav_msgs::OccupancyGrid to an erl::GridMap object.
 * Note that the orientation in info.origin is ignored.
 * @param gridmap_msg The ROS message containing the Grid Map data.
 * @param copy_data A flag indicating whether the data needs to be copied into the map.
 * @return The resulting erl::GridMap.
 */
inline erl::GridMap<uint8_t> fromROS(const nav_msgs::OccupancyGrid &gridmap_msg, bool copy_data = true,
                                     bool binary_map = false, int threshold = 50) {
  std::vector<int> sz{erl::odd_ceil(gridmap_msg.info.width), erl::odd_ceil(gridmap_msg.info.height)};
  std::vector<double> res{gridmap_msg.info.resolution, gridmap_msg.info.resolution};
  std::vector<double> origin{gridmap_msg.info.origin.position.x + sz[0] * (gridmap_msg.info.resolution / 2),
                             gridmap_msg.info.origin.position.y + sz[1] * (gridmap_msg.info.resolution / 2)};

  // By default, Occupancy map is in row major order.
  erl::GridMap<uint8_t> grid_map(sz, origin, res, true);
  if (copy_data)  // Copy the Map data only if requested.
  {
    grid_map.initZeroMap();
    for (int i = 0; i < static_cast<int>(gridmap_msg.info.width); ++i)
      for (int j = 0; j < static_cast<int>(gridmap_msg.info.height); ++j) {
        auto cell_val = gridmap_msg.data[j * gridmap_msg.info.width + i];
        if (binary_map) {
          // unknown cell has value 255 (-1). TODO: Change this if the datatype is not uint8_t anymore.
          grid_map.setMapValueCell({i, j}, (cell_val > threshold && cell_val < 255) ? 1 : 0);
        } else
          grid_map.setMapValueCell({i, j}, cell_val);
      }
  }
  return grid_map;
}

/**
 * Converts an erl::GridMap to nav_msgs::OccupancyGrid for publishing over ROS.
 * Assumes the provided gridmap is 2D and the resolution is equal in both dimensions.
 * @tparam T The type of the map data.
 * @param gridmap The erl::GridMap to be converted.
 * @return The resulting message type.
 */
template <typename T>
inline nav_msgs::OccupancyGrid toROSOccupancyGrid(const erl::GridMap<T> &gridmap, bool binary_map = false) {
  nav_msgs::OccupancyGrid gridmap_msg;

  if (gridmap.dim() != 2 || gridmap.res()[0] != gridmap.res()[1]) return gridmap_msg;

  gridmap_msg.info.map_load_time = ros::Time::now();
  gridmap_msg.info.resolution = gridmap.res()[0];
  gridmap_msg.info.width = gridmap.size()[0];
  gridmap_msg.info.height = gridmap.size()[1];
  std::vector<double> lower_left = gridmap.cells2meters({0, 0});
  gridmap_msg.info.origin.position.x = lower_left[0];
  gridmap_msg.info.origin.position.y = lower_left[1];

  // If binary_map (0 and 1), multiply the value by the maximum value in ROS Occupancy Grid (100).
  int mul_grid_val = binary_map ? 100 : 1;
  if (gridmap.rowmajor()) {
    for (const auto &item : gridmap.map()) {
      if (std::is_same<T, char>::value)
        gridmap_msg.data.push_back((item - 48) * mul_grid_val);  // '0' is numerically 48
      else
        gridmap_msg.data.push_back(item * mul_grid_val);
    }
  } else {
    const size_t mapsz = gridmap.map().size();
    gridmap_msg.data.resize(mapsz);
    for (size_t k = 0; k < mapsz; ++k) {
      if (std::is_same<T, char>::value)
        gridmap_msg.data[gridmap.subv2ind_rowmajor(gridmap.ind2subv_colmajor(k))] =
            (gridmap.map()[k] - 48) * mul_grid_val;
      else
        gridmap_msg.data[gridmap.subv2ind_rowmajor(gridmap.ind2subv_colmajor(k))] = gridmap.map()[k] * mul_grid_val;
    }
  }
  return gridmap_msg;
  // TODO: The resulting Occupancy Map is still perpendicular with the original map. Fix this.
}

/**
 * Converts a shape_msgs::Mesh to an erl::MeshMap object.
 * @param meshmap_msg The ROS message containing the MeshMap data.
 * @return The resulting erl::MeshMap.
 */
inline erl::MeshMap fromROS(const shape_msgs::Mesh &meshmap_msg) {
  std::vector<erl::MeshMap::Point> vertices;
  vertices.reserve(meshmap_msg.vertices.size());
  for (unsigned k = 0; k < meshmap_msg.vertices.size(); ++k)
    vertices.push_back(
        erl::MeshMap::Point(meshmap_msg.vertices[k].x, meshmap_msg.vertices[k].y, meshmap_msg.vertices[k].z));

  std::vector<boost::array<uint32_t, 3>::const_iterator> faces_begin;
  faces_begin.reserve(meshmap_msg.triangles.size());
  std::vector<boost::array<uint32_t, 3>::const_iterator> faces_end;
  faces_end.reserve(meshmap_msg.triangles.size());
  for (unsigned k = 0; k < meshmap_msg.triangles.size(); ++k) {
    faces_begin.push_back(meshmap_msg.triangles[k].vertex_indices.begin());
    faces_end.push_back(meshmap_msg.triangles[k].vertex_indices.end());
  }

  erl::MeshMap meshmap;
  meshmap.addFaces(vertices, faces_begin, faces_end);
  return meshmap;
}

/**
 * Converts a pcl_msgs::PolygonMesh to an erl::MeshMap object.
 * @param meshmap_msg The ROS message containing the MeshMap data.
 * @return The resulting erl::MeshMap.
 */
inline erl::MeshMap fromROS(const pcl_msgs::PolygonMesh &meshmap_msg) {
  // https://answers.ros.org/question/11556/datatype-to-access-pointcloud2/
  std::vector<erl::MeshMap::Point> vertices;
  vertices.reserve(meshmap_msg.cloud.height * meshmap_msg.cloud.width);
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(meshmap_msg.cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(meshmap_msg.cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(meshmap_msg.cloud, "z");
  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
    vertices.push_back(erl::MeshMap::Point(*iter_x, *iter_y, *iter_z));

  std::vector<std::vector<uint32_t>::const_iterator> faces_begin;
  faces_begin.reserve(meshmap_msg.polygons.size());
  std::vector<std::vector<uint32_t>::const_iterator> faces_end;
  faces_end.reserve(meshmap_msg.polygons.size());
  for (unsigned k = 0; k < meshmap_msg.polygons.size(); ++k) {
    faces_begin.push_back(meshmap_msg.polygons[k].vertices.begin());
    faces_end.push_back(meshmap_msg.polygons[k].vertices.end());
  }

  erl::MeshMap meshmap;
  meshmap.addFaces(vertices, faces_begin, faces_end);
  return meshmap;
}

/**
 * Converts an erl::MeshMap to spcl_msgs::PolygonMesh for publishing over ROS.
 * @param meshmap The erl::MeshMap to be converted.
 * @return The resulting message type.
 */
inline pcl_msgs::PolygonMesh toROS(const erl::MeshMap &meshmap) {
  // https://answers.ros.org/question/219876/using-sensor_msgspointcloud2-natively/
  pcl_msgs::PolygonMesh meshmap_msg;

  meshmap_msg.cloud.height = 1;
  meshmap_msg.cloud.width = meshmap.mesh().number_of_vertices();
  meshmap_msg.cloud.is_bigendian = false;
  meshmap_msg.cloud.is_dense = true;

  sensor_msgs::PointCloud2Modifier modifier(meshmap_msg.cloud);
  modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  modifier.resize(meshmap.mesh().number_of_vertices());

  // iterators
  sensor_msgs::PointCloud2Iterator<float> out_x(meshmap_msg.cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> out_y(meshmap_msg.cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> out_z(meshmap_msg.cloud, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> out_r(meshmap_msg.cloud, "r");
  sensor_msgs::PointCloud2Iterator<uint8_t> out_g(meshmap_msg.cloud, "g");
  sensor_msgs::PointCloud2Iterator<uint8_t> out_b(meshmap_msg.cloud, "b");
  size_t idx = 0;

  // copy point coordinates into the point cloud
  std::unordered_map<erl::MeshMap::Mesh::Vertex_index, size_t> idx_map;
  for (erl::MeshMap::Mesh::Vertex_index vd : meshmap.mesh().vertices()) {
    const erl::MeshMap::Point &pt = meshmap.mesh().point(vd);
    idx_map.insert({vd, idx});

    *out_x = static_cast<float>(pt.x());
    *out_y = static_cast<float>(pt.y());
    *out_z = static_cast<float>(pt.z());

    // TODO: implement retrieving color from mesh vertices
    *out_r = uint8_t(0);
    *out_g = uint8_t(0);
    *out_b = uint8_t(0);

    // increment
    ++out_x;
    ++out_y;
    ++out_z;
    ++out_r;
    ++out_g;
    ++out_b;
    ++idx;
  }

  // iterate over the faces
  meshmap_msg.polygons.resize(meshmap.mesh().number_of_faces());
  {
    unsigned i_face = 0;
    for (erl::MeshMap::Mesh::Face_index fd : meshmap.mesh().faces()) {
      for (erl::MeshMap::Mesh::Vertex_index vd : vertices_around_face(meshmap.mesh().halfedge(fd), meshmap.mesh()))
        meshmap_msg.polygons[i_face].vertices.push_back(idx_map.at(vd));
      ++i_face;
    }
  }

  //  typedef Surface_mesh::Face_index face_descriptor;
  //  printf("Number of faces: %d\n", smesh.number_of_faces());
  //  std::vector<unsigned>* faces = new std::vector<unsigned>[smesh.number_of_faces()];
  //  std::cout << "Iterate over faces\n";
  //  {
  //    unsigned i_face = 0;
  //    BOOST_FOREACH(face_descriptor fd, smesh.faces()){
  //      BOOST_FOREACH(vertex_descriptor vd, vertices_around_face(smesh.halfedge(fd), smesh)){
  //        printf("vertex: %u\n", vd);
  //        faces[i_face].push_back(vd);
  //      }
  //      i_face++;
  //    }
  //  }

  return meshmap_msg;
}

/**
 * Converts an erl::MeshMap to shape_msgs::Mesh for publishing over ROS.
 * @param meshmap The erl::MeshMap to be converted.
 * @return The resulting message type.
 */
inline shape_msgs::Mesh toROSMesh(const erl::MeshMap &meshmap) {
  shape_msgs::Mesh meshmap_msg;

  // copy point coordinates
  meshmap_msg.vertices.resize(meshmap.mesh().number_of_vertices());
  std::unordered_map<erl::MeshMap::Mesh::Vertex_index, uint32_t> idx_map;
  {
    uint32_t idx = 0;
    for (erl::MeshMap::Mesh::Vertex_index vd : meshmap.mesh().vertices()) {
      const erl::MeshMap::Point &pt = meshmap.mesh().point(vd);
      idx_map.insert({vd, idx});
      meshmap_msg.vertices[idx].x = static_cast<float>(pt.x());
      meshmap_msg.vertices[idx].y = static_cast<float>(pt.y());
      meshmap_msg.vertices[idx].z = static_cast<float>(pt.z());
      ++idx;
    }
  }

  // iterate over the faces
  // TODO: Warning; assumes the mesh is triangular
  meshmap_msg.triangles.resize(meshmap.mesh().number_of_faces());
  {
    unsigned i_face = 0;
    for (erl::MeshMap::Mesh::Face_index fd : meshmap.mesh().faces()) {
      unsigned i_vert = 0;
      for (erl::MeshMap::Mesh::Vertex_index vd : vertices_around_face(meshmap.mesh().halfedge(fd), meshmap.mesh())) {
        meshmap_msg.triangles[i_face].vertex_indices[i_vert] = idx_map.at(vd);
        ++i_vert;
        if (i_vert > 2) break;
      }
      ++i_face;
    }
  }
  return meshmap_msg;
}

// commented out by zhl to get rid of dependency of erl_sensing
// inline erl::MultiLandmarkFilter<int8_t> fromROS(const erl_msgs::MultiTargetBelief &msg) {
//   erl::MultiLandmarkFilter<int8_t> filter;

//   for (size_t i = 0; i < msg.ids.size(); i++) {
//     // Gather Data on each target from message.
//     unsigned id = msg.ids[i];
//     Eigen::Vector3d mean;
//     mean << msg.poses[i].pose.position.x, msg.poses[i].pose.position.y, msg.poses[i].pose.position.z;
//     Eigen::Matrix3d cov;
//     cov << msg.poses[i].covariance[0], msg.poses[i].covariance[1], msg.poses[i].covariance[2],
//         msg.poses[i].covariance[6], msg.poses[i].covariance[7], msg.poses[i].covariance[8],
//         msg.poses[i].covariance[12], msg.poses[i].covariance[13], msg.poses[i].covariance[14];

//     // Add a landmark.
//     auto landmark = std::static_pointer_cast<erl::GaussianFilter<erl::SE3Pose, erl::LandmarkTarget<int8_t>>>(
//           std::make_shared<erl::LandmarkFilter3D<int8_t>>(id, mean, cov, nullptr, nullptr));
//     // Add the target filter to the multiFilter
//     filter.addTarget(id, landmark);
//   }

//   return filter;
// }

}  // namespace erl

#endif
