#ifndef __DEPTH_SIM_H__
#define __DEPTH_SIM_H__

#include <string.h>

#include <erl_utilities/erl_types.h>
#include <erl_utilities/erl_utils.h>
#include <erl_map/grid_map.h>
#include <erl_map/mesh_map.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

namespace erl {

class LaserScannerSim {
    erl::RndNumberGen rng;
    sensor_msgs::LaserScan scan_msg;
    erl::aligned_vector<Eigen::Vector3d> end_pts;
  public:
    LaserScannerSim( double range_min, double range_max,
                     double angle_min, double angle_max,
                     unsigned beam_count, std::string frame_id,
                     double scan_rate );
    
    template <typename T>
    inline sensor_msgs::LaserScan getScan(
      const erl::GridMap<T> & gridmap,
      const Eigen::Matrix2d& R, 
      const Eigen::Vector2d& p,
      double noise_sd );
      
    template <typename T>
    inline sensor_msgs::LaserScan getScan(
      const erl::GridMap<T> & gridmap,
      const Eigen::Matrix3d& R, 
      const Eigen::Vector3d& p,
      double noise_sd ); 

    inline sensor_msgs::LaserScan getScan(
      const erl::MeshMap & meshmap,
      const Eigen::Matrix3d& R, 
      const Eigen::Vector3d& p,
      double noise_sd );
     
  private:
    void addNoise(double noise_sd); 
};



class CloudScannerSim {
    erl::RndNumberGen rng;
    sensor_msgs::PointCloud2 cloud_msg;
    erl::aligned_vector<Eigen::Vector3d> end_pts;
    std::vector<double> range_bracket;
    std::vector<double> azimuth_bracket;
    std::vector<double> elevation_bracket;
    std::vector<double> az_el_increment_bracket;
  public:  
    CloudScannerSim( double range_min, double range_max,
                     double azimuth_min, double azimuth_max,
                     double elevation_min, double elevation_max,
                     unsigned az_beam_count, unsigned el_beam_count,
                     std::string frame_id );

    template <typename T>
    inline sensor_msgs::PointCloud2 getCloud(
      const erl::GridMap<T> & gridmap,
      const Eigen::Matrix3d& R, 
      const Eigen::Vector3d& p,
      double noise_sd );

    inline sensor_msgs::PointCloud2 getCloud(
      const erl::MeshMap & meshmap,
      const Eigen::Matrix3d& R, 
      const Eigen::Vector3d& p,
      double noise_sd );
  private:
    void addNoise(double noise_sd);
};


// http://docs.ros.org/kinetic/api/realsense2_camera/html/base__realsense__node_8cpp_source.html
class DepthScannerSim {
    erl::RndNumberGen rng;
    sensor_msgs::Image img_msg;
    erl::aligned_vector<Eigen::Vector3d> end_pts;
    std::vector<double> range_bracket;
    std::vector<double> focal_xy;
    std::vector<double> center_xy;
  public:
     DepthScannerSim( unsigned height, unsigned width,
                      double range_min, double range_max,
                      double fx, double fy,
                      double cx, double cy,
                      std::string frame_id );
    template <typename T>
    inline sensor_msgs::Image getDepth(
      const erl::GridMap<T> & gridmap,
      const Eigen::Matrix3d& R, 
      const Eigen::Vector3d& p,
      double noise_sd );  

    inline sensor_msgs::Image getDepth(
      const erl::MeshMap & meshmap,
      const Eigen::Matrix3d& R, 
      const Eigen::Vector3d& p,
      double noise_sd );

  private:
    void addNoise(double noise_sd);
};

} // erl


//////////////////////////////////////////////
//////// IMPLEMENTATION //////////////////////
//////////////////////////////////////////////


inline erl::LaserScannerSim::LaserScannerSim(
  double range_min, double range_max,
  double angle_min, double angle_max,
  unsigned beam_count, std::string frame_id,
  double scan_rate )
{
  scan_msg.angle_min = angle_min;
  scan_msg.angle_max = angle_max;
  scan_msg.angle_increment = (beam_count > 1) ? (angle_max - angle_min)/(beam_count - 1) : 0.0f;
  scan_msg.scan_time = (1.0/scan_rate);
  scan_msg.time_increment = scan_msg.scan_time / beam_count;
  scan_msg.range_min = range_min;
  scan_msg.range_max = range_max;
  scan_msg.header.frame_id = frame_id;
  scan_msg.ranges.resize(beam_count);
  
  // Compute the cartesian coordinates of the end points
  for( unsigned r = 0; r < beam_count; ++r ) {
    double ang = angle_min + r * scan_msg.angle_increment;
    end_pts.push_back(Eigen::Vector3d( range_max*std::cos(ang), range_max*std::sin(ang), 0.0 ) );
  }
}


template <typename T>
inline sensor_msgs::LaserScan erl::LaserScannerSim::getScan(
  const erl::GridMap<T> & gridmap,
  const Eigen::Matrix2d& R, 
  const Eigen::Vector2d& p,
  double noise_sd )
{
  if( gridmap.dim() < 2 || !gridmap.inMap(p) )
    return scan_msg;
  
  std::vector<double> pvec{p.x(),p.y()};
  for( unsigned r = 0; r < scan_msg.ranges.size(); ++r ) {
    Eigen::Vector2d ep = R*end_pts[r].head<2>() + p;     
    std::vector<double> ip;
    if( gridmap.intersectSegment( pvec, {ep.x(),ep.y()}, ip ) ) {
      float dist = std::sqrt((ip[0]-p.x())*(ip[0]-p.x()) + (ip[1]-p.y())*(ip[1]-p.y()));
      scan_msg.ranges[r] = std::max(std::min(dist,scan_msg.range_max), scan_msg.range_min);
    } else
      scan_msg.ranges[r] = scan_msg.range_max;
  }
  addNoise( noise_sd ); // add noise
  scan_msg.header.stamp = ros::Time::now();
  return scan_msg;  
}

template <typename T>
inline sensor_msgs::LaserScan erl::LaserScannerSim::getScan(
  const erl::GridMap<T> & gridmap,
  const Eigen::Matrix3d& R, 
  const Eigen::Vector3d& p,
  double noise_sd )
{
  
  if( gridmap.dim() < 3 || !gridmap.inMap(p) )
    return scan_msg;
  
  std::vector<double> pvec{p.x(),p.y(),p.z()};
  for( unsigned r = 0; r < scan_msg.ranges.size(); ++r ) {
    Eigen::Vector3d ep = R*end_pts[r] + p;     
    std::vector<double> ip;
    if( gridmap.intersectSegment( pvec, {ep.x(),ep.y(),ep.z()}, ip ) ) {
      float dist = std::sqrt((ip[0]-p.x())*(ip[0]-p.x()) + (ip[1]-p.y())*(ip[1]-p.y()) + (ip[2]-p.z())*(ip[2]-p.z()));
      scan_msg.ranges[r] = std::max(std::min(dist,scan_msg.range_max), scan_msg.range_min);
    } else
      scan_msg.ranges[r] = scan_msg.range_max;
  }
  addNoise( noise_sd ); // add noise
  scan_msg.header.stamp = ros::Time::now();
  return scan_msg;  
}


inline sensor_msgs::LaserScan erl::LaserScannerSim::getScan(
  const erl::MeshMap & meshmap,
  const Eigen::Matrix3d& R, 
  const Eigen::Vector3d& p,
  double noise_sd )
{
  erl::MeshMap::Point pt(p.x(),p.y(),p.z());
  for( unsigned r = 0; r < scan_msg.ranges.size(); ++r ) {    
    erl::MeshMap::Point emp( R(0,0)*end_pts[r](0) + R(0,1)*end_pts[r](1) + R(0,2)*end_pts[r](2),
                             R(1,0)*end_pts[r](0) + R(1,1)*end_pts[r](1) + R(1,2)*end_pts[r](2),
                             R(2,0)*end_pts[r](0) + R(2,1)*end_pts[r](1) + R(2,2)*end_pts[r](2) );
    erl::MeshMap::Ray ray_query(pt, emp);
    erl::MeshMap::Point ip;
    if( intersectRayAABBTree(ray_query, meshmap.tree(), ip) ) {
      float dist = std::sqrt((ip.x()-pt.x())*(ip.x()-pt.x()) + (ip.y()-pt.y())*(ip.y()-pt.y()) + (ip.z()-pt.z())*(ip.z()-pt.z()));
      scan_msg.ranges[r] = std::max(std::min(dist,scan_msg.range_max), scan_msg.range_min);
    } else
      scan_msg.ranges[r] = scan_msg.range_max;      
  }
  addNoise( noise_sd ); // add noise
  scan_msg.header.stamp = ros::Time::now();
  return scan_msg;  
}

inline void erl::LaserScannerSim::addNoise(double noise_sd)
{
  if( noise_sd > 0.0 )
    for( unsigned r = 0; r < scan_msg.ranges.size(); ++r )
      if(scan_msg.ranges[r] < scan_msg.range_max)
        scan_msg.ranges[r] = std::max( std::min(scan_msg.ranges[r] + static_cast<float>(rng.gaussian(0.0,noise_sd)),
                                         scan_msg.range_max), scan_msg.range_min);    
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////

inline erl::CloudScannerSim::CloudScannerSim(double range_min, double range_max,
                                             double azimuth_min, double azimuth_max,
                                             double elevation_min, double elevation_max,
                                             unsigned az_beam_count, unsigned el_beam_count,
                                             std::string frame_id )
  : range_bracket{range_min, range_max},
    azimuth_bracket{azimuth_min, azimuth_max},
    elevation_bracket{elevation_min, elevation_max}, 
    az_el_increment_bracket{ (az_beam_count > 1) ? (azimuth_min - azimuth_max)/(az_beam_count - 1) : 0.0,
                             (el_beam_count > 1) ? (elevation_min - elevation_max)/(el_beam_count - 1) : 0.0 }
{
  cloud_msg.height = el_beam_count;
  cloud_msg.width = az_beam_count;
  cloud_msg.is_bigendian = false;
  cloud_msg.is_dense = false;
  cloud_msg.header.frame_id = frame_id;
  sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
  modifier.setPointCloud2FieldsByString(1,"xyz");
  modifier.resize( cloud_msg.height * cloud_msg.width );
  
  end_pts.reserve( cloud_msg.height * cloud_msg.width );
  for( unsigned vr = 0; vr < el_beam_count; ++vr )
    for( unsigned hr = 0; hr < az_beam_count; ++hr )
    {
      // Convert from spherical to cartesian coordiantes 
      double azimuth = azimuth_min + hr*az_el_increment_bracket[0];
      double elevation = elevation_min + vr*az_el_increment_bracket[1];
      end_pts.push_back(Eigen::Vector3d( range_max * std::cos(elevation) * std::cos(azimuth),
                                         range_max * std::cos(elevation) * std::sin(azimuth),
                                         range_max * std::sin(elevation) ) );
    }
}


template <typename T>
inline sensor_msgs::PointCloud2 erl::CloudScannerSim::getCloud(
  const erl::GridMap<T> & gridmap,
  const Eigen::Matrix3d& R, 
  const Eigen::Vector3d& p,
  double noise_sd )
{
  if( gridmap.dim() < 3 || !gridmap.inMap(p) )
    return cloud_msg;

  Eigen::Matrix3d RT = R.transpose();
  std::vector<double> pvec{p.x(),p.y(),p.z()};
  sensor_msgs::PointCloud2Iterator<float> ix(cloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iy(cloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iz(cloud_msg, "z");
    
  for( unsigned vr = 0; vr < cloud_msg.height; ++vr )
    for( unsigned hr = 0; hr < cloud_msg.width; ++hr ) {
      Eigen::Vector3d ep = R*end_pts[hr + cloud_msg.width*vr] + p;
      std::vector<double> ip;
      if( gridmap.intersectSegment( pvec, {ep.x(),ep.y(),ep.z()}, ip ) ) {
        Eigen::Vector3d ipv = RT*(Eigen::Vector3d(ip.data())-p); // send the intersection point back to the image frame
        *ix = static_cast<float>(ipv.x());
        *iy = static_cast<float>(ipv.y());
        *iz = static_cast<float>(ipv.z());
      } else {
        *ix = std::numeric_limits<float>::quiet_NaN();
        *iy = std::numeric_limits<float>::quiet_NaN();
        *iz = std::numeric_limits<float>::quiet_NaN();
      }
      ++ix, ++iy, ++iz;
    }
  addNoise( noise_sd ); // add noise
  cloud_msg.header.stamp = ros::Time::now();
  return cloud_msg;    
}

inline sensor_msgs::PointCloud2 erl::CloudScannerSim::getCloud(
  const erl::MeshMap & meshmap,
  const Eigen::Matrix3d& R, 
  const Eigen::Vector3d& p,
  double noise_sd )
{
  Eigen::Matrix3d RT = R.transpose();
  erl::MeshMap::Point pt(p.x(),p.y(),p.z());
  sensor_msgs::PointCloud2Iterator<float> ix(cloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iy(cloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iz(cloud_msg, "z");
  
  for( unsigned vr = 0; vr < cloud_msg.height; ++vr )
    for( unsigned hr = 0; hr < cloud_msg.width; ++hr ) {
      erl::MeshMap::Point emp( R(0,0)*end_pts[hr + cloud_msg.width*vr](0) + 
                               R(0,1)*end_pts[hr + cloud_msg.width*vr](1) + 
                               R(0,2)*end_pts[hr + cloud_msg.width*vr](2),
                               R(1,0)*end_pts[hr + cloud_msg.width*vr](0) + 
                               R(1,1)*end_pts[hr + cloud_msg.width*vr](1) + 
                               R(1,2)*end_pts[hr + cloud_msg.width*vr](2),
                               R(2,0)*end_pts[hr + cloud_msg.width*vr](0) + 
                               R(2,1)*end_pts[hr + cloud_msg.width*vr](1) + 
                               R(2,2)*end_pts[hr + cloud_msg.width*vr](2) );
      erl::MeshMap::Ray ray_query(pt, emp);
      erl::MeshMap::Point ip;
      if( intersectRayAABBTree(ray_query, meshmap.tree(), ip) ) {
        Eigen::Vector3d ipv(ip.x(),ip.y(),ip.z());
        ipv = ipv-p; 
        double dist = ipv.norm();
        if( range_bracket[0] <= dist && dist <= range_bracket[1] ){
          ipv = RT*ipv; // send the intersection point back to the image frame
          *ix = static_cast<float>(ipv.x());
          *iy = static_cast<float>(ipv.y());
          *iz = static_cast<float>(ipv.z());          
        } else {
          *ix = std::numeric_limits<float>::quiet_NaN();
          *iy = std::numeric_limits<float>::quiet_NaN();
          *iz = std::numeric_limits<float>::quiet_NaN();        
        }
      } else {
        *ix = std::numeric_limits<float>::quiet_NaN();
        *iy = std::numeric_limits<float>::quiet_NaN();
        *iz = std::numeric_limits<float>::quiet_NaN();
      }
      ++ix, ++iy, ++iz;      
    }
  addNoise( noise_sd ); // add noise
  cloud_msg.header.stamp = ros::Time::now();
  return cloud_msg;      
}

inline void erl::CloudScannerSim::addNoise(double noise_sd)
{
  if( noise_sd > 0.0 ) {
    sensor_msgs::PointCloud2Iterator<float> ix(cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iy(cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iz(cloud_msg, "z");

    for( unsigned vr = 0; vr < cloud_msg.height; ++vr )
      for( unsigned hr = 0; hr < cloud_msg.width; ++hr )
      {
        if( !std::isnan(*ix) )
        {
          double curr_dist = std::sqrt((*ix)*(*ix)+(*iy)*(*iy)+(*iz)*(*iz));
          float dev = static_cast<float>( std::max( std::min(rng.gaussian(0.0,noise_sd),
                                                              range_bracket[1] - curr_dist), 
                                                     curr_dist - range_bracket[0] ));
          *ix = *ix + dev*end_pts[hr + cloud_msg.width*vr].x()/range_bracket[1];
          *iy = *iy + dev*end_pts[hr + cloud_msg.width*vr].y()/range_bracket[1];
          *iz = *iz + dev*end_pts[hr + cloud_msg.width*vr].z()/range_bracket[1];
        }
        ++ix, ++iy, ++iz;  
      }
  }  
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
inline erl::DepthScannerSim::DepthScannerSim( unsigned height, unsigned width,
                                              double range_min, double range_max,
                                              double fx, double fy,
                                              double cx, double cy,
                                              std::string frame_id )
  : range_bracket{range_min, range_max}, focal_xy{fx,fy}, center_xy{cx,cy}
{
  img_msg.header.frame_id = frame_id;
  img_msg.height = height;
  img_msg.width = width;
  img_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  img_msg.is_bigendian = false;
  img_msg.step = img_msg.width * sizeof(float);
  img_msg.data.resize(img_msg.step * img_msg.height);
  
  end_pts.reserve( img_msg.height * img_msg.width );
  for( unsigned vr = 0; vr < img_msg.height; ++vr )
    for( unsigned hr = 0; hr < img_msg.width; ++hr )
    {
      // Convert from pixel to cartesian coordiantes
      end_pts.push_back(Eigen::Vector3d( range_max * (hr + 0.5 - cx)/fx,
                                         range_max * (vr + 0.5 - cy)/fy,
                                         range_max ) );
    }
}


template <typename T>
inline sensor_msgs::Image erl::DepthScannerSim::getDepth(
  const erl::GridMap<T> & gridmap,
  const Eigen::Matrix3d& R, 
  const Eigen::Vector3d& p,
  double noise_sd )
{
  if( gridmap.dim() < 3 || !gridmap.inMap(p) )
    return img_msg;

  Eigen::Matrix3d RT = R.transpose();
  std::vector<double> pvec{p.x(),p.y(),p.z()};
  for( unsigned vr = 0; vr < img_msg.height; ++vr )
    for( unsigned hr = 0; hr < img_msg.width; ++hr ) {
      //float * pixel = &(img_msg.data[vr * img_msg.step + hr]);
      Eigen::Vector3d ep = R*end_pts[hr + img_msg.width*vr] + p;
      std::vector<double> ip;
      float pixel_depth = std::numeric_limits<float>::quiet_NaN();
      if( gridmap.intersectSegment( pvec, {ep.x(),ep.y(),ep.z()}, ip ) ) {
        double depth = RT(2,0)*(ip[0] - p.x()) + RT(2,1)*(ip[1] - p.y()) + RT(2,2)*(ip[2] - p.z());
        if( range_bracket[0] <= depth && depth <= range_bracket[1] )
          pixel_depth = static_cast<float>(depth);
      }
      std::memcpy(&img_msg.data[vr * img_msg.step + hr], &pixel_depth, sizeof(pixel_depth));
    }
  addNoise( noise_sd ); // add noise
  img_msg.header.stamp = ros::Time::now();
  return img_msg; 
}


inline sensor_msgs::Image erl::DepthScannerSim::getDepth(
  const erl::MeshMap & meshmap,
  const Eigen::Matrix3d& R, 
  const Eigen::Vector3d& p,
  double noise_sd )
{
  Eigen::Matrix3d RT = R.transpose();
  erl::MeshMap::Point pt(p.x(),p.y(),p.z());
  for( unsigned vr = 0; vr < img_msg.height; ++vr )
    for( unsigned hr = 0; hr < img_msg.width; ++hr ) {
      //float * pixel = &(img_msg.data[vr * img_msg.step + hr]);
      erl::MeshMap::Point emp( R(0,0)*end_pts[hr + img_msg.width*vr](0) + 
                               R(0,1)*end_pts[hr + img_msg.width*vr](1) + 
                               R(0,2)*end_pts[hr + img_msg.width*vr](2),
                               R(1,0)*end_pts[hr + img_msg.width*vr](0) + 
                               R(1,1)*end_pts[hr + img_msg.width*vr](1) + 
                               R(1,2)*end_pts[hr + img_msg.width*vr](2),
                               R(2,0)*end_pts[hr + img_msg.width*vr](0) + 
                               R(2,1)*end_pts[hr + img_msg.width*vr](1) + 
                               R(2,2)*end_pts[hr + img_msg.width*vr](2) );
      erl::MeshMap::Ray ray_query(pt, emp);
      erl::MeshMap::Point ip;
      float pixel_depth = std::numeric_limits<float>::quiet_NaN();
      if( intersectRayAABBTree(ray_query, meshmap.tree(), ip) ) {
        double depth = RT(2,0)*(ip.x() - p.x()) + RT(2,1)*(ip.y() - p.y()) + RT(2,2)*(ip.z() - p.z());
        if( range_bracket[0] <= depth && depth <= range_bracket[1] )
          pixel_depth = static_cast<float>(depth);
      }
      std::memcpy(&img_msg.data[vr * img_msg.step + hr], &pixel_depth, sizeof(pixel_depth));
    }
  addNoise( noise_sd ); // add noise
  img_msg.header.stamp = ros::Time::now();
  return img_msg; 
}
      

inline void erl::DepthScannerSim::addNoise(double noise_sd)
{
  if( noise_sd > 0.0 ) {
    for( unsigned vr = 0; vr < img_msg.height; ++vr )
      for( unsigned hr = 0; hr < img_msg.width; ++hr ) {
      
        //float * pixel = &(img_msg.data[vr * img_msg.step + hr]);
        float pixel_depth;
        std::memcpy(&pixel_depth, &img_msg.data[vr * img_msg.step + hr], sizeof(pixel_depth));
        if(pixel_depth < static_cast<float>(range_bracket[1])) {
          pixel_depth = static_cast<float>(std::max(std::min(pixel_depth + rng.gaussian(0.0,noise_sd), range_bracket[1]),
                                               range_bracket[0]));
          std::memcpy(&img_msg.data[vr * img_msg.step + hr], &pixel_depth, sizeof(pixel_depth));
        }
      }
  }
}



#endif
