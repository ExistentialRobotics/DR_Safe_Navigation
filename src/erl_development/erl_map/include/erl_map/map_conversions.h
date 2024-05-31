#ifndef __MAP_CONVERSIONS_H_
#define __MAP_CONVERSIONS_H_

#include "erl_map/grid_map.h"
//#include "erl_map/mesh_map.h"
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Polygon_mesh_processing/bbox.h>
#include <unordered_map>

namespace erl
{

template <typename T, typename Point>
inline CGAL::Surface_mesh<Point> toMeshMap(const GridMap<T>& gridmap);


template <typename T, typename Point>
inline GridMap<T> toGridMap( const CGAL::Surface_mesh<Point>& mesh,
                             const std::vector<double>& resolution,
                             int fp_pow = 16 );

template <typename T>
inline GridMap<int8_t> toInt8GridMap( const GridMap<T>& gridmap );

template <typename PT>
inline bool isInTriangle(int i, int j, const PT& p0, const PT& p1, const PT& p2, int& _depth);




//////////////////////////////////////////////////////////////////////////////////
// Implementation
//////////////////////////////////////////////////////////////////////////////////
namespace
{

template <class Vertex>
struct swizzle_xyz
{
  inline Vertex forward(const Vertex& v)  const { return v; }
  inline Vertex backward(const Vertex& v) const { return v; }
};
template <class Vertex>
struct swizzle_zxy
{
  inline Vertex forward(const Vertex& v)  const { return Vertex(v.z(), v.x(), v.y()); }
  inline Vertex backward(const Vertex& v) const { return Vertex(v.y(), v.z(), v.x()); }
};
template <class Vertex>
struct swizzle_yzx
{
  inline Vertex forward(const Vertex& v)  const { return Vertex(v.y(), v.z(), v.x()); }
  inline Vertex backward(const Vertex& v) const { return Vertex(v.z(), v.x(), v.y()); }
};

// Assumes a triangular mesh!
template <typename Swizzle, typename T, typename Mesh, typename Point>
void rasterizeTriangle(
  const typename Mesh::Face_index& fd,
  const std::unordered_map<typename Mesh::Vertex_index,Point> & points,
  const Mesh& mesh,
  GridMap<T>& gridmap,
  int fp_pow, int fp_scale)
{
  const Swizzle swizzler;
  std::vector<Point> tripts;
  for( const auto& vd : vertices_around_face(mesh.halfedge(fd), mesh) )
    tripts.push_back(swizzler.forward(points.at(vd)));
  
  // check if triangle is valid
  if( tripts[1].x() == tripts[0].x() && tripts[1].y() == tripts[0].y() ) return;
  if( tripts[2].x() == tripts[1].x() && tripts[2].y() == tripts[1].y() ) return;
  if( tripts[0].x() == tripts[2].x() && tripts[0].y() == tripts[2].y() ) return;
  if( (tripts[0].x() - tripts[2].x())*(tripts[1].y() - tripts[0].y())
     -(tripts[0].y() - tripts[2].y())*(tripts[1].x() - tripts[0].x()) == 0 ) return;
  
  int minX = std::min(std::min(tripts[0].x(),tripts[1].x()),tripts[2].x()) / fp_scale;
  int minY = std::min(std::min(tripts[0].y(),tripts[1].y()),tripts[2].y()) / fp_scale;
  int maxX = std::max(std::max(tripts[0].x(),tripts[1].x()),tripts[2].x()) / fp_scale;
  int maxY = std::max(std::max(tripts[0].y(),tripts[1].y()),tripts[2].y()) / fp_scale;  
  for (int j = minY; j <= maxY; j++) {
    for (int i = minX; i <= maxX; i++) {
      int depth;
      if (isInTriangle(
        (i << fp_pow) + (1 << (fp_pow - 1)), // centered
        (j << fp_pow) + (1 << (fp_pow - 1)), // centered
        tripts[0], tripts[1], tripts[2], depth))
      {
        Point vx = swizzler.backward(Point(i, j, depth >> fp_pow));
        // tag the voxel as occupied
        // NOTE: voxels are likely to be tagged multiple times (e.g. center exactly on edge, overlaps, etc.)
        gridmap.setMapValueCell({vx.x(),vx.y(),vx.z()},T(1));
      }
    }
  }
}

}





                             
template <typename T, typename Point>
inline CGAL::Surface_mesh<Point> toMeshMap(const GridMap<T>& gridmap)
{
  typedef typename CGAL::Surface_mesh<Point>::Vertex_index vertex_descriptor;
  CGAL::Surface_mesh<Point> mesh;
  
  if( gridmap.dim() != 3 ) return mesh;
  
  double dx = 0.5*gridmap.res()[0], dy = 0.5*gridmap.res()[1], dz = 0.5*gridmap.res()[2];
  for( int i = 0; i < gridmap.size()[0]; ++i)
    for( int j = 0; j < gridmap.size()[1]; ++j)
      for( int k = 0; k < gridmap.size()[2]; ++k)
        if( gridmap.map()[gridmap.subv2ind({i,j,k})] > T(0) )
        {
          // cells2meters
          double center_x = gridmap.min()[0] + i*gridmap.res()[0] + dx;
          double center_y = gridmap.min()[1] + j*gridmap.res()[1] + dy;
          double center_z = gridmap.min()[2] + k*gridmap.res()[2] + dz;
          
          //std::cout << center_x - dx << std::endl;
          //std::cout << center_x + dx << std::endl;
          //std::cout << center_y - dy << std::endl;
          //std::cout << center_y + dy << std::endl;
          //std::cout << center_z - dz << std::endl;
          //std::cout << center_z + dz << std::endl;
                              
          // Back
          Point face_1_1(center_x - dx, center_y + dy, center_z - dz);
          Point face_1_2(center_x - dx, center_y - dy, center_z - dz); 
          Point face_1_3(center_x - dx, center_y - dy, center_z + dz);
          Point face_1_4(center_x - dx, center_y + dy, center_z + dz);
          
          //triangles.push_back(Triangle(face_1_1, face_1_3, face_1_4));
          //triangles.push_back(Triangle(face_1_1, face_1_2, face_1_3));
          
          vertex_descriptor v_mpm = mesh.add_vertex(face_1_1);
          vertex_descriptor v_mmm = mesh.add_vertex(face_1_2);
          vertex_descriptor v_mmp = mesh.add_vertex(face_1_3);
          vertex_descriptor v_mpp = mesh.add_vertex(face_1_4);
          mesh.add_face(v_mpm, v_mmp, v_mpp);
          mesh.add_face(v_mpm, v_mmm, v_mmp);
          
          // Right
          //Point face_2_1(center_x - dx, center_y - dy, center_z - dz);
          Point face_2_2(center_x + dx, center_y - dy, center_z - dz);
          Point face_2_3(center_x + dx, center_y - dy, center_z + dz); 
          //Point face_2_4(center_x - dx, center_y - dy, center_z + dz);
          
          //triangles.push_back(Triangle(face_2_1, face_2_3, face_2_4));
          //triangles.push_back(Triangle(face_2_1, face_2_2, face_2_3));
          
          //vertex_descriptor v_mmm = mesh.add_vertex(face_2_1);
          vertex_descriptor v_pmm = mesh.add_vertex(face_2_2);
          vertex_descriptor v_pmp = mesh.add_vertex(face_2_3);
          //vertex_descriptor v_mmp = mesh.add_vertex(face_2_4);
          mesh.add_face(v_mmm, v_pmp, v_mmp);
          mesh.add_face(v_mmm, v_pmm, v_pmp);          
          
          // Front
          //Point face_3_1(center_x + dx, center_y - dy, center_z - dz);
          Point face_3_2(center_x + dx, center_y + dy, center_z - dz);
          Point face_3_3(center_x + dx, center_y + dy, center_z + dz); 
          //Point face_3_4(center_x + dx, center_y - dy, center_z + dz);
          
          //triangles.push_back(Triangle(face_3_1, face_3_3, face_3_4));
          //triangles.push_back(Triangle(face_3_1, face_3_2, face_3_3));

          //vertex_descriptor v_pmm = mesh.add_vertex(face_3_1);
          vertex_descriptor v_ppm = mesh.add_vertex(face_3_2);
          vertex_descriptor v_ppp = mesh.add_vertex(face_3_3);
          //vertex_descriptor v_pmp = mesh.add_vertex(face_3_4);
          mesh.add_face(v_pmm, v_ppp, v_pmp);
          mesh.add_face(v_pmm, v_ppm, v_ppp);
                    
          // Left
          //Point face_4_1(center_x + dx, center_y + dy, center_z - dz);
          //Point face_4_2(center_x - dx, center_y + dy, center_z - dz);
          //Point face_4_3(center_x - dx, center_y + dy, center_z + dz);
          //Point face_4_4(center_x + dx, center_y + dy, center_z + dz);
          
          //triangles.push_back(Triangle(face_4_1, face_4_3, face_4_4));
          //triangles.push_back(Triangle(face_4_1, face_4_2, face_4_3));

          //vertex_descriptor v_ppm = mesh.add_vertex(face_4_1);
          //vertex_descriptor v_mpm = mesh.add_vertex(face_4_2);
          //vertex_descriptor v_mpp = mesh.add_vertex(face_4_3);
          //vertex_descriptor v_ppp = mesh.add_vertex(face_4_4);
          mesh.add_face(v_ppm, v_mpp, v_ppp);
          mesh.add_face(v_ppm, v_mpm, v_mpp);

          // Top
          //triangles.push_back(Triangle(face_1_3, face_3_3, face_4_3));
          //triangles.push_back(Triangle(face_1_3, face_2_3, face_3_3));          
          mesh.add_face(v_mmp, v_ppp, v_mpp);
          mesh.add_face(v_mmp, v_pmp, v_ppp);

          // Bottom
          //triangles.push_back(Triangle(face_1_2, face_3_2, face_4_2));
          //triangles.push_back(Triangle(face_1_2, face_2_2, face_3_2));
          mesh.add_face(v_mmm, v_mpm, v_ppm);
          mesh.add_face(v_mmm, v_ppm, v_pmm);          
        }
  // CGAL::Surface_mesh_simplification::Count_ratio_stop_predicate<typename CGAL::Surface_mesh<Point>> stop(0.1);
  // CGAL::Surface_mesh_simplification::edge_collapse(mesh, stop);
  // https://doc.cgal.org/latest/Surface_mesh_simplification/index.html
  return mesh;   
}




template <typename T, typename Point>
inline GridMap<T> toGridMap( const CGAL::Surface_mesh<Point>& mesh,
                             const std::vector<double>& resolution,
                             int fp_pow /*= 16*/ )
{
  int fp_scale = (1<<fp_pow);
  
  // Initialize the gridmap
  
  const auto bbox = CGAL::Polygon_mesh_processing::bbox(mesh);
  std::vector<double> mapmin{bbox.xmin(),bbox.ymin(),bbox.zmin()};
  std::vector<double> mapmax{bbox.xmax(),bbox.ymax(),bbox.zmax()};
  GridMap<T> gridmap(mapmin,mapmax,resolution);
  gridmap.initZeroMap();
  
  // Produce (fixed fp) integer vertices and triangles
  typedef typename CGAL::Surface_mesh<Point>::Vertex_index vertex_descriptor;
  std::unordered_map<vertex_descriptor,CGAL::Simple_cartesian<int>::Point_3> points;
  points.reserve(mesh.number_of_vertices());
  std::vector<int> box_scale{gridmap.size()[0]*fp_scale, gridmap.size()[1]*fp_scale, gridmap.size()[2]*fp_scale};
  for( const auto& vd : mesh.vertices() )
  {
    const auto pt = mesh.point(vd);
    
    // Scale the vertex coordinates
    double pxd = std::round(box_scale[0]*((pt.x() - gridmap.origin()[0])/(gridmap.max()[0] - gridmap.min()[0]) + 0.5));
    double pyd = std::round(box_scale[1]*((pt.y() - gridmap.origin()[1])/(gridmap.max()[1] - gridmap.min()[1]) + 0.5));
    double pzd = std::round(box_scale[2]*((pt.z() - gridmap.origin()[2])/(gridmap.max()[2] - gridmap.min()[2]) + 0.5));
    
    int pxi = static_cast<int>(pxd < 0.0 ? 0.0 : box_scale[0]-1.0 < pxd ? box_scale[0]-1.0 : pxd);
    int pyi = static_cast<int>(pyd < 0.0 ? 0.0 : box_scale[1]-1.0 < pyd ? box_scale[1]-1.0 : pyd);
    int pzi = static_cast<int>(pzd < 0.0 ? 0.0 : box_scale[2]-1.0 < pzd ? box_scale[2]-1.0 : pzd);
    
    points.insert(std::make_pair(vd,CGAL::Simple_cartesian<int>::Point_3(pxi,pyi,pzi)));
  }
  
  for( const auto& fd : mesh.faces() )
  { 
    rasterizeTriangle<swizzle_xyz<CGAL::Simple_cartesian<int>::Point_3>,T,CGAL::Surface_mesh<Point>,
                        CGAL::Simple_cartesian<int>::Point_3>(fd, points, mesh, gridmap, fp_pow, fp_scale); // xy view
    rasterizeTriangle<swizzle_yzx<CGAL::Simple_cartesian<int>::Point_3>,T,CGAL::Surface_mesh<Point>,
                        CGAL::Simple_cartesian<int>::Point_3>(fd, points, mesh, gridmap, fp_pow, fp_scale); // yz view
    rasterizeTriangle<swizzle_zxy<CGAL::Simple_cartesian<int>::Point_3>,T,CGAL::Surface_mesh<Point>,
                        CGAL::Simple_cartesian<int>::Point_3>(fd, points, mesh, gridmap, fp_pow, fp_scale); // zx view    
  }
  return gridmap;
}


// TODO: update based on https://github.com/sylefeb/VoxSurf/blob/master/main.cpp
template <typename PT>
inline bool isInTriangle(int i, int j, const PT& p0, const PT& p1, const PT& p2, int& _depth)
{
  std::array<int64_t,2> delta_p0({i-p0.x(),j-p0.y()});
  std::array<int64_t,2> delta_p1({i-p1.x(),j-p1.y()});
  std::array<int64_t,2> delta_p2({i-p2.x(),j-p2.y()});
  std::array<int64_t,2> delta10({p1.x()-p0.x(),p1.y()-p0.y()});
  std::array<int64_t,2> delta21({p2.x()-p1.x(),p2.y()-p1.y()});
  std::array<int64_t,2> delta02({p0.x()-p2.x(),p0.y()-p2.y()});
  
  int64_t c0 = delta_p0[0] * delta10[1] - delta_p0[1] * delta10[0];
  int64_t c1 = delta_p1[0] * delta21[1] - delta_p1[1] * delta21[0];
  int64_t c2 = delta_p2[0] * delta02[1] - delta_p2[1] * delta02[0];
  bool inside = (c0 <= 0 && c1 <= 0 && c2 <= 0) || (c0 >= 0 && c1 >= 0 && c2 >= 0);

  if (inside) {
    int64_t area = c0 + c1 + c2;
    int64_t b0 = (c1 << 10) / area;
    int64_t b1 = (c2 << 10) / area;
    int64_t b2 = (1 << 10) - b0 - b1;
    _depth = static_cast<int>((b0 * p0.z() + b1 * p1.z() + b2 * p2.z()) >> 10);
  }
  return inside;
}

}

template <typename T>
inline erl::GridMap<int8_t> erl::toInt8GridMap( const erl::GridMap<T>& gridmap )
{
  erl::GridMap<int8_t> newmap(gridmap.size(), gridmap.origin(), gridmap.res(), gridmap.rowmajor());
  std::vector<int8_t> map(gridmap.map().size());
  for(unsigned k = 0; k < map.size(); ++k) {
    if (std::is_same<T, char>::value)
      map[k] = gridmap.map()[k] == '1' ? int8_t(1) : int8_t(0);
    else
      map[k] = static_cast<int8_t>(gridmap.map()[k]);    
  }
  newmap.setMap(map);
  return newmap;
}

#endif
