#ifndef __MESH_MAP_H_
#define __MESH_MAP_H_

#include <CGAL/Simple_cartesian.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/AABB_face_graph_triangle_primitive.h>
//#include <CGAL/Triangle_3.h>
//#include <CGAL/AABB_triangle_primitive.h>
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>

#include <memory>
#include <fstream> // std::ifstream

namespace erl {

class MeshMap {
 public:
  typedef CGAL::Simple_cartesian<double> K;
  typedef K::Point_3 Point;
  typedef K::Segment_3 Segment;
  typedef K::Vector_3 Vector;
  typedef K::Ray_3 Ray;

  typedef CGAL::Surface_mesh<Point> Mesh;
  typedef CGAL::Triangle_3<K> Triangle;
  //typedef std::list<Triangle>::iterator Iterator;
  //typedef CGAL::AABB_triangle_primitive<K, Iterator> Primitive;
  typedef CGAL::AABB_face_graph_triangle_primitive<Mesh> Primitive;
  typedef CGAL::AABB_traits<K, Primitive> Traits;
  typedef CGAL::AABB_tree<Traits> Tree;
  typedef boost::optional<Tree::Intersection_and_primitive_id<Ray>::Type> RayPrimitiveIntersection;
  typedef boost::optional<Tree::Intersection_and_primitive_id<Segment>::Type> SegmentPrimitiveIntersection;

  MeshMap() : mesh_ptr_(new Mesh), tree_ptr_(new Tree) {} // default constructor
  MeshMap(std::shared_ptr<Mesh> m_ptr) : mesh_ptr_(std::move(m_ptr)), tree_ptr_(new Tree) { rebuildTree(); }

  /**
   * Adds a list of vertices and faces to the mesh
   * @param vertices The list of vertices to add
   * @param faces_begin An array of begin iterators for the face indices 
   * @param faces_end An array of end iterators for the face indices
   */
  template<typename Iter>
  void addFaces(const std::vector<Point> &vertices,
                const std::vector<Iter> &faces_begin,
                const std::vector<Iter> &faces_end);

  bool loadOFF(const std::string &filename);
  void saveOFF(const std::string &filename) const;
  bool loadSTL(const std::string &filename);
  void saveSTL(const std::string &filename) const;

  Mesh const &mesh() const { return *mesh_ptr_; }
  Tree const &tree() const { return *tree_ptr_; }

  /**
   * Check if point y is visible from sensor pose (p,R) and is within the field of view
   * specified by range_bracket, azimuth_bracket, and elevation_bracket
   *
   * @param y query point
   * @param R sensor orientation (Forward-Left-UP frame; not optical frame)
   * @param p sensor position
   * @param range_bracket sensor [min_range, max_range]
   * @param azimuth_bracket sensor [min_azimuth, max_azimuth]
   * @param elevation_bracket sensor [min_elevation, max_elevation]
   */
  bool isPointVisibleFromPose(const Eigen::Vector3d &y,
                              const Eigen::Matrix3d &R,
                              const Eigen::Vector3d &p,
                              const Eigen::Vector2d &range_bracket,
                              const Eigen::Vector2d &azimuth_bracket,
                              const Eigen::Vector2d &elevation_bracket);

 private:
  std::shared_ptr<Mesh> mesh_ptr_; // This is a pointer because CGAL Surface Mesh does not implement a move constructor
  std::shared_ptr<Tree> tree_ptr_;
  void rebuildTree() {
    //tree_ptr_->rebuild(triangles.begin(), triangles.end());
    tree_ptr_->rebuild(faces(*mesh_ptr_).first, faces(*mesh_ptr_).second, *mesh_ptr_);
    //tree_ptr_->build();
    tree_ptr_->accelerate_distance_queries();
  }
};

inline bool triangulateMesh(MeshMap::Mesh &mesh);

inline bool intersectRayAABBTree(const MeshMap::Ray &ray_query,
                                 const MeshMap::Tree &tree,
                                 MeshMap::Point &intersection_point);
inline bool read_STL(std::istream &input,
                          std::vector<std::array<double, 3> > &points,
                          std::vector<std::array<int, 3> > &facets,
                          bool verbose);

} // erl




//////////////////////////////////////////////////////////////////////////////////
// MeshMap Implementation
//////////////////////////////////////////////////////////////////////////////////

template<typename Iter>
inline void erl::MeshMap::addFaces(const std::vector<MeshMap::Point> &vertices,
                                   const std::vector<Iter> &faces_begin,
                                   const std::vector<Iter> &faces_end) {
  std::vector<erl::MeshMap::Mesh::Vertex_index> vi(vertices.size());
  for (unsigned k = 0; k < vertices.size(); ++k)
    vi[k] = mesh_ptr_->add_vertex(vertices[k]);

  // https://stackoverflow.com/questions/670734/pointer-to-class-data-member
  size_t num_faces = std::min(faces_begin.size(), faces_end.size());
  for (unsigned k = 0; k < num_faces; ++k) {
    std::vector<erl::MeshMap::Mesh::Vertex_index> vr; // vertex indices of face k
    for (auto it = faces_begin[k]; it != faces_end[k]; ++it)
      vr.push_back(vi[*it]);
    mesh_ptr_->add_face(vr);
  }
  rebuildTree();
  //  for(unsigned k = 0; k < faces.size(); ++k)
  //  {
  //    switch(faces[k].size())
  //    {
  //      case 3 : mesh_.add_face( vi[faces[k][0]], vi[faces[k][1]], vi[faces[k][2]] );
  //               break; 
  //      case 4 : mesh_.add_face( vi[faces[k][0]], vi[faces[k][1]], vi[faces[k][2]], vi[faces[k][3]] );
  //               break;
  //      default: std::vector<MeshMap::Mesh::Vertex_index> vr(faces[k].size());
  //               for(unsigned j=0; j<vr.size(); ++j)
  //                 vr[j] = vi[faces[k][j]];
  //               mesh_.add_face(vr);
  //    }
  //  }
}

inline bool erl::MeshMap::loadOFF(const std::string &filename) {
  std::ifstream input(filename);
  if (input) {
    mesh_ptr_->clear();
    if (!(input >> *mesh_ptr_) || mesh_ptr_->is_empty()) {
      //std::cerr << "Error reading the mesh." << std::endl;
      return false;
    }


    //if(!erl::triangulateMesh(*mesh_ptr_)) {
    //std::cerr << "Error triangulating the mesh." << std::endl;
    //return false;
    //}

    if (mesh_ptr_->number_of_faces() > 0)
      rebuildTree();
    return true;
  }
  //std::cerr << "Error opening input file." << std::endl;
  return false;
}

inline void erl::MeshMap::saveOFF(const std::string &filename) const {
  std::ofstream ofs(filename);
  ofs << *mesh_ptr_;
  ofs.close();
}

inline bool erl::MeshMap::loadSTL(const std::string &filename) {
  std::ifstream input(filename);
  if (input) {
    std::vector<std::array<double, 3>> vertices;
    std::vector<std::array<int, 3>> triangles;
    read_STL(input, vertices, triangles, true);
    std::cout << vertices.size() << "  " << triangles.size() << std::endl;

    std::vector<erl::MeshMap::Mesh::Vertex_index> vi(vertices.size());
    for (unsigned k = 0; k < vertices.size(); ++k) {
      erl::MeshMap::Point p(vertices[k][0], vertices[k][1], vertices[k][2]);
      vi[k] = mesh_ptr_->add_vertex(p);
    }

    // https://stackoverflow.com/questions/670734/pointer-to-class-data-member
    size_t num_faces = triangles.size();
    for (unsigned k = 0; k < num_faces; ++k) {
      std::vector<erl::MeshMap::Mesh::Vertex_index> vr; // vertex indices of face k
      for (auto it = triangles[k].begin(); it != triangles[k].end(); ++it)
        vr.push_back(vi[*it]);
      mesh_ptr_->add_face(vr);
    }
    rebuildTree();

  }
  return false;
}

//
//void erl::MeshMap::saveSTL(const std::string& filename) const
//{
//  std::ofstream ofs(filename);
//  CGAL::write_STL(*mesh_ptr_, ofs);
//  ofs.close();
//}



inline bool
erl::MeshMap::isPointVisibleFromPose(const Eigen::Vector3d &y,
                                     const Eigen::Matrix3d &R,
                                     const Eigen::Vector3d &p,
                                     const Eigen::Vector2d &range_bracket,
                                     const Eigen::Vector2d &azimuth_bracket,
                                     const Eigen::Vector2d &elevation_bracket) {
  Eigen::Vector3d y_sensor_frame = R.transpose() * (y - p); // world to sensor frame

  // Check the azimuth constraints
  double azimuth = std::atan2(y_sensor_frame.y(), y_sensor_frame.x());
  if (azimuth <= azimuth_bracket(0) || azimuth >= azimuth_bracket(1))
    return false;

  // Check the elevation constraints
  double elevation = std::atan2(y_sensor_frame.z(), std::hypot(y_sensor_frame.x(), y_sensor_frame.y()));
  if (elevation <= elevation_bracket(0) || elevation >= elevation_bracket(1))
    return false;

  // Check the distance constraints
  double dx = y.x() - p.x();
  double dy = y.y() - p.y();
  double dz = y.z() - p.z();
  double dist2 = dx * dx + dy * dy + dz * dz;
  double dist = std::sqrt(dist2);
  if (dist <= range_bracket(0) || dist >= range_bracket(1))
    return false;

  // Check if y is actually visible
  erl::MeshMap::Point pt(p.x(), p.y(), p.z());
  erl::MeshMap::Vector v(dx, dy, dz);
  erl::MeshMap::Ray ray_query(pt, v);
  erl::MeshMap::Point intersection_point;
  bool occluded = intersectRayAABBTree(ray_query, *tree_ptr_, intersection_point);
  if (occluded) {
    // if y is occluded see if the distance to the intersection point is in fact smaller than the distance to y
    dx = intersection_point.x() - p.x();
    dy = intersection_point.y() - p.y();
    dz = intersection_point.z() - p.z();
    if (dx * dx + dy * dy + dz * dz < dist2)
      return false;
  }
  return true;
}

inline bool erl::triangulateMesh(erl::MeshMap::Mesh &mesh) {
  // Check if there are nontriangular faces and triangulate if necessary
  for (const auto &fd : mesh.faces())
    if (next(next(halfedge(fd, mesh), mesh), mesh) != prev(halfedge(fd, mesh), mesh)) {
      CGAL::Polygon_mesh_processing::triangulate_faces(mesh);
      break;
    }

  // Confirm that all faces are triangles
  for (const auto &fd : mesh.faces())
    if (next(next(halfedge(fd, mesh), mesh), mesh) != prev(halfedge(fd, mesh), mesh))
      return false;
  return true;
}

inline bool erl::intersectRayAABBTree(const erl::MeshMap::Ray &ray_query,
                                      const erl::MeshMap::Tree &tree,
                                      erl::MeshMap::Point &intersection_point) {
  erl::MeshMap::RayPrimitiveIntersection intersection = tree.first_intersection(ray_query);
  if (intersection) {
    const erl::MeshMap::Point *pp = boost::get<erl::MeshMap::Point>(&(intersection->first));
    if (pp) {
      intersection_point = *pp;
      return true;
    }
  }
  return false;
}

inline bool erl::read_STL(std::istream &input,
                     std::vector<std::array<double, 3> > &points,
                     std::vector<std::array<int, 3> > &facets,
                     bool verbose = false) {
  std::string s, solid("solid");
  std::map<std::array<double, 3>, int> pmap;
  int index = 0;
  std::array<int, 3> ijk;
  std::array<double, 3> p;

  char line[80];
  for (int i = 0; i < 80; i++) {
    boost::uint8_t c;
    input.read(reinterpret_cast<char *>(&c), sizeof(c));
    line[i] = c;
    if (i == 5) {
      s = std::string(line, 5);
      if (s == solid) {
        break;
      }
    }
  }

  if (s != solid) {
    boost::uint32_t N32;
    input.read(reinterpret_cast<char *>(&N32), sizeof(N32));
    unsigned int N = N32;

    for (unsigned int i = 0; i < N; i++) {
      float normal[3];
      input.read(reinterpret_cast<char *>(&normal[0]), sizeof(normal[0]));
      input.read(reinterpret_cast<char *>(&normal[1]), sizeof(normal[1]));
      input.read(reinterpret_cast<char *>(&normal[2]), sizeof(normal[2]));

      for (int j = 0; j < 3; j++) {
        float x, y, z;
        input.read(reinterpret_cast<char *>(&x), sizeof(x));
        input.read(reinterpret_cast<char *>(&y), sizeof(y));
        input.read(reinterpret_cast<char *>(&z), sizeof(z));
        p[0] = x;
        p[1] = y;
        p[2] = z;
        std::map<std::array<double, 3>, int>::iterator iti =
            pmap.insert(std::make_pair(p, -1)).first;
        if (iti->second == -1) {
          ijk[j] = index;
          iti->second = index++;
          points.push_back(p);
        } else {
          ijk[j] = iti->second;
        }
      }
      if ((ijk[0] != ijk[1]) &&
          (ijk[0] != ijk[2]) &&
          (ijk[1] != ijk[2])) {
        facets.push_back(ijk);
      } else {
        if (verbose) {
          std::cerr << "ignore degenerate face" << std::endl;
        }
      }
      char c;
      input.read(reinterpret_cast<char *>(&c), sizeof(c));
      input.read(reinterpret_cast<char *>(&c), sizeof(c));
    }
    return true;
  } else {
    std::string facet("facet"),
        outer("outer"),
        loop("loop"),
        vertex("vertex"),
        endloop("endloop"),
        endsolid("endsolid");

    while (input >> s) {
      if (s == endsolid) {
        //std::cerr << "found endsolid" << std::endl;
      } else if (s == facet) {
        //std::cerr << "found facet" << std::endl;
        std::getline(input, s); // ignore the normal
        input >> s;
        if (s != outer) {
          if (verbose)
            std::cerr << "Expect 'outer' and got " << s << std::endl;
          return false;
        }
        input >> s;
        if (s != loop) {
          if (verbose)
            std::cerr << "Expect 'loop' and got " << s << std::endl;
          return false;
        }
        int count = 0;
        do {
          input >> s;
          if (s == vertex) {
            //      std::cerr << "found vertex" << std::endl;
            if (count < 3) {
              input >> p[0] >> p[1] >> p[2];
              std::map<std::array<double, 3>, int>::iterator iti =
                  pmap.insert(std::make_pair(p, -1)).first;
              if (iti->second == -1) {
                ijk[count] = index;
                iti->second = index++;
                points.push_back(p);
              } else {
                ijk[count] = iti->second;
              }
              ++count;
            } else {
              if (verbose)
                std::cerr << "We can only read triangulated surfaces" << std::endl;
              return false;
            }
          }
        } while (s != endloop);

        facets.push_back(ijk);
      }
    }
    return true;
  }
}

#endif

