//rosrun erl_map test_mesh_map src/erl_development/erl_map/test/data/fla_warehouse1.off

//#include "erl_map/mesh_map.h"
#include <fstream> // std::ifstream
#include <unordered_map>

// CGAL
#include <CGAL/config.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Triangle_3.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Surface_mesh.h>

// AABB Tree
#include <CGAL/AABB_triangle_primitive.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_face_graph_triangle_primitive.h>

// Polygon_mesh_processing
#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <CGAL/Polygon_mesh_processing/orientation.h>
#include <CGAL/Polygon_mesh_processing/polygon_soup_to_polygon_mesh.h>
#include <CGAL/Polygon_mesh_processing/bbox.h>
#include <CGAL/Polygon_mesh_processing/refine.h>
#include <CGAL/Polygon_mesh_processing/fair.h>
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>
#include <CGAL/Polygon_mesh_processing/connected_components.h>


// Future CGAL Versions
// #include <CGAL/Polygon_mesh_processing/repair_polygon_soup.h>
// #include <CGAL/IO/STL_reader.h>
// #include <CGAL/draw_surface_mesh.h>

// Basic typedefs
typedef CGAL::Simple_cartesian<double> Kernel;
typedef Kernel::FT FT;
typedef Kernel::Point_3 Point;
typedef Kernel::Vector_3 Vector;
typedef Kernel::Ray_3 Ray;
typedef Kernel::Segment_3 Segment;

// Polyhedron
typedef CGAL::Polyhedron_3<Kernel> Polyhedron;
typedef Polyhedron::Vertex                                   Vertex;
typedef Polyhedron::Vertex_iterator                          Vertex_iterator;
typedef Polyhedron::Halfedge_handle                          Halfedge_handle;
typedef Polyhedron::Edge_iterator                            Edge_iterator;
typedef Polyhedron::Facet_iterator                           Facet_iterator;
typedef Polyhedron::Halfedge_around_vertex_const_circulator  HV_circulator;
typedef Polyhedron::Halfedge_around_facet_circulator         HF_circulator;

// Surface Mesh
typedef CGAL::Surface_mesh<Point> Mesh;
typedef Mesh::Vertex_index vertex_descriptor;
typedef Mesh::Face_index face_descriptor;
//typedef boost::graph_traits<Mesh>::face_descriptor face_descriptor;
//typedef boost::graph_traits<Mesh>::halfedge_descriptor halfedge_descriptor;

// AABB Tree
typedef CGAL::Triangle_3<Kernel> Triangle;
typedef CGAL::AABB_triangle_primitive<Kernel, std::list<Triangle>::iterator> Primitive;
typedef CGAL::AABB_traits<Kernel, Primitive> PrimitiveTraits;
typedef CGAL::AABB_tree<PrimitiveTraits> PrimitiveTree;
typedef boost::optional<PrimitiveTree::Intersection_and_primitive_id<Segment>::Type > SegmentPrimitiveIntersection;
typedef boost::optional<PrimitiveTree::Intersection_and_primitive_id<Ray>::Type> RayPrimitiveIntersection;

typedef CGAL::AABB_face_graph_triangle_primitive<Mesh> FaceGraphPrimitive;
typedef CGAL::AABB_traits<Kernel, FaceGraphPrimitive> FaceGraphPrimitiveTraits;
typedef CGAL::AABB_tree<FaceGraphPrimitiveTraits> FaceGraphPrimitiveTree;
typedef boost::optional<FaceGraphPrimitiveTree::Intersection_and_primitive_id<Segment>::Type > SegmentFaceGraphPrimitiveIntersection;
typedef boost::optional<FaceGraphPrimitiveTree::Intersection_and_primitive_id<Ray>::Type> RayFaceGraphPrimitiveIntersection;




void display_cgal_version()
{
  std::cout << "My CGAL library is " <<  CGAL_VERSION_NR << " (1MMmmb1000)" << std::endl; 
  std::cout << std::endl;
  std::cout << "where MM is the major number release, mm is the minor number release, and "
            << "b is the bug fixing number release." << std::endl;
}

bool readTriangularMesh(const std::string& filename, Mesh& mesh)
{
  mesh.clear();
  std::ifstream input(filename);
  if (!input || !(input >> mesh) || mesh.is_empty())
    return false;
  
  // Check if there are nontriangular faces and triangulate if necessary
  for( const auto& fd : mesh.faces() )
    if (next(next(halfedge(fd, mesh), mesh), mesh) != prev(halfedge(fd,mesh),mesh))
    {
      CGAL::Polygon_mesh_processing::triangulate_faces(mesh);
      break;
    }
  
  // Confirm that all faces are triangles.
  for( const auto& fd : mesh.faces() )
    if (next(next(halfedge(fd, mesh), mesh), mesh) != prev(halfedge(fd,mesh),mesh))
      return false;
  
  return true;
}

bool readPolyhedron(const std::string& filename, Polyhedron& poly)
{
  poly.clear();
  std::ifstream input(filename);
  Polyhedron P;
  if ( !input || !(input >> poly) || poly.empty() || !CGAL::is_triangle_mesh(poly))
    return false;
  return true;
}


int test_mesh(const std::string& filename)
{
  Mesh M;
  if( !readTriangularMesh(filename,M) )
  {
    std::cerr << "Error reading a triangular mesh." << std::endl;
    return EXIT_FAILURE;
  }

  std::cout << M.number_of_vertices() << "  "  << M.number_of_faces() << std::endl;
  
  double d = CGAL::Polygon_mesh_processing::is_outward_oriented(M)?-1:1;
  std::cout << "outward oriented: " << d << std::endl;
  
  auto bb = CGAL::Polygon_mesh_processing::bbox(M);
  std::cout << "bbox = " << bb << std::endl;
  
  
  std::unordered_map<vertex_descriptor,Point> pts;
  for( auto fd : M.faces() )
  {
    for( auto vd : vertices_around_face(M.halfedge(fd), M) )
    {
      //std::cout << vd << std::endl;
      pts.insert(std::make_pair(vd,M.point(vd)));
    }
  }
  std::cout << "num pts = " << pts.size() << std::endl;
  
  return EXIT_SUCCESS;
}


int test_poly(const std::string& filename)
{
  Polyhedron P;
  if( !readPolyhedron(filename,P) )
  {
    std::cerr << "Error reading a triangular mesh." << std::endl;
    return EXIT_FAILURE;
  }
  
  std::cout << P.size_of_vertices() << "  "  << P.size_of_facets() << std::endl;

  double d = CGAL::Polygon_mesh_processing::is_outward_oriented(P)?-1:1;
  std::cout << "outward oriented: " << d << std::endl;
  
  auto bb = CGAL::Polygon_mesh_processing::bbox(P);
  std::cout << "bbox = " << bb << std::endl;
  
  return EXIT_SUCCESS;
}


int main(int argc, char** argv)
{
  display_cgal_version();
  const std::string filename(argv[1]);
  
  int fp_scale = (1<<16);
  std::cout << fp_scale << std::endl;
  
  return test_mesh(filename);
} 


  
  /*
  
  erl::MeshMap mm;
  mm.loadOFF(argv[1]);
  std::cout << mm.mesh().number_of_vertices() << "  "  << mm.mesh().number_of_faces() << std::endl;
  
  for (const auto& fd : mm.mesh().faces())
    std::cout << fd << std::endl;
  
  double d = CGAL::Polygon_mesh_processing::is_outward_oriented(mm.mesh())?-1:1;
  auto bb = CGAL::Polygon_mesh_processing::bbox(mm.mesh());
  std::cout << bb << std::endl;
  
  //CGAL::draw(mm.mesh());
  // https://stackoverflow.com/questions/46808246/cgal-get-face-data-from-surface-mesh
  // CGAL::Polygon_mesh_processing::bbox()
  */


