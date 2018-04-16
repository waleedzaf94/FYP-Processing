#ifndef DEFINITIONS_HPP
#define DEFINITIONS_HPP

#ifdef BOOST_PARAMETER_MAX_ARITY
#undef BOOST_PARAMETER_MAX_ARITY
#endif
#define BOOST_PARAMETER_MAX_ARITY 12
//#define CGAL_NDEBUG

#include <stdio.h>
#include <string>
#include <vector>
#include <iostream>
#include <map>
#include <fstream>
#include <algorithm>
#include <thread>
#include <chrono>
#include <sstream>
#include <fstream>
#include <cstring>
#include <math.h>
#include <cmath>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/convex_hull_3.h>
#include <CGAL/Polyhedron_incremental_builder_3.h>
#include <CGAL/Polygon_mesh_processing/refine.h>
#include <CGAL/IO/OBJ_reader.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/IO/Polyhedron_iostream.h>
#include <CGAL/poisson_surface_reconstruction.h>
#include <CGAL/Polyhedron_items_3.h>
#include <CGAL/HalfedgeDS_list.h>

// Point set shape detection imports
#include <CGAL/IO/read_ply_points.h>
#include <CGAL/Point_with_normal_3.h>
#include <CGAL/property_map.h>
#include <CGAL/Shape_detection_3.h>
#include <CGAL/Timer.h>

// Advancing front surface reconstruction
#include <CGAL/structure_point_set.h>
#include <CGAL/Delaunay_triangulation_3.h>
#include <CGAL/Triangulation_vertex_base_with_info_3.h>
#include <CGAL/Advancing_front_surface_reconstruction.h>

// Surface mesh generation
#include <CGAL/Mesh_triangulation_3.h>
#include <CGAL/Mesh_complex_3_in_triangulation_3.h>
#include <CGAL/Mesh_criteria_3.h>
#include <CGAL/Polyhedral_mesh_domain_with_features_3.h>
#include <CGAL/make_mesh_3.h>
#include <CGAL/bounding_box.h>
#include <CGAL/Bbox_3.h>

// Poisson surface reconstruction
#include <CGAL/trace.h>
#include <CGAL/Surface_mesh_default_triangulation_3.h>
#include <CGAL/make_surface_mesh.h>
#include <CGAL/Implicit_surface_3.h>
#include <CGAL/IO/output_surface_facets_to_polyhedron.h>
#include <CGAL/Poisson_reconstruction_function.h>
#include <CGAL/compute_average_spacing.h>
#include <CGAL/Polygon_mesh_processing/distance.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::FT                                          FT;
typedef CGAL::Polyhedron_3<Kernel>                          Polyhedron_3;
typedef Kernel::Point_3                                     Point_3;
typedef Kernel::Vector_3                                    Vector_3;
typedef Polyhedron_3::Facet_iterator                        Facet_iterator;
typedef Polyhedron_3::Vertex_iterator                       Vertex_iterator;
typedef Polyhedron_3::Halfedge_around_facet_circulator      Halfedge_facet_circulator;
typedef Polyhedron_3::HalfedgeDS                            HalfedgeDS;
typedef std::vector<Point_3>                                PointVector;
typedef std::vector<std::size_t>                            FacetIndices;
typedef std::vector<FacetIndices>                           FacetVector;

// Shape Detection 3 type definitions
typedef std::pair<Point_3, Vector_3>                            Point_with_normal;
typedef std::vector<Point_with_normal>                          Pwn_vector;
typedef CGAL::First_of_pair_property_map<Point_with_normal>     Point_map;
typedef CGAL::Second_of_pair_property_map<Point_with_normal>    Normal_map;
typedef CGAL::Shape_detection_3::Efficient_RANSAC_traits<Kernel, Pwn_vector, Point_map, Normal_map> Traits;
typedef CGAL::Shape_detection_3::Efficient_RANSAC<Traits>       Efficient_ransac;
typedef CGAL::Shape_detection_3::Plane<Traits>                  cgalPlane;
typedef CGAL::Shape_detection_3::Cone<Traits>                   cgalCone;
typedef CGAL::Shape_detection_3::Cylinder<Traits>               cgalCylinder;
typedef CGAL::Shape_detection_3::Sphere<Traits>                 cgalSphere;
typedef CGAL::Shape_detection_3::Torus<Traits>                  cgalTorus;

// Point set structuring types
typedef CGAL::Point_set_with_structure<Traits>                  Structure;

// Advancing front types
typedef CGAL::Advancing_front_surface_reconstruction_vertex_base_3<Kernel>  LVb;
typedef CGAL::Advancing_front_surface_reconstruction_cell_base_3<Kernel>    LCb;
typedef CGAL::Triangulation_data_structure_3<LVb, LCb>                      Tds;
typedef CGAL::Delaunay_triangulation_3<Kernel, Tds>             Triangulation_3;
typedef Triangulation_3::Vertex_handle                          Vertex_handle;
typedef CGAL::cpp11::array<std::size_t, 3>                      Facet;

// Surface mesh generation types
typedef CGAL::Mesh_polyhedron_3<Kernel>::type                   Mesh_polyhedron;
typedef CGAL::Polyhedral_mesh_domain_with_features_3<Kernel>    Mesh_domain;
typedef CGAL::HalfedgeDS_default<CGAL::Epick, CGAL::I_Polyhedron_derived_items_3<CGAL::Mesh_3::Mesh_polyhedron_items<int> >, std::allocator<int> > Mesh_hds;
    
#ifdef CGAL_CONCURRENT_MESH_3
    typedef CGAL::Parallel_tag      Concurrency_tag;
#else
    typedef CGAL::Sequential_tag    Concurrency_tag;
#endif
    
// Triangulation
typedef CGAL::Mesh_triangulation_3<Mesh_domain, CGAL::Default, Concurrency_tag>::type Tr;
typedef CGAL::Mesh_complex_3_in_triangulation_3<Tr, Mesh_domain::Corner_index, Mesh_domain::Curve_segment_index> C3t3;
// Criteria
typedef CGAL::Mesh_criteria_3<Tr> Mesh_criteria;

typedef std::vector<Polyhedron_3>   Polyhedron_vector;
typedef std::vector<Pwn_vector>     Pwn_vector_vector;

// Poisson surface reconstruction
typedef Kernel::Sphere_3                                                    Sphere_3;
typedef CGAL::Poisson_reconstruction_function<Kernel>                       Poisson_reconstruction_function;
typedef CGAL::Surface_mesh_default_triangulation_3                          STr;
typedef CGAL::Surface_mesh_complex_2_in_triangulation_3<STr>                C2t3;
typedef CGAL::Implicit_surface_3<Kernel, Poisson_reconstruction_function>   Surface_3;

#endif // DEFINITIONS_HPP


// // Custom Builders
// #include "ModelBuilder.hpp"
// #include "PolyhedronBuilder.hpp"
