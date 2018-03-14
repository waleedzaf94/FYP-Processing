
#ifndef CGAL_PROCESSING_H
#define CGAL_PROCESSING_H

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

#include "processingIO.hpp"

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/convex_hull_3.h>
#include <CGAL/Polyhedron_incremental_builder_3.h>
#include <CGAL/Polygon_mesh_processing/refine.h>
#include <CGAL/IO/OBJ_reader.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/IO/Polyhedron_iostream.h>
#include <CGAL/poisson_surface_reconstruction.h>

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
//#include <CGAL/Mesh_triangulation_3.h>
//#include <CGAL/Mesh_complex_3_in_triangulation_3.h>
//#include <CGAL/Mesh_criteria_3.h>
//#include <CGAL/Polyhedral_mesh_domain_with_features_3.h>
//#include <CGAL/make_mesh_3.h>

class CGALProcessing {

    public:
    #ifdef amanDev
    std::string plyFolder = "/home/aman/Desktop/FYP-Processing/models/PLY";
    std::string objFolder = "/home/aman/Desktop/FYP-Processing/models/OBJ";
    std::string fname = "/home/aman/Desktop/FYP-Processing/335.obj";
    #else
    std::string plyFolder = "/Users/waleedzafar/projects/fyp/one/models/PLY/";
    std::string objFolder = "/Users/waleedzafar/projects/fyp/one/models/OBJ/";
    std::string fname = "/Users/waleedzafar/projects/fyp/one/models/Chi_11.ply";
    #endif
    
    struct float3 {
        float x, y, z;
    };
    struct PlyFile {
        std::vector<float3> vertices;
        std::vector<float3> normals;
        std::vector<std::vector<int> > faces;
    };
    
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
//    typedef CGAL::Mesh_polyhedron_3<Kernel>::type                   Mesh_polyhedron;
//    typedef CGAL::Polyhedral_mesh_domain_with_features_3<Kernel>    Mesh_domain;
    
    // Functor to init the advancing front algorithm with indexed points
    struct On_the_fly_pair {
        const Pwn_vector &points;
        typedef std::pair<Point_3, std::size_t> result_type;
        
        On_the_fly_pair (const Pwn_vector & points) : points(points) {}
        
        result_type
        operator()(std::size_t i) const {
            return result_type(points[i].first, i);
        }
    };
    
    // Specialized priority functor that favors structure coherence
    template <typename Structure>
    struct Priority_with_structure_coherence {
        
        Structure &structure;
        double bound;
        
        Priority_with_structure_coherence (Structure &structure, double bound) : structure(structure), bound(bound) {}
        
        template <typename AdvancingFront, typename Cell_handle>
        double operator() (AdvancingFront &adv, Cell_handle &c, const int index) const {
            // if perimeter > bound, return infinity so that the facet is not used
            if (bound != 0) {
                double d = 0;
                d = sqrt(squared_distance(c->vertex((index+1)%4)->point(),
                                          c->vertex((index+2)%4)->point()));
                if (d>bound)
                    return adv.infinity();
                d += sqrt(squared_distance(c->vertex((index+2)%4)->point(),
                                           c->vertex((index+3)%4)->point()));
                if (d>bound)
                    return adv.infinity();
                d += sqrt(squared_distance(c->vertex((index+1)%4)->point(),
                                           c->vertex((index+3)%4)->point()));
                if (d>bound)
                    return adv.infinity();
            }
            
            Facet f = {{
                c->vertex ((index + 1) % 4)->info (),
                c->vertex ((index + 2) % 4)->info (),
                c->vertex ((index + 3) % 4)->info ()
            }};
            // facet_coherence takes values between -1 and 3, 3 being the most
            // coherent and -1 being incoherent. Smaller weight means higher
            // priority.
            double weight = 100. * (5 - structure.facet_coherence (f));
            return weight * adv.smallest_radius_delaunay_sphere (c, index);
        }
    };
    
    typedef CGAL::Advancing_front_surface_reconstruction<Triangulation_3, Priority_with_structure_coherence<Structure> > Reconstruction;
    
    // Public functions
    void testOBJ();
    void shapeDetection();
    bool writePlyPointsAndNormals(std::vector<Point_with_normal>, std::string);
    void readModelInfo(modelInfo, PointVector &, std::vector<std::vector<std::size_t> > &);
    void inputTest(std::string);
    void polyhedronProcessing(Polyhedron_3 &);
    void polyhedronProcessing(std::string);
    void polyhedronToModelInfo(Polyhedron_3 &, modelInfo &);
    void poissonSurfaceReconstruction(Polyhedron_3 &);
    void advancingFrontSurfaceReconstruction(Pwn_vector &, Polyhedron_3 &);
    void polyhedronToPwnVector(Polyhedron_3 &, Pwn_vector &);
    void readPlyToPwn(std::string, Pwn_vector &);
    void printPolyhedronInfo(Polyhedron_3 &);
    void surfaceMeshGeneration(Polyhedron_3 &, Polyhedron_3 &);
    void pwnToPointVector(Pwn_vector &, PointVector &);
    void facetVectorToStd(std::vector<Facet> &, std::vector<std::vector<std::size_t> > &);
    
    void outputWriter(std::string, Polyhedron_3 &);
    void incrementBuilder(Polyhedron_3 &, PointVector &, std::vector<std::vector<std::size_t> > &);
    void incrementBuilder(Polyhedron_3 &, Pwn_vector &, std::vector<Facet> &);
    void writeShapesToFiles(CGAL::Shape_detection_3::Efficient_RANSAC<Traits>::Shape_range, std::vector<Point_with_normal>);
    
};

// A modifier creating a triangle with the incremental builder.
template<class HDS>
class polyhedron_builder : public CGAL::Modifier_base<HDS> {
public:
    CGALProcessing::PointVector             &coords;
    std::vector<std::vector<std::size_t> >  &faces;
    
    polyhedron_builder( CGALProcessing::PointVector &_coords, std::vector<std::vector<std::size_t> > &_tris ) : coords(_coords), faces(_tris) {
        
    }
    
    void operator()(HDS & hds) {
        
        // create a cgal incremental builder
        CGAL::Polyhedron_incremental_builder_3<HDS> B( hds, true);
        B.begin_surface( coords.size(), faces.size());
        
        // add the polyhedron vertices
        for( CGALProcessing::Point_3 i : coords ){
            B.add_vertex(i);
        }
        // add the polyhedron triangles
        for (std::vector<size_t> i: faces) {
            try {
            if (B.test_facet(i.begin(), i.end())) {
                B.begin_facet();
                for (size_t j: i) {
                    B.add_vertex_to_facet(j);
                }
                B.end_facet();
            }
            } catch (std::exception e) {
                std::cerr << e.what() << std::endl;
            }
        }
        
        // finish up the surface
        B.end_surface();
    }
};

#endif //CGAL_PROCESSING_H


