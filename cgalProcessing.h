
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

// Point set shape detection imports
#include <CGAL/IO/read_ply_points.h>
#include <CGAL/Point_with_normal_3.h>
#include <CGAL/property_map.h>
#include <CGAL/Shape_detection_3.h>
#include <CGAL/Timer.h>

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
    typedef Polyhedron_3::Facet_iterator                        Facet_iterator;
    typedef Polyhedron_3::Halfedge_around_facet_circulator      Halfedge_facet_circulator;
    typedef Polyhedron_3::HalfedgeDS                            HalfedgeDS;
    typedef std::vector<Point_3>                                PointVector;
    typedef Polyhedron_3::Halfedge_around_facet_const_circulator    HFCC;
    typedef HalfedgeDS::Face_handle                             Facet_handle;
    
    //Shape Detection 3 type definitions
    typedef std::pair<Kernel::Point_3, Kernel::Vector_3>            Point_with_normal;
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
    
    // Public functions
    void testOBJ();
    void shapeDetection();
    bool writePlyPointsAndNormals(std::vector<Point_with_normal>, std::string);
    void readModelInfo(modelInfo, PointVector &, std::vector<std::vector<std::size_t> > &);
    void inputTest(std::string, PointVector &, std::vector<std::vector<std::size_t> > &);
    void inputTest(std::string);
    
    private:
    
    void outputWriter(std::string, Polyhedron_3 &);
    void incrementBuilder(Polyhedron_3 &, PointVector &, std::vector<std::vector<std::size_t> > &);
    void writeShapesToFiles(CGAL::Shape_detection_3::Efficient_RANSAC<Traits>::Shape_range, std::vector<Point_with_normal>);
    
};

#endif //CGAL_PROCESSING_H


