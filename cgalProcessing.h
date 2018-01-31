
#include <stdio.h>
#include <string>
#include <vector>
#include <fstream>
#include<algorithm>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/convex_hull_3.h>
#include <CGAL/Polyhedron_incremental_builder_3.h>
#include <CGAL/Polygon_mesh_processing/refine.h>
//#include <CGAL/Polygon_mesh_processing/fair.h>
#include <CGAL/IO/OBJ_reader.h>
#include <CGAL/Surface_mesh.h>

#include<CGAL/IO/Polyhedron_iostream.h>
#include <iostream> 
#include <map>


class CGALProcessing {


    public:
    #ifdef amanDev
    std::string plyFolder = "/home/aman/Desktop/FYP-Processing/models/PLY";
    std::string objFolder = "/home/aman/Desktop/FYP-Processing/models/OBJ";
    std::string fname = "/home/aman/Desktop/FYP-Processing/335.obj";
    #else
    std::string plyFolder = "/Users/waleedzafar/projects/fyp/one/models/PLY/";
    std::string objFolder = "/Users/waleedzafar/projects/fyp/one/models/OBJ/";
    std::string fname = "/Users/waleedzafar/projects/fyp/one/models/DotNet.ply";
    #endif
    void testOBJ();
    void shapeDetection();
    typedef CGAL::Exact_predicates_inexact_constructions_kernel  Kernel;
    typedef CGAL::Polyhedron_3<Kernel>                     Polyhedron_3;
    typedef Kernel::Point_3                                Point_3;
    // typedef CGAL::Surface_mesh<Point_3>               Surface_mesh;
    typedef Polyhedron_3::Facet_iterator                   Facet_iterator;
    typedef Polyhedron_3::Halfedge_around_facet_circulator Halfedge_facet_circulator;
    typedef Polyhedron_3::HalfedgeDS             HalfedgeDS;
    typedef std::vector<Point_3> PointVector;


    private:
    void inputTest(std::string, PointVector &, std::vector<std::vector<std::size_t> > &);
    void outputWriter(std::string, Polyhedron_3 &);
    void incrementBuilder(Polyhedron_3 &, PointVector &, std::vector<std::vector<std::size_t> > &);
};
