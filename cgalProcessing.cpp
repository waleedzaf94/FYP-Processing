
#include <stdio.h>
#include <string>
#include <vector>
#include <fstream>

#include "cgalProcessing.h"


#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/convex_hull_3.h>
#include <CGAL/Polyhedron_incremental_builder_3.h>
#include <CGAL/Polygon_mesh_processing/refine.h>
#include <CGAL/Polygon_mesh_processing/fair.h>
#include <CGAL/IO/OBJ_reader.h>
// #include <CGAL/Surface_mesh.h>

#include <iostream> 
#include <map>


typedef CGAL::Exact_predicates_inexact_constructions_kernel  K;
typedef CGAL::Polyhedron_3<K>                     Polyhedron_3;
typedef K::Point_3                                Point_3;
// typedef CGAL::Surface_mesh<Point_3>               Surface_mesh;
typedef std::vector<Point_3> PointVector;

typedef Polyhedron_3::Facet_iterator                   Facet_iterator;
typedef Polyhedron_3::Halfedge_around_facet_circulator Halfedge_facet_circulator;


inline
void CGALProcessing::testImportOBJ()
{
    const char* filename = "/home/aman/Desktop/FYP-Processing/models/DotNet.obj";
    std::ifstream input(filename);
    PointVector points;
    PointVector outputVector;
    std::vector<std::vector<std::size_t> > faces;
    CGAL::read_OBJ(input, points, faces);
    // std::cout << points.size() << " " << faces.size() << "\n";
    // for(Point_3 i : points) 
    //     std::cout << i << " \n";
    // for (std::vector<std::size_t> i : faces)
    // {
    //     for ( int x : i ) 
    //         std::cout << x << ' ';
    //     std::cout << std::endl;
    // }
    Polyhedron_3 poly;
    
    // compute convex hull of non-collinear points
    CGAL::convex_hull_3(points.begin(), points.end(), poly);
    std::cout << "The convex hull contains " << poly.size_of_vertices() << " vertices" << std::endl;
    // Surface_mesh sm;
        
    if (!CGAL::is_triangle_mesh(poly)){
        std::cerr << "Input geometry is not triangulated." << std::endl;
        return;
    }
    else
        std::cout << "Triangular mesh \n";

     // Write polyhedron in Object File Format (OFF).
    CGAL::set_ascii_mode( std::cout);
    std::cout << "OFF" << std::endl << poly.size_of_vertices() << ' '
              << poly.size_of_facets() << " 0" << std::endl;
    std::copy( poly.points_begin(), poly.points_end(),
               std::ostream_iterator<Point_3>( std::cout, "\n"));
    for (  Facet_iterator i = poly.facets_begin(); i != poly.facets_end(); ++i) {
        Halfedge_facet_circulator j = i->facet_begin();
        // Facets in polyhedral surfaces are at least triangles.
        CGAL_assertion( CGAL::circulator_size(j) >= 3);
        std::cout << CGAL::circulator_size(j) << ' ';
        do {
            std::cout << ' ' << std::distance(poly.vertices_begin(), j->vertex());
        } while ( ++j != i->facet_begin());
        std::cout << std::endl;
    }
    // CGAL::convex_hull_3(points.begin(), points.end(), sm);
    // std::cout << "The convex hull contains " << num_vertices(sm) << " vertices" << std::endl;
 }
