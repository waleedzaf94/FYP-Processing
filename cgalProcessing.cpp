#include "cgalProcessing.h"
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polyhedron_3.h>

// Point set shape detection imports
//#include <CGAL/IO/read_xyz_points.h>
#include <CGAL/IO/read_ply_points.h>
#include <CGAL/Point_with_normal_3.h>
#include <CGAL/property_map.h>
#include <CGAL/Shape_detection_3.h>



typedef CGAL::Exact_predicates_inexact_constructions_kernel  Kernel;
typedef CGAL::Polyhedron_3<Kernel>                     Polyhedron_3;
typedef Kernel::Point_3                                Point_3;
// typedef CGAL::Surface_mesh<Point_3>               Surface_mesh;
typedef Polyhedron_3::Facet_iterator                   Facet_iterator;
typedef Polyhedron_3::Halfedge_around_facet_circulator Halfedge_facet_circulator;
typedef Polyhedron_3::HalfedgeDS             HalfedgeDS;
typedef std::vector<Point_3> PointVector;

typedef std::pair<Kernel::Point_3, Kernel::Vector_3> Point_with_normal;
typedef std::vector<Point_with_normal> Pwn_vector;
typedef CGAL::First_of_pair_property_map<Point_with_normal> Point_map;
typedef CGAL::Second_of_pair_property_map<Point_with_normal> Normal_map;
typedef CGAL::Shape_detection_3::Efficient_RANSAC_traits<Kernel, Pwn_vector, Point_map, Normal_map> Traits;
typedef CGAL::Shape_detection_3::Efficient_RANSAC<Traits> Efficient_ransac;
typedef CGAL::Shape_detection_3::Plane<Traits> cgalPlane;
typedef CGAL::Shape_detection_3::Cone<Traits> cgalCone;
typedef CGAL::Shape_detection_3::Cylinder<Traits> cgalCylinder;
typedef CGAL::Shape_detection_3::Sphere<Traits> cgalSphere;
typedef CGAL::Shape_detection_3::Torus<Traits> cgalTorus;

// A modifier creating a triangle with the incremental builder.
template<class HDS>
class polyhedron_builder : public CGAL::Modifier_base<HDS> {
public:
      PointVector &coords;
      std::vector<std::vector<std::size_t> >   &tris;
      polyhedron_builder( PointVector &_coords, std::vector<std::vector<std::size_t> > &_tris ) : coords(_coords), tris(_tris) {}
      void operator()( HDS& hds) {
            typedef typename HDS::Vertex   Vertex;
            typedef typename Vertex::Point_3 Point;
            
            // create a cgal incremental builder
                  CGAL::Polyhedron_incremental_builder_3<HDS> B( hds, true);
                  B.begin_surface( coords.size(), tris.size());
            
            // add the polyhedron vertices
            for( Point_3 i : coords ){
                  B.add_vertex(i);
            }
      
      // add the polyhedron triangles
            for( std::vector<size_t> i : tris){
                  B.begin_facet();
                  B.add_vertex_to_facet( i[0] );
                  B.add_vertex_to_facet( i[1] );
                  B.add_vertex_to_facet( i[2] );
                  B.end_facet();
            }
      
      // finish up the surface
            B.end_surface();
      }
};

inline
void CGALProcessing::testOBJ()
{
    std::string filename = "/Users/waleedzafar/projects/fyp/one/models/DotNet.obj";
      PointVector points;
      std::vector<std::vector<std::size_t>> faces;
      Polyhedron_3 poly;
      this->inputTest(filename, points, faces);
//     // compute convex hull of non-collinear points
      CGAL::convex_hull_3(points.begin(), points.end(), poly);
      std::cout << "The convex hull contains " << poly.size_of_vertices() << " vertices" << std::endl;
      if (!CGAL::is_triangle_mesh(poly)){
            std::cerr << "Input geometry is not triangulated." << std::endl;
            return;
      }
      std::string output = "dump.off";
      // This crashes due to too many faces
      // this->incrementBuilder(poly, points, faces);

      this->outputWriter(output, poly);
//     Surface_mesh sm;
//     CGAL::convex_hull_3(points.begin(), points.end(), sm);
//     std::cout << "The convex hull contains " << num_vertices(sm) << " vertices" << std::endl;
 }

inline 
void CGALProcessing::incrementBuilder(Polyhedron_3 &P, PointVector &points, std::vector<std::vector<std::size_t> > &faces)
{
    polyhedron_builder<HalfedgeDS> builder(points, faces);
    P.delegate(builder);
}


inline 
void CGALProcessing::outputWriter(std::string filename, Polyhedron_3 &poly)
{
//     write the polyhedron out as a .OFF file
    std::ofstream os(filename);
    os << poly;
    os.close();
     // Write polyhedron in Object File Format (OFF).
//     CGAL::set_ascii_mode( std::cout);
//     std::cout << "OFF" << std::endl << poly.size_of_vertices() << ' '
//               << poly.size_of_facets() << " 0" << std::endl;
//     std::copy( poly.points_begin(), poly.points_end(),
//                std::ostream_iterator<Point_3>( std::cout, "\n"));
//     for (  Facet_iterator i = poly.facets_begin(); i != poly.facets_end(); ++i) {
//         Halfedge_facet_circulator j = i->facet_begin();
//         // Facets in polyhedral surfaces are at least triangles.
//         CGAL_assertion( CGAL::circulator_size(j) >= 3);
//         std::cout << CGAL::circulator_size(j) << ' ';
//         do {
//             std::cout << ' ' << std::distance(poly.vertices_begin(), j->vertex());
//         } while ( ++j != i->facet_begin());
//         std::cout << std::endl;
//     }
}

inline
void CGALProcessing::inputTest(std::string filename, PointVector &points, std::vector<std::vector<std::size_t> > &faces)
{
      std::ifstream input(filename);
      CGAL::read_OBJ(input, points, faces);
      std::cout << points.size() << " " << faces.size() << "\n";
}

inline
void CGALProcessing::shapeDetection(){
    // points with normals
    Pwn_vector points;
    
    // loads point set from a file
    // read_xyz_points_and_normals takes an OutputIterator for storing the points
    // and a property_map to store the normal vector with each point
    std::ifstream stream(this->fname);
    
//    if (!stream || !CGAL::read_xyz_points_and_normals(stream, std::back_inserter(points), Point_map(), Normal_map())) {
//        std::cerr << "Error. Cannot read file." << std::endl;
//    }
    if (!stream || !CGAL::read_ply_points_and_normals(stream, std::back_inserter(points), Point_map(), Normal_map())) {
        std::cerr << "Error. Cannot read file." << std::endl;
    }
    
    std::cout << points.size() << " points originally." << std::endl;
    
    // Instantiate shape detection engine
    Efficient_ransac ransac;
    ransac.set_input(points);
    ransac.add_shape_factory<cgalPlane>();
//    ransac.add_shape_factory<cgalSphere>();
//    ransac.add_shape_factory<cgalCylinder>();
//    ransac.add_shape_factory<cgalCone>();
//    ransac.add_shape_factory<cgalTorus>();
    ransac.detect();
    
    std::cout << ransac.shapes().end() - ransac.shapes().begin() << " shapes detected." << std::endl;
    std::cout << ransac.number_of_unassigned_points() << " unassigned points." << std::endl;
    
    Efficient_ransac::Shape_range shapes = ransac.shapes();
    Efficient_ransac::Shape_range::iterator it = shapes.begin();
    
    while (it != shapes.end()) {
        if (cgalPlane* plane = dynamic_cast<cgalPlane*>(it->get())) {
            Kernel::Vector_3 normal = plane->plane_normal();
            std::cout << "Plane with normal " << normal << std::endl;
            std::cout << "Kernel::Plane_3: " << static_cast<Kernel::Plane_3>(*plane) << std::endl;
            std::cout << "Info: " << plane->info() << std::endl;
        }
        else {
            std::cout << (*it)->info() << std::endl;
        }
        it++;
    }
    
}







