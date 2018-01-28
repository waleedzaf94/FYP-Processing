#include "cgalProcessing.h"
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polyhedron_3.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel  Kernel;
typedef CGAL::Polyhedron_3<Kernel>                     Polyhedron_3;
typedef Kernel::Point_3                                Point_3;
// typedef CGAL::Surface_mesh<Point_3>               Surface_mesh;
typedef Polyhedron_3::Facet_iterator                   Facet_iterator;
typedef Polyhedron_3::Halfedge_around_facet_circulator Halfedge_facet_circulator;
typedef Polyhedron_3::HalfedgeDS             HalfedgeDS;
typedef std::vector<Point_3> PointVector;

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
