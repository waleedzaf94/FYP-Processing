
#include "cgalProcessing.h"

// A modifier creating a triangle with the incremental builder.
template<class HDS>
class polyhedron_builder : public CGAL::Modifier_base<HDS> {
public:
    CGALProcessing::PointVector &coords;
      std::vector<std::vector<std::size_t> >   &tris;
    polyhedron_builder( CGALProcessing::PointVector &_coords, std::vector<std::vector<std::size_t> > &_tris ) : coords(_coords), tris(_tris) {}
      void operator()( HDS& hds) {
            typedef typename HDS::Vertex   Vertex;
            typedef typename Vertex::Point_3 Point;
            
            // create a cgal incremental builder
                  CGAL::Polyhedron_incremental_builder_3<HDS> B( hds, true);
                  B.begin_surface( coords.size(), tris.size());
            
            // add the polyhedron vertices
            for( CGALProcessing::Point_3 i : coords ){
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
    CGAL::Timer timer;
    
    // loads point set from a file
    // read_xyz_points_and_normals takes an OutputIterator for storing the points
    // and a property_map to store the normal vector with each point
    std::ifstream stream(this->fname);
    
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
    
    timer.start();
    ransac.preprocess();
    timer.stop();
    std::cout << "Preprocessing time: " << timer.time() * 1000 << " ms\n";
    
    Efficient_ransac::Shape_range shapes = ransac.shapes();
    FT bestCoverage = 0;
    
    for (size_t i = 0; i<3; i++) {
        timer.reset();
        timer.start();
        
        ransac.detect();
        
        timer.stop();
        
        FT coverage = FT(points.size() - ransac.number_of_unassigned_points()) / FT(points.size());
        
        std::cout << "time: " << timer.time() * 1000 << " ms\n";
        std::cout << ransac.shapes().end() - ransac.shapes().begin() << " primitives, " << coverage << " coverage\n";
        
        if (coverage > bestCoverage) {
            bestCoverage = coverage;
            shapes = ransac.shapes();
        }
    }
    
    this->writeShapesToFiles(shapes);
    
    Efficient_ransac::Shape_range::iterator it = shapes.begin();
    
    while (it != shapes.end()) {
        boost::shared_ptr<Efficient_ransac::Shape> shape = *it;
        std::cout << (*it)->info() << std::endl;
        
        FT sumDistances = 0;
        std::vector<std::size_t>::const_iterator indexIt = (*it)->indices_of_assigned_points().begin();
        
        while (indexIt != (*it)->indices_of_assigned_points().end()) {
            const Point_with_normal &p = *(points.begin() + (*indexIt));
            sumDistances += CGAL::sqrt((*it)->squared_distance(p.first));
            indexIt++;
        }
        
        FT averageDistance = sumDistances / shape->indices_of_assigned_points().size();
        std::cout << " average distance: " << averageDistance << std::endl;
        it++;
    }
    
//    ransac.detect();
    
//    std::cout << ransac.shapes().end() - ransac.shapes().begin() << " shapes detected." << std::endl;
//    std::cout << ransac.number_of_unassigned_points() << " unassigned points." << std::endl;
//
//    Efficient_ransac::Shape_range shapes = ransac.shapes();
//    Efficient_ransac::Shape_range::iterator it = shapes.begin();
//
//    while (it != shapes.end()) {
//        if (cgalPlane* plane = dynamic_cast<cgalPlane*>(it->get())) {
//            Kernel::Vector_3 normal = plane->plane_normal();
//            std::cout << "Plane with normal " << normal << std::endl;
//            std::cout << "Kernel::Plane_3: " << static_cast<Kernel::Plane_3>(*plane) << std::endl;
//            std::cout << "Info: " << plane->info() << std::endl;
//        }
//        else {
//            std::cout << (*it)->info() << std::endl;
//        }
//        it++;
//    }
    
}

inline
void CGALProcessing::writeShapesToFiles(Efficient_ransac::Shape_range shapes) {
    Efficient_ransac::Shape_range::iterator it = shapes.begin();
    int shapeNum=0;
    while (it != shapes.end()) {
//        Pwn_vector points = it->get()->;
        std::vector<size_t> indices = it->get()->indices_of_assigned_points();
        long numPoints = indices.size();
        std::vector<size_t>::const_iterator iti = indices.begin();
        while (iti != indices.end()) {
//            Point_with_normal pw = *(points.begin() + (*iti));
//            Kernel::Point_3 p = pw.first;
        }
    }
}

inline
void CGALProcessing::writeShapeToFile(Efficient_ransac::Shape_range::iterator it, std::string filepath) {
    
}
























