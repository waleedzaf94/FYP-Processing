
#include "cgalProcessing.h"

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
            if (B.test_facet(i.begin(), i.end())) {
                B.begin_facet();
                for (size_t j: i) {
                    B.add_vertex_to_facet(j);
                }
                B.end_facet();
            }
        }
  
        // finish up the surface
        B.end_surface();
      }
};

inline 
void CGALProcessing::incrementBuilder(Polyhedron_3 &P, PointVector &points, std::vector<std::vector<std::size_t> > &faces)
{
    polyhedron_builder<HalfedgeDS> builder(points, faces);
    P.delegate(builder);
}

void CGALProcessing::inputTest(std::string filename) {
//    modelInfo model = readPlyFile(filename);
    modelInfo model = readObjFile(filename);
    PointVector points;
    std::vector<std::vector<std::size_t> > faces;
    readModelInfo(model, points, faces);
    Polyhedron_3 p;
    incrementBuilder(p, points, faces);
    std::cout << "Polyhedron: Vertices: " << p.size_of_vertices() << " Faces: " << p.size_of_facets() << std::endl;
    this->polyhedronProcessing(p);
    modelInfo mm;
    this->polyhedronToModelInfo(p, mm);
    std::string outName = this->plyFolder + "custom.obj";
    writeObjFile(outName, mm);
//    std::ofstream out(outName);
//    write_off(out, p);
//    std::cout << "Wrote from polyhedron: " << outName << std::endl;
}

void CGALProcessing::polyhedronToModelInfo(Polyhedron_3 & P, modelInfo & model) {
    for (Vertex_iterator it=P.vertices_begin(); it != P.vertices_end(); it++) {
        Point_3 point = it->point();
        vertexInfo v;
        v.x = point.cartesian(0);
        v.y = point.cartesian(1);
        v.z = point.cartesian(2);
        model.vertices.push_back(v);
    }
    for (Facet_iterator it=P.facets_begin(); it != P.facets_end(); it++) {
        Halfedge_facet_circulator j = it->facet_begin();
        if (CGAL::circulator_size(j) >= 3) {
            faceInfo face;
            do {
                face.vertexIndices.push_back(std::distance(P.vertices_begin(), j->vertex()));
            } while ( ++j != it->facet_begin());
            model.faces.push_back(face);
        }
        else {
            std::cerr << "Invalid facet circulator size" << std::endl;
        }
    }
}

void CGALProcessing::inputTest(std::string filename, PointVector &points, std::vector<std::vector<std::size_t> > &faces)
{
      std::ifstream input(filename);
      CGAL::read_OBJ(input, points, faces);
      std::cout << points.size() << " " << faces.size() << "\n";
}

void CGALProcessing::polyhedronProcessing(Polyhedron_3 & P) {
    std::cout << "Polyhedron Processing" << std::endl;
}

inline
void CGALProcessing::readModelInfo(modelInfo model, PointVector & points, std::vector<std::vector<std::size_t> > &faces) {
    for (vertexInfo v: model.vertices) {
        points.push_back(Point_3(static_cast<double>(v.x), static_cast<double>(v.y), static_cast<double>(v.z)));
    }
    for (faceInfo f: model.faces) {
        std::vector<std::size_t> indices;
        for (int i: f.vertexIndices) {
            indices.push_back(i);
        }
        faces.push_back(indices);
    }
}

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
    
    this->writeShapesToFiles(shapes, points);
    
    return;
    
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
void CGALProcessing::writeShapesToFiles(Efficient_ransac::Shape_range shapes, Pwn_vector points) {
    Efficient_ransac::Shape_range::iterator it = shapes.begin();
    int shapeNum=0;
    while (it != shapes.end()) {
        std::vector<size_t> indices = it->get()->indices_of_assigned_points();
        std::vector<size_t>::const_iterator iti = indices.begin();
        Pwn_vector newPoints(1);
        while (iti != indices.end()) {
            Point_with_normal pw = *(points.begin() + (*iti));
//            Kernel::Point_3 p = pw.first;
//            Kernel::Vector_3 v = pw.second;
//            CGAL::Epick::Direction_3 d = v.direction();
//            printf("Point: %f, %f, %f \n", p.x(), p.y(), p.z());
//            printf("Vector: %f, %f, %f, %d \n", v.x(), v.y(), v.z(), v.dimension());
//            printf("Direction: %f, %f, %f \n", d.dx(), d.dy(), d.dz());
            newPoints.push_back(pw);
            iti++;
        }
        std::cout << "Shape with " << newPoints.size() << " points\n";
        std::string outFile = this->plyFolder + "/CGAL/" + std::to_string(shapeNum) + ".ply";
        if (this->writePlyPointsAndNormals(newPoints, outFile)) {
            std::cerr << "Wrote file: " << outFile << std::endl;
        }
        else {
            std::cerr << "Error writing file: " << outFile << std::endl;
        }
        shapeNum++;
        it++;
    }
}

inline
bool CGALProcessing::writePlyPointsAndNormals (Pwn_vector points, std::string fname) {
    std::ofstream out(fname);
    if (!out) {
        std::cerr << "Error writing file: " << fname << std::endl;
        return false;
    }
    // PLY Header
    out << "ply\n";
    out << "format ascii 1.0\n";
    out << "element vertex " << std::distance(points.begin(), points.end()) << std::endl;
    out << "property double x\n";
    out << "property double y\n";
    out << "property double z\n";
    out << "property double nx\n";
    out << "property double ny\n";
    out << "property double nz\n";
    out << "end_header\n";
    
    //Vertices
//    std::vector<Point_with_normal>::iterator it = points.begin();
//    while (
    for (auto it = points.begin(); it != points.end(); it++) {
        Kernel::Point_3 point = it->first;
        Kernel::Vector_3 normal = it->second;
        out << point.x() << " " << point.y() << " " << point.z() << " " << normal.x() << " " << normal.y() << " " << normal.z() << std::endl;
    }
    
    return true;
}






















