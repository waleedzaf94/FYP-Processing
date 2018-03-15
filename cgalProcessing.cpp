
#include "cgalProcessing.h"

inline 
void CGALProcessing::incrementBuilder(Polyhedron_3 &P, PointVector &points, std::vector<std::vector<std::size_t> > &faces) {
    polyhedron_builder<HalfedgeDS> builder(points, faces);
    P.delegate(builder);
}

inline
void CGALProcessing::incrementBuilder(Mesh_polyhedron &P, PointVector &points, std::vector<std::vector<std::size_t> > &faces) {
    mesh_polyhedron_builder<Mesh_hds> builder(points, faces);
    P.delegate(builder);
}

inline
void CGALProcessing::incrementBuilder(Polyhedron_3 &P, Pwn_vector &points, std::vector<Facet> &faces) {
    PointVector p;
    std::vector<std::vector<std::size_t> > f;
    this->pwnToPointVector(points, p);
    this->facetVectorToStd(faces, f);
    polyhedron_builder<HalfedgeDS> builder(p, f);
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
//    modelInfo mm;
//    this->polyhedronToModelInfo(p, mm);
//    std::string outName = this->plyFolder + "custom.obj";
//    writeObjFile(outName, mm);
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

void CGALProcessing::printPolyhedronInfo(Polyhedron_3 & P) {
    std::cout << "Polyhedron Vertices: " << P.size_of_vertices() << " Faces: " << P.size_of_facets() << std::endl;
}

void CGALProcessing::polyhedronProcessing(Polyhedron_3 & P) {
    Pwn_vector points;
    polyhedronToPwnVector(P, points);
    cout << "READ\n";
}

void CGALProcessing::polyhedronProcessing(std::string filename) {
    Polyhedron_3 poly;
    Pwn_vector points;
    this->readPlyToPwn(filename, points);
    this->advancingFrontSurfaceReconstruction(points, poly);
    this->printPolyhedronInfo(poly);
    Polyhedron_3 out;
    this->surfaceMeshGeneration(poly, out);
    this->printPolyhedronInfo(out);
}

void CGALProcessing::surfaceMeshGeneration(Polyhedron_3 & input, Polyhedron_3 & output) {
    
    Mesh_polyhedron polyhedron;
    modelInfo model;
    this->polyhedronToModelInfo(input, model);
    PointVector points;
    std::vector<std::vector<std::size_t> > faces;
    this->readModelInfo(model, points, faces);
    incrementBuilder(polyhedron, points, faces);
    
    if (!CGAL::is_triangle_mesh(polyhedron)){
        std::cerr << "Input geometry is not triangulated." << std::endl;
        return;
    }
    // Create domain
    Mesh_domain domain(polyhedron);
    
    // Get sharp features
    domain.detect_features();
    // Mesh criteria
    Mesh_criteria criteria(CGAL::parameters::edge_size = 0.025,
                           CGAL::parameters::facet_angle = 25, CGAL::parameters::facet_size = 0.05, CGAL::parameters::facet_distance = 0.005,
                           CGAL::parameters::cell_radius_edge_ratio = 3, CGAL::parameters::cell_size = 0.05);
    
    // Mesh generation
    C3t3 c3t3 = CGAL::make_mesh_3<C3t3>(domain, criteria);
    // Output
    std::ofstream medit_file("out.mesh");
    c3t3.output_to_medit(medit_file);
    
}

inline
void CGALProcessing::pwnToPointVector(Pwn_vector & input, PointVector & output) {
    output.clear();
    for (Point_with_normal pwn: input) {
        Point_3 point = pwn.first;
        output.push_back(point);
    }
}

inline
void CGALProcessing::facetVectorToStd(std::vector<Facet> & input, std::vector<std::vector<std::size_t> > & output) {
    output.clear();
    for (Facet facet: input) {
        std::vector<std::size_t> indices;
        indices.push_back(facet[0]);
        indices.push_back(facet[1]);
        indices.push_back(facet[2]);
        output.push_back(indices);
    }
}

void CGALProcessing::advancingFrontSurfaceReconstruction(Pwn_vector & points, Polyhedron_3 & poly) {
    
    
    std::cerr << "Shape detection...\n";
    Efficient_ransac ransac;
    ransac.set_input(points);
    ransac.add_shape_factory<cgalPlane>(); // Only planes are useful for stucturing
    // Default RANSAC parameters
    Efficient_ransac::Parameters op;
    op.probability = 0.05;
    op.min_points = 100;
    op.epsilon = 0.005;
    op.cluster_epsilon = 0.05;
    op.normal_threshold = 0.8;
    ransac.detect(op); // Plane detection
    std::cerr << "done\nPoint set structuring...\n";
    Pwn_vector structured_pts;
    Structure pss (points.begin (), points.end (), ransac,
                   op.cluster_epsilon);  // Same parameter as RANSAC
    for (std::size_t i = 0; i < pss.size(); ++ i)
        structured_pts.push_back (pss[i]);
    std::cerr << "done\nAdvancing front...\n";
    std::vector<std::size_t> point_indices(boost::counting_iterator<std::size_t>(0),
                                           boost::counting_iterator<std::size_t>(structured_pts.size()));
    Triangulation_3 dt (boost::make_transform_iterator(point_indices.begin(), On_the_fly_pair(structured_pts)),
                        boost::make_transform_iterator(point_indices.end(), On_the_fly_pair(structured_pts)));
    Priority_with_structure_coherence<Structure> priority (pss,
                                                           1000. * op.cluster_epsilon); // Avoid too large facets
    Reconstruction R(dt, priority);
    R.run ();
    std::cerr << "done\nWriting result... ";
    std::vector<Facet> output;
    const Reconstruction::TDS_2& tds = R.triangulation_data_structure_2();
    for(Reconstruction::TDS_2::Face_iterator fit = tds.faces_begin(); fit != tds.faces_end(); ++fit)
        if(fit->is_on_surface())
            output.push_back (CGAL::make_array(fit->vertex(0)->vertex_3()->id(),
                                               fit->vertex(1)->vertex_3()->id(),
                                               fit->vertex(2)->vertex_3()->id()));
    
    this->incrementBuilder(poly, structured_pts, output);
    
    std::string outname = this->plyFolder + "advancing3.off";
    std::ofstream f (outname.c_str());
    f << "OFF\n" << structured_pts.size () << " " << output.size() << " 0\n"; // Header
    for (std::size_t i = 0; i < structured_pts.size (); ++ i)
        f << structured_pts[i].first << std::endl;
    for (std::size_t i = 0; i < output.size (); ++ i)
        f << "3 "
        << output[i][0] << " "
        << output[i][1] << " "
        << output[i][2] << std::endl;
    std::cerr << "all done\n" << std::endl;
    f.close();
}

inline
void CGALProcessing::polyhedronToPwnVector(Polyhedron_3 & P, Pwn_vector & points) {
    // Still need to extract normals from Polyhedron
    for (Vertex_iterator it=P.vertices_begin(); it != P.vertices_end(); it++) {
        Point_3 point  = it->point();
        Vector_3 vec;
        Point_with_normal pwn(point, vec);
        points.push_back(pwn);
    }
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

inline
void CGALProcessing::readPlyToPwn(std::string fname, Pwn_vector & points) {
    points.clear();
    
    // loads point set from a file
    // read_ply_points_and_normals takes an OutputIterator for storing the points
    // and a property_map to store the normal vector with each point
    std::ifstream stream(fname);
    
    if (!stream || !CGAL::read_ply_points_and_normals(stream, std::back_inserter(points), Point_map(), Normal_map())) {
        std::cerr << "Error. Cannot read file." << std::endl;
    }
    
    std::cout << points.size() << " points originally." << std::endl;
}

void CGALProcessing::shapeDetection(){
    // points with normals
    Pwn_vector points;
    CGAL::Timer timer;
    
    this->readPlyToPwn(this->fname, points);
    
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






















