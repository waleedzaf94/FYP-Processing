
#include "cgalProcessing.h"

/// Public functions

// Control

void CGALProcessing::inputPolyhedron(std::string filePath, std::string filetype) {
    //    modelInfo model = readPlyFile(filename);
    inputFileType = filetype;
    printf("Reading in %s file\n", filetype.c_str());
    this->modelbuilder.readFile(filePath, filetype);
    this->modelbuilder.toPolyhedron(this->polyhedron3);
    this->modelbuilder.toPwnVector(this->pwn_points);
    printf("Read file");
    return;
    
    
    if (filetype=="obj")
    {
        std::cout << "Reading in OBJ File " << std::endl;
        modelbuilder.readFile(filePath, filetype);  // Throwing STOI exception
        PointVector points;
        FacetVector faces;
        // ALL OK UNTIL HERE
        std::cout << "To Points and Faces...";
        modelbuilder.toPointAndFacetVectors(points, faces);
        std::cout << "Read File... Starting Increment Builder";
        this->incrementBuilder(this->polyhedron3, points, faces);
        printInfo(polyhedron3);
        // std::cout << "Polyhedron3: Vertices: " << this->polyhedron3.size_of_vertices() << " Faces: " << this->polyhedron3.size_of_facets() << std::endl;
        return;
    }
    if (filetype == "off"){
        std::ifstream fin(filePath);
        // modelbuilder.readFile(filePath, filetype); // Throwing STOI exception
//        CGAL::read_off(fin, this->meshPolyhedron); // This works. Is it required?
        CGAL::read_off(fin, this->polyhedron3);
        std::cout << "Read file: " << filePath << std::endl;
        printInfo(this->polyhedron3);
        return;
    }
    if (filetype == "ply")
    {
        modelbuilder.readFile(filePath, filetype);
        modelbuilder.toPolyhedron(polyhedron3);
        printInfo(polyhedron3);
        return;
    }
}
 

void CGALProcessing::outputPolyhedron(std::string filePath, std::string filetype)
{
    std::cout << "Saving to " << filePath << std::endl;
    if (filetype == "ply")
    {
        ModelBuilder::modelInfo model;
        printInfo(polyhedron3);
        toModelInfo(polyhedron3, model);
        modelbuilder.printModelInfo(model);
        modelbuilder.setOutputModel(model);
        modelbuilder.writeFile(filePath, filetype); 
    }
    else if (filetype == "off")
    {
        ModelBuilder::modelInfo model;
        toModelInfo(polyhedron3, model);
        modelbuilder.setOutputModel(model);
        modelbuilder.writeFile(filePath, filetype);  
    }
    else    // Case of off and ply
    {
        modelbuilder.writeFile(filePath, filetype); 
    }
} 

// The wrappers should initialize an instance of the Polyhedron3 -> Get Info -> ProcessImpl -> Set Polyhedron3 
void CGALProcessing::advancingFrontWrapper()
{
    if (inputFileType == "ply")
        advancingFrontSurfaceReconstruction(modelbuilder.points);
    return;
}
// This is reading the original PWD_Vectors as Advancing Front Does Not Set Points
// Maybe we need a Polyhedron_3->Pwd_Method 
void CGALProcessing::shapeDetectionWrapper()
{
    if (inputFileType == "ply")
        pointSetShapeDetection(modelbuilder.points);
    return;
}

void CGALProcessing::poissonReconstructionWrapper() {
    Polyhedron_3 output;
    poissonSurfaceReconstruction(this->pwn_points, output);
    this->polyhedron3 = output;
}

template<class T>
void CGALProcessing::printInfo(T & P)
{
    std::cout << "Polyhedron Vertices: " << P.size_of_vertices() << " Faces: " << P.size_of_facets() << std::endl;
}

void CGALProcessing::toModelInfo(Polyhedron_3 & P, ModelBuilder::modelInfo & model)
{
    for (Vertex_iterator it=P.vertices_begin(); it != P.vertices_end(); it++) {
        Point_3 point = it->point();
        ModelBuilder::vertexInfo v;
        v.x = point.cartesian(0);
        v.y = point.cartesian(1);
        v.z = point.cartesian(2);
        model.vertices.push_back(v);
    }
    for (Facet_iterator it=P.facets_begin(); it != P.facets_end(); it++) {
        Halfedge_facet_circulator j = it->facet_begin();
        if (CGAL::circulator_size(j) >= 3) {
            ModelBuilder::faceInfo face;
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

void CGALProcessing::toPwnVector(Polyhedron_3 & P, Pwn_vector & points) {
    // Still need to extract normals from Polyhedron
    for (Vertex_iterator it=P.vertices_begin(); it != P.vertices_end(); it++) {
        Point_3 point  = it->point();
        Vector_3 vec;
        Point_with_normal pwn(point, vec);
        points.push_back(pwn);
    }
}



// TODO
// Core processing
void CGALProcessing::polyhedronProcessing() {
    // Polyhedron_3 poly;
    //    Pwn_vector points;
    //    this->readPlyToPwn(filename, points);
    //    this->advancingFrontSurfaceReconstruction(points, poly);
    //    this->printPolyhedronInfo(poly);
    // modelInfo model = readPlyFile(filename);
    // modelInfoToPolyhedron(model, poly);
    Polyhedron_3 out;
    surfaceMeshGeneration(polyhedron3, out);
    printInfo(out);
}


void CGALProcessing::advancingFrontSurfaceReconstruction(Pwn_vector & points) {
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
    FacetIndices point_indices(boost::counting_iterator<std::size_t>(0),
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
    
    this->incrementBuilder(polyhedron3, structured_pts, output);
    std::cout << "Info after advancing front" <<std::endl;
    printInfo(polyhedron3);
    // TODO - Maybe add structured_pts to the modelbuilder.points


    // TODO - Fix this outfile 
    // FUCK YOU WALEED
    // std::string outname = this->plyFolder + "advancing3.off";
    // std::ofstream f (outname.c_str());
    // f << "OFF\n" << structured_pts.size () << " " << output.size() << " 0\n"; // Header
    // for (std::size_t i = 0; i < structured_pts.size (); ++ i)
    //     f << structured_pts[i].first << std::endl;
    // for (std::size_t i = 0; i < output.size (); ++ i)
    //     f << "3 "
    //     << output[i][0] << " "
    //     << output[i][1] << " "
    //     << output[i][2] << std::endl;
    // std::cerr << "all done\n" << std::endl;
    // f.close();
}

void CGALProcessing::pointSetShapeDetection(Pwn_vector & points){
    // Reads a PLY file to a Pwn_vector
    // Runs Efficient_ransac on the points and normals
    // Detects planes and segments the point set according to planes
    // Writes the detected shapes to individual PLY files
    
    CGAL::Timer timer;
    
    // modelbuilder.readPlyToPwn(this->fname, points);
    
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
    
    writeShapesToFiles(shapes, points);
    
    return;
    
    // WTF - Assuming this is never run
    // Efficient_ransac::Shape_range::iterator it = shapes.begin();
    
    // while (it != shapes.end()) {
    //     boost::shared_ptr<Efficient_ransac::Shape> shape = *it;
    //     std::cout << (*it)->info() << std::endl;
        
    //     FT sumDistances = 0;
    //     FacetIndices::const_iterator indexIt = (*it)->indices_of_assigned_points().begin();
        
    //     while (indexIt != (*it)->indices_of_assigned_points().end()) {
    //         const Point_with_normal &p = *(points.begin() + (*indexIt));
    //         sumDistances += CGAL::sqrt((*it)->squared_distance(p.first));
    //         indexIt++;
    //     }
        
    //     FT averageDistance = sumDistances / shape->indices_of_assigned_points().size();
    //     std::cout << " average distance: " << averageDistance << std::endl;
    //     it++;
    // }
    
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

void CGALProcessing::poissonSurfaceReconstruction(Pwn_vector & points, Polyhedron_3 & output) {
    double average_spacing = CGAL::compute_average_spacing<CGAL::Sequential_tag>(points.begin(), points.end(), CGAL::First_of_pair_property_map<Point_with_normal>(), 6);
    
    CGAL::poisson_surface_reconstruction_delaunay(points.begin(), points.end(), CGAL::First_of_pair_property_map<Point_with_normal>(), CGAL::Second_of_pair_property_map<Point_with_normal>(), output, average_spacing);
}

void CGALProcessing::poissonSurfaceReconstruction(Pwn_vector & points) {
    // Poisson options
    FT sm_angle = 20.0; // Min triangle angle in degrees.
    FT sm_radius = 30; // Max triangle size w.r.t. point set average spacing.
    FT sm_distance = 0.375; // Surface Approximation error w.r.t. point set average spacing.
    // Reads the point set file in points[].
    // Note: read_xyz_points_and_normals() requires an iterator over points
    // + property maps to access each point's position and normal.
    // The position property map can be omitted here as we use iterators over Point_3 elements.
//    PointList points;
    
    // Creates implicit function from the read points using the default solver.
    // Note: this method requires an iterator over points
    // + property maps to access each point's position and normal.
    // The position property map can be omitted here as we use iterators over Point_3 elements.
    Poisson_reconstruction_function function(points.begin(), points.end(),
                                             CGAL::make_normal_of_point_with_normal_pmap(Pwn_vector::value_type()) );
    // Computes the Poisson indicator function f()
    // at each vertex of the triangulation.
    if ( ! function.compute_implicit_function() )
        return;
    // Computes average spacing
//    FT average_spacing = CGAL::compute_average_spacing<CGAL::Sequential_tag>(points.begin(), points.end(), 6 /* knn = 1 ring */);
    FT average_spacing = 2;
    // Gets one point inside the implicit surface
    // and computes implicit function bounding sphere radius.
    Point_3 inner_point = function.get_inner_point();
    Sphere_3 bsphere = function.bounding_sphere();
    FT radius = std::sqrt(bsphere.squared_radius());
    // Defines the implicit surface: requires defining a
    // conservative bounding sphere centered at inner point.
    FT sm_sphere_radius = 5.0 * radius;
    FT sm_dichotomy_error = sm_distance*average_spacing/1000.0; // Dichotomy error must be << sm_distance
    Surface_3 surface(function,
                      Sphere_3(inner_point,sm_sphere_radius*sm_sphere_radius),
                      sm_dichotomy_error/sm_sphere_radius);
    // Defines surface mesh generation criteria
    CGAL::Surface_mesh_default_criteria_3<STr> criteria(sm_angle,  // Min triangle angle (degrees)
                                                        sm_radius*average_spacing,  // Max triangle size
                                                        sm_distance*average_spacing); // Approximation error
    // Generates surface mesh with manifold option
    STr tr; // 3D Delaunay triangulation for surface mesh generation
    C2t3 c2t3(tr); // 2D complex in 3D Delaunay triangulation
    CGAL::make_surface_mesh(c2t3,                                 // reconstructed mesh
                            surface,                              // implicit surface
                            criteria,                             // meshing criteria
                            CGAL::Manifold_with_boundary_tag());  // require manifold mesh
    if(tr.number_of_vertices() == 0)
        return;
    // saves reconstructed surface mesh
    std::ofstream out("335Poisson.off");
    Polyhedron_3 output_mesh;
    CGAL::output_surface_facets_to_polyhedron(c2t3, output_mesh);
    out << output_mesh;
    // computes the approximation error of the reconstruction
    double max_dist =
    CGAL::Polygon_mesh_processing::approximate_max_distance_to_point_set(output_mesh,
                                                                         points,
                                                                         4000);
    std::cout << "Max distance to point_set: " << max_dist << std::endl;
}

void CGALProcessing::surfaceMeshGeneration(Polyhedron_3 & input, Polyhedron_3 & output) {
    // Follows example from https://doc.cgal.org/latest/Mesh_3/index.html#Mesh_33DPolyhedralDomains
    // Takes a Polyhedron_3 as input
    
    Mesh_polyhedron polyhedron;
    // std::string fname = this->offFolder + "temp.off";
    ModelBuilder::modelInfo model;
    // TODO
    // polyhedronToModelInfo(input, model);
    //    writeOffFile(fname, model);
    //    std::ofstream fout(fname);
    //    fout << input;
    //    CGAL::write_off(fout, input);
    //    std::ifstream fin(fname);
    //    fin >> polyhedron;
    //    CGAL::read_off(fin, polyhedron);
    //    modelInfo model;
    //    this->polyhedronToModelInfo(input, model);
    PointVector points;
    FacetVector faces;
    // TODO
    // this->modelInfoToPointAndFacetVectors(model, points, faces);
    incrementBuilder(polyhedron, points, faces);
    std::cout << "Finished Builder with faces: " << polyhedron.size_of_facets() << endl;
    
    if (!CGAL::is_triangle_mesh(polyhedron)){
        std::cerr << "Input geometry is not triangulated." << std::endl;
        return;
    }
    // Create domain
    Mesh_domain domain(polyhedron);
    std::cout << "Initialized domain" << std::endl;

    // Get sharp features
     domain.detect_features();
     std::cout << "Sharp Features" << endl;

    // Mesh criteria
    //    Mesh_criteria criteria(CGAL::parameters::edge_size = 0.025, CGAL::parameters::facet_angle = 25, CGAL::parameters::facet_size = 0.05, CGAL::parameters::facet_distance = 0.005, CGAL::parameters::cell_radius_edge_ratio = 3, CGAL::parameters::cell_size = 0.05);
    //    Mesh_criteria criteria(CGAL::parameters::facet_angle = 1);
    Mesh_criteria criteria(CGAL::parameters::facet_angle = 30);
    std::cout << "Set Criteria" << endl;

    // Mesh generation
    C3t3 c3t3 = CGAL::make_mesh_3<C3t3>(domain, criteria);
    std::cout << "Finished Mesh Make" << endl;
    
    // Output
    std::ofstream medit_file("out.mesh");
    c3t3.output_to_medit(medit_file);
}

void CGALProcessing::surfaceMeshGeneration(std::string outFile) {

    // TODO CHECK THIS BEHAVIOUR
    Mesh_polyhedron polyhedron = this->meshPolyhedron;
    CGAL::convex_hull_3(polyhedron.points_begin(), polyhedron.points_end(), polyhedron);
    std::cout << "Generated convex hull: Vertices: " << polyhedron.size_of_vertices() << " Faces: " << polyhedron.size_of_facets() << std::endl;
    //    if (polyhedron.points_begin() != polyhedron.points_end()) {
    //        Kernel::Iso_cuboid_3 cube = CGAL::bounding_box(polyhedron.points_begin(), polyhedron.points_end());
    //        CGAL::Bbox_3 bbox = cube.bbox();
    //    }
    //    else {
    //        std::cerr << "Polyhedron points begin == points end\n";
    //        return;
    //    }
    Mesh_domain domain(polyhedron);
    std::cout << "Constructed domain\n";
    domain.detect_features();
    std::cout << "Detected sharp features\n";
    Mesh_criteria criteria(CGAL::parameters::facet_angle = 30);
    std::cout << "Set criteria\n";

    //  WTF
    C3t3 c3t3 = CGAL::make_mesh_3<C3t3>(domain, criteria);
    std::cout << "Created mesh\n";
    std::ofstream medit_file(outFile);
    c3t3.output_to_medit(medit_file);
    std::cout << "Wrote output: " << outFile << std::endl;
}

// IO
inline
void CGALProcessing::writeShapesToFiles(Efficient_ransac::Shape_range shapes, Pwn_vector points) {
    Efficient_ransac::Shape_range::iterator it = shapes.begin();
    int shapeNum=0;
    while (it != shapes.end()) {
        FacetIndices indices = it->get()->indices_of_assigned_points();
        FacetIndices::const_iterator iti = indices.begin();
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
        // std::string outFile = this->plyFolder + "/CGAL/" + std::to_string(shapeNum) + ".ply";
        std::string outFile = "../models/CGAL/" + std::to_string(shapeNum) + ".ply";
        // IS THERE ANY WAY WE CAN WRITE ALL OF THE SHAPES INTO ONE FILE
        if (modelbuilder.writePlyPointsAndNormals(newPoints, outFile)) {
            std::cerr << "Wrote file: " << outFile << std::endl;
        }
        else {
            std::cerr << "Error writing file: " << outFile << std::endl;
        }
        shapeNum++;
        it++;
    }
}

// Increment Builders
inline
void CGALProcessing::incrementBuilder(Mesh_polyhedron &P, PointVector &points, FacetVector &faces) {
    mesh_polyhedron_builder<Mesh_hds> builder(points, faces);
    P.delegate(builder);
}

inline
void CGALProcessing::incrementBuilder(Polyhedron_3 &P, PointVector &points, FacetVector &faces) {
    polyhedron_builder<HalfedgeDS> builder(points, faces);
    P.delegate(builder);
}

inline
void CGALProcessing::incrementBuilder(Polyhedron_3 &P, Pwn_vector &points, std::vector<Facet> &faces) {
    PointVector p;
    FacetVector f;
    this->pwnToPointVector(points, p);
    this->facetVectorToStd(faces, f);
    polyhedron_builder<HalfedgeDS> builder(p, f);
    P.delegate(builder);
}


// Increment Builder Helpers
inline
void CGALProcessing::pwnToPointVector(Pwn_vector & input, PointVector & output) {
    output.clear();
    for (Point_with_normal pwn: input) {
        Point_3 point = pwn.first;
        output.push_back(point);
    }
}

inline
void CGALProcessing::facetVectorToStd(std::vector<Facet> & input, FacetVector & output) {
    output.clear();
    for (Facet facet: input) {
        FacetIndices indices;
        indices.push_back(facet[0]);
        indices.push_back(facet[1]);
        indices.push_back(facet[2]);
        output.push_back(indices);
    }
}
