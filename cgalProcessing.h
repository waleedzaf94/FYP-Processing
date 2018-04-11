#ifndef CGAL_PROCESSING_H
#define CGAL_PROCESSING_H
#include "ModelBuilder.hpp"
#include "Definitions.hpp"

class CGALProcessing {

    public:
    // #ifdef amanDev
    // std::string plyFolder = "/home/aman/Desktop/FYP-Processing/models/PLY";
    // std::string objFolder = "/home/aman/Desktop/FYP-Processing/models/OBJ";
    // std::string offFolder = "/home/aman/Desktop/FYP-Processing/models/OFF/";
    // std::string modelsFolder = "/home/aman/Desktop/FYP-Processing/models/";
    // std::string fname = "/home/aman/Desktop/FYP-Processing/335.obj";
    // #else
    // std::string plyFolder = "/Users/waleedzafar/projects/fyp/one/models/PLY/";
    // std::string objFolder = "/Users/waleedzafar/projects/fyp/one/models/OBJ/";
    // std::string offFolder = "/Users/waleedzafar/projects/fyp/one/models/OFF/";
    // std::string modelsFolder = "/Users/waleedzafar/projects/fyp/one/models/";
    // std::string fname = "/Users/waleedzafar/projects/fyp/one/models/Chi_11.ply";
    // #endif
    
    
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
    
    /// Public functions
    // Control
    CGALProcessing() {
        // polyhedronbuilder = new PolyhedronBuilder();
        // modelbuilder();
    };
    void inputPolyhedron(std::string, std::string);
    void outputPolyhedron(std::string, std::string);
    void polyhedronProcessing();
    void polyhedronProcessing(std::string);
    
    // Algorithm Wrappers
    void advancingFrontWrapper();
    void shapeDetectionWrapper();
    void poissonReconstructionWrapper();
    void surfaceMeshGeneration(Polyhedron_3 &, Polyhedron_3 &);
    void surfaceMeshGeneration(std::string);

    private:
    Polyhedron_3 polyhedron3;
    Mesh_polyhedron meshPolyhedron;
    ModelBuilder modelbuilder;
    Pwn_vector pwn_points;
    std::string inputFileType;
    
    Polyhedron_vector polyhedrons;
    Pwn_vector_vector pwnSets;

    // Algorithms
    void advancingFrontSurfaceReconstruction(Pwn_vector &);
    void pointSetShapeDetection(Pwn_vector &);
    void poissonSurfaceReconstruction(Pwn_vector &, Polyhedron_3 &);
    void poissonSurfaceReconstruction(Pwn_vector &);

    // IO
    void writeShapesToFiles(CGAL::Shape_detection_3::Efficient_RANSAC<Traits>::Shape_range, std::vector<Point_with_normal>);
    
    template <class T>
    void printInfo(T &);
    
    // Helpers
    void facetVectorToStd(std::vector<Facet> &, FacetVector &);
    void incrementBuilder(Mesh_polyhedron &, PointVector &, FacetVector &);
    void incrementBuilder(Polyhedron_3 &, PointVector &, FacetVector &);
    void incrementBuilder(Polyhedron_3 &, Pwn_vector &, std::vector<Facet> &);
    void pwnToPointVector(Pwn_vector &, PointVector &);
    void toModelInfo(Polyhedron_3 &, ModelBuilder::modelInfo & );
    void toPwnVector(Polyhedron_3 &, Pwn_vector & ) ;

};

// A modifier creating a triangle with the incremental builder.
template<class HDS>
class polyhedron_builder : public CGAL::Modifier_base<HDS> {
public:
    PointVector             &coords;
    std::vector<std::vector<std::size_t> >  &faces;
    
    polyhedron_builder(PointVector &_coords, std::vector<std::vector<std::size_t> > &_tris ) : coords(_coords), faces(_tris) {
        
    }
    
    void operator()(HDS & hds) {
        
        // create a cgal incremental builder
        CGAL::Polyhedron_incremental_builder_3<HDS> B( hds, true);
        B.begin_surface( coords.size(), faces.size());
        
        // add the polyhedron vertices
        for( Point_3 i : coords ){
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

// A modifier creating a triangle with the incremental builder.
template<class Mesh_hds>
class mesh_polyhedron_builder : public CGAL::Modifier_base<Mesh_hds> {
public:
    PointVector             &coords;
    std::vector<std::vector<std::size_t> >  &faces;
    
    mesh_polyhedron_builder(PointVector &_coords, std::vector<std::vector<std::size_t> > &_tris ) : coords(_coords), faces(_tris) {
        
    }
    
    void operator()(Mesh_hds & hds) {
        
        // create a cgal incremental builder
        CGAL::Polyhedron_incremental_builder_3<Mesh_hds> B( hds, true);
        B.begin_surface( coords.size(), faces.size());
        
        // add the polyhedron vertices
        for( Point_3 i : coords ){
            B.add_vertex(i);
        }
        // add the polyhedron triangles
        for (std::vector<size_t> i: faces) {
            try {
                if (B.test_facet(i.begin(), i.end())) {
                    if (i.size() != 3)
                        continue;
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
