#ifndef CGAL_PROCESSING_H
#define CGAL_PROCESSING_H
#include "ModelBuilder.hpp"
#include "Definitions.hpp"
#include "polyhedronBuilders.h"
//#include "ComputeFacetNormal.h"
//#include "ComputeVertexNormal.h"
//#include "PropertyMap.h"

class CGALProcessing {

    public:
    
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
        // modelbuilder();
    };
    void inputPolyhedron(std::string, std::string);
    void outputPolyhedron(std::string, std::string);
    
    // Algorithm Wrappers
    void advancingFrontWrapper(const std::vector<double> args = std::vector<double>());
    void shapeDetectionWrapper();
    void shapeDetectionWrapper(std::string);
    void poissonReconstructionWrapper();
    void surfaceMeshGenerationWrapper();

    private:
    Polyhedron_3 polyhedron3;
    Mesh_polyhedron meshPolyhedron;
    ModelBuilder modelbuilder;
    Pwn_vector pwn_points;
    std::string inputFileType;
    std::string lastUpdated; // polyhedron3, meshPolyhedron, pwn_points, modelbuilder
    
    Polyhedron_vector polyhedrons;
    Pwn_vector_vector pwnSets;

    // Algorithms
    void advancingFrontSurfaceReconstruction(Pwn_vector & points, double probability = 0.05, int min_points = 100, double epsilon = 0.005, double cluster_epsilon = 0.05, double normal_threshold = 0.8);
    void pointSetShapeDetection(Pwn_vector &, Pwn_vector &);
    void poissonSurfaceReconstruction(Pwn_vector &, Polyhedron_3 &);
    void poissonSurfaceReconstruction(Pwn_vector &);
    void surfaceMeshGeneration(Polyhedron_3 &, Polyhedron_3 &);
    void surfaceMeshGeneration(std::string);
    std::vector<double> computeAFArguments(Pwn_vector & points);

    // IO
    void writeShapesToFiles(CGAL::Shape_detection_3::Efficient_RANSAC<Traits>::Shape_range, Pwn_vector);
    void writeShapesToFile(CGAL::Shape_detection_3::Efficient_RANSAC<Traits>::Shape_range, Pwn_vector &, Pwn_vector &);
    
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

#endif //CGAL_PROCESSING_H
