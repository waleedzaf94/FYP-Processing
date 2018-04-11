//
//  polyhedronBuilders.h
//  processing
//
//  Created by Waleed Zafar on 11/4/2018.
//

#ifndef polyhedronBuilders_h
#define polyhedronBuilders_h

#include "Definitions.hpp"

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


#endif /* polyhedronBuilders_h */
