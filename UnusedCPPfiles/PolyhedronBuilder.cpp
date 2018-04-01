// #include "cgalProcessing.h"

// void CGALProcessing::PolyhedronBuilder::ToModelInfo(Polyhedron_3 & P, ModelBuilder::modelInfo & model)
// {
//     for (Vertex_iterator it=P.vertices_begin(); it != P.vertices_end(); it++) {
//         Point_3 point = it->point();
//         ModelBuilder::vertexInfo v;
//         v.x = point.cartesian(0);
//         v.y = point.cartesian(1);
//         v.z = point.cartesian(2);
//         model.vertices.push_back(v);
//     }
//     for (Facet_iterator it=P.facets_begin(); it != P.facets_end(); it++) {
//         Halfedge_facet_circulator j = it->facet_begin();
//         if (CGAL::circulator_size(j) >= 3) {
//             ModelBuilder::faceInfo face;
//             do {
//                 face.vertexIndices.push_back(std::distance(P.vertices_begin(), j->vertex()));
//             } while ( ++j != it->facet_begin());
//             model.faces.push_back(face);
//         }
//         else {
//             std::cerr << "Invalid facet circulator size" << std::endl;
//         }
//     }
// }

// void CGALProcessing::PolyhedronBuilder::ToPwnVector(Polyhedron_3 & P, Pwn_vector & points) {
//     // Still need to extract normals from Polyhedron
//     for (Vertex_iterator it=P.vertices_begin(); it != P.vertices_end(); it++) {
//         Point_3 point  = it->point();
//         Vector_3 vec;
//         Point_with_normal pwn(point, vec);
//         points.push_back(pwn);
//     }
// }

// template<class T>
// void CGALProcessing::PolyhedronBuilder::PrintInfo(T & P)
// {
//     std::cout << "Polyhedron Vertices: " << P.size_of_vertices() << " Faces: " << P.size_of_facets() << std::endl;
// }