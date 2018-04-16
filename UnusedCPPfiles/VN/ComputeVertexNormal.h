
#ifndef _SMESHLIB_OPERATIONS_COMPUTEVERTEXNORMAL_H_
#define _SMESHLIB_OPERATIONS_COMPUTEVERTEXNORMAL_H_

#include "PropertyMap.h"

namespace SMeshLib   {
namespace Operations {
;

// The ComputeVertexNormal is a functor (delegate) to compute the normal of a vertex in CGAL::Polyhdeon_3.
// operator() is thread-safe.
// TPolyhedron is a type of CGAL::Polyhdeon_3.
// TFacetNormals is a property map which associates a normal vector to each facet in the CGAL::Polyhdeon_3.
// The vertex normal is the average of all facet normals incident on it.
template<class TPolyhedron, class TFacetNormals>
struct ComputeVertexNormal
{
public:
	
	// Redefine types from TPoly for convenience.
	typedef typename TPolyhedron::Vertex                                  Vertex;
	typedef typename TPolyhedron::Facet                                   Facet;
	typedef typename TPolyhedron::Traits::Vector_3                        Vector3;
	typedef typename TPolyhedron::Halfedge_around_vertex_const_circulator HalfEdgeConstCirculator;
	
	// Return type of operator() required by QtConcurrent.
	typedef Vector3 result_type;
	
public:
	
	ComputeVertexNormal(const TFacetNormals& facetNormals_)
		: facetNormals(facetNormals_)
	{}
	
	// Compute normal of the given vertex.
	inline Vector3 operator() (const Vertex& v) const
	{
		Vector3                 n = CGAL::NULL_VECTOR;
		HalfEdgeConstCirculator s = v.vertex_begin();
		HalfEdgeConstCirculator e = s;
		CGAL_For_all(s, e)
		{
			// Border edge doesn't have facet and hence no normal.
			if(!s->is_border())
			{
				n = n + facetNormals.value(&*(s->facet()));
			}
		}
		return n/std::sqrt(n*n);
	}
	
public:
	
	const TFacetNormals& facetNormals;
};

};	// End namespace Operations.
};	// End namespace SMeshLib.

#endif // _SMESHLIB_OPERATIONS_COMPUTEVERTEXNORMAL_H_
