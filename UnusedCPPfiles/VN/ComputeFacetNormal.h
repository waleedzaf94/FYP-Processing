
#ifndef _SMESHLIB_OPERATIONS_COMPUTEFACETNORMAL_H_
#define _SMESHLIB_OPERATIONS_COMPUTEFACETNORMAL_H_

namespace SMeshLib   {
namespace Operations {
;

// The ComputeFacetNormal is a functor (delegate) to compute the normal of a facet in CGAL::Polyhdeon_3.
// ComputeFacetNormal is thread-safe.
// TPolyhedron is a type of CGAL::Polyhdeon_3.
template<class TPolyhedron>
struct ComputeFacetNormal
{
public:
	
	// Redefine types from TPolyhedron for convenience.
	typedef typename TPolyhedron::Facet                 Facet;
	typedef typename TPolyhedron::Halfedge_const_handle HalfedgeConstHandle;
	typedef typename TPolyhedron::Traits::Point_3       Point3;
	typedef typename TPolyhedron::Traits::Vector_3      Vector3;
	
	// Return type of operator() required by QtConcurrent.
	typedef Vector3 result_type;
	
public:
	
	// Compute normal of the given facet.
	// Facet can be triangle, quadrilateral or a polygon as long as its planar.
	// Use first three vertices to compute the normal.
	inline Vector3 operator() (const Facet& f) const
	{
		HalfedgeConstHandle h  = f.halfedge();
		Point3              p1 = h->vertex()->point();
		Point3              p2 = h->next()->vertex()->point();
		Point3              p3 = h->next()->next()->vertex()->point();
		Vector3             n  = CGAL::cross_product(p2-p1, p3-p1);
		return n / std::sqrt(n*n);
	}
};

};	// End namespace Operations.
};	// End namespace SMeshLib.

#endif // _SMESHLIB_OPERATIONS_COMPUTEFACETNORMAL_H_
