
#include "ImportOBJ.h"
#include "ComputeFacetNormal.h"
#include "ComputeVertexNormal.h"
#include "PropertyMap.h"
#include "CGAL/Simple_cartesian.h"
#include <CGAL/Polyhedron_items_3.h>
#include "CGAL/HalfedgeDS_list.h"
#include "CGAL/Polyhedron_3.h"

typedef CGAL::Simple_cartesian<double> Kernel;
typedef CGAL::Polyhedron_3<Kernel, 
	                       CGAL::Polyhedron_items_3, 
						   CGAL::HalfedgeDS_list> CgalPolyhedron;

// Compute the normal of all vertices in a CGAL::Polyhedron_3 and stores it in 
// vertexNormals.
// TPolyhedron is a type of CGAL::Polyhedron_3.
// TVertexNormals is a property map which associates a normal vector to vertex 
// in the CGAL::Polyhdeon_3.
// TFacetNormals is a property map which associates a normal vector to facet 
// in the CGAL::Polyhdeon_3.
template<class TPolyhedron, class TVertexNormals, class TFacetNormals>
void computeVertexNormals(const TPolyhedron& polyhedron, TVertexNormals* vertexNormals, const TFacetNormals& facetNormals)
{
	if(vertexNormals == 0)
	{
		return;
	}
	
	typename TPolyhedron::Vertex_const_iterator _begin = polyhedron.vertices_begin();
	typename TPolyhedron::Vertex_const_iterator _end   = polyhedron.vertices_end();
	
	SMeshLib::Operations::ComputeVertexNormal<TPolyhedron, TFacetNormals> _computeVertexNormal(facetNormals);
	CGAL_For_all(_begin, _end)
	{
		vertexNormals->setValue(&*_begin, _computeVertexNormal(*_begin));
	}
}

// Compute the normal of all facets in a CGAL::Polyhedron_3 and stores it in facetNormals.
// TPolyhedron is a type of CGAL::Polyhedron_3.
// TFacetNormals is a property map which associates a normal vector to facet in the CGAL::Polyhdeon_3. 
template<class TPolyhedron, class TFacetNormals>
void computeFacetNormals(const TPolyhedron& polyhedron, TFacetNormals* facetNormals)
{
	if(facetNormals == 0)
	{
		return;
	}
	
	typename TPolyhedron::Facet_const_iterator _begin = polyhedron.facets_begin();
	typename TPolyhedron::Facet_const_iterator _end   = polyhedron.facets_end();

	SMeshLib::Operations::ComputeFacetNormal<TPolyhedron> _computeFacetNormal;
	CGAL_For_all(_begin, _end)
	{
		facetNormals->setValue(&*_begin, _computeFacetNormal(*_begin));
	}
}

void testComputeVertexNormal()
{
	CgalPolyhedron _poly;
	SMeshLib::IO::importOBJ("Venus.obj", &_poly);
	
	typedef SMeshLib::PropertyMap<const CgalPolyhedron::Facet*, CgalPolyhedron::Traits::Vector_3> FacetNormalPM;
	FacetNormalPM _facetNormals;
	computeFacetNormals(_poly, &_facetNormals);
	
	typedef SMeshLib::PropertyMap<const CgalPolyhedron::Vertex*, CgalPolyhedron::Traits::Vector_3> VertexNormalPM;
	VertexNormalPM _vertexNormals;
	computeVertexNormals(_poly, &_vertexNormals, _facetNormals);
}
