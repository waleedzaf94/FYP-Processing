//VCG
#include <vcg/complex/complex.h>
#include <vcg/complex/algorithms/inertia.h>
#include <vcg/complex/algorithms/hole.h>
#include <vcg/complex/algorithms/local_optimization.h>
#include <vcg/complex/algorithms/point_sampling.h>
#include <vcg/complex/algorithms/smooth.h>
#include <vcg/complex/algorithms/refine.h>

//#include <vcg/complex/algorithms/ransac_matching.h>
#include <vcg/complex/algorithms/local_optimization/tri_edge_flip.h>
#include <vcg/complex/algorithms/update/color.h>
#include <vcg/complex/algorithms/update/selection.h>
#include <vcg/space/point_matching.h>
//Topology Computation
#include <vcg/complex/algorithms/update/topology.h>
#include <vcg/complex/algorithms/update/flag.h>
#include <vcg/complex/algorithms/update/normal.h>
//Half Edge Iterators
#include <vcg/simplex/face/pos.h>
//Input Output
#include <wrap/io_trimesh/import.h>
#include <wrap/io_trimesh/export.h>

class MyVertex;
class MyEdge;
class MyFace;

struct MyUsedTypes : public vcg::UsedTypes<vcg::Use<MyVertex> ::AsVertexType, vcg::Use<MyEdge> ::AsEdgeType, vcg::Use<MyFace> ::AsFaceType> {};

class MyVertex : public vcg::Vertex<MyUsedTypes, vcg::vertex::Coord3f, vcg::vertex::Normal3f, vcg::vertex::BitFlags> {};

class MyEdge : public vcg::Edge<MyUsedTypes> {};

class MyFace : public vcg::Face<MyUsedTypes, vcg::face::FFAdj, vcg::face::VertexRef, vcg::face::BitFlags> {};

class MyMesh : public vcg::tri::TriMesh<std::vector<MyVertex>, std::vector<MyFace>, std::vector<MyEdge> > {};

class MyDelaunayFlip : public vcg::tri::TriEdgeFlip<MyMesh, MyDelaunayFlip> {
public:
    typedef vcg::tri::TriEdgeFlip<MyMesh, MyDelaunayFlip> TEF;
    inline MyDelaunayFlip (const TEF::PosType &p, int i, vcg::BaseParameterClass *pp) : TEF(p, i, pp) {}
};

class VCGProcessing {
private:
    //Members
    #ifdef amanDev
    string plyFolder = "/home/aman/Desktop/FYP-Processing/models/PLY";
    string objFolder = "/home/aman/Desktop/FYP-Processing/models/OBJ";
    string fname = "/home/aman/Desktop/FYP-Processing/335.obj";
    #else
    string plyFolder = "/Users/waleedzafar/projects/fyp/one/models/PLY/";
    string objFolder = "/Users/waleedzafar/projects/fyp/one/models/OBJ/";
    string fname = "/Users/waleedzafar/projects/fyp/one/335.obj";
    #endif
    MyMesh mesh;

    //Functions
    void ransacTest(MyMesh&);
    void createBoundingBox(MyMesh&);
    void holeFillTrivialEar(MyMesh&);
    void importOBJAsMesh(string, MyMesh&);
    bool normalTest(typename vcg::face::Pos<MyMesh::FaceType>);
public:
    //Functions
    void setPLYFolder(string);
    void setOBJFolder(string);
    MyMesh& getMesh();
    void importOBJAsMesh(string);
    void saveMeshAsPLY(MyMesh&, string);
    void saveMeshAsOBJ(MyMesh&, string);
    void printVertexLocation(MyVertex&);
};
