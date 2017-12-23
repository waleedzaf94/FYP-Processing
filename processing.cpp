
#define amanDev

//Standard
#include <iostream>
#include <stdio.h>
#include <string>
#include <vector>

//PCL
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/common/common_headers.h>
#include <boost/thread/thread.hpp>

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

using namespace std;

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

class ProcessXYZ {
private:
    //Members
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr;
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
    void importOBJAsPSD(string);
    void importOBJAsMesh(string);
    void saveModelAsPLY(string);
    void statisticalOutlierRemoval(pcl::PointCloud<pcl::PointXYZ>::ConstPtr);
    void radiusOutlierRemoval(pcl::PointCloud<pcl::PointXYZ>::ConstPtr);
    void conditionalOutlierRemoval(pcl::PointCloud<pcl::PointXYZ>::ConstPtr);
    void ransacTest(MyMesh&);
    void createBoundingBox(MyMesh&);
    void holeFillTrivialEar(MyMesh&);
    bool normalTest(typename vcg::face::Pos<MyMesh::FaceType>);
public:
    //Functions
    void setPLYFolder(string);
    void setOBJFolder(string);
    MyMesh& getMesh();
    void viewModel();
    void viewModel(pcl::PointCloud<pcl::PointXYZ>);
    void processModel(string);
    void saveModelAsPLY(pcl::PointCloud<pcl::PointXYZ>, string);
    void saveMeshAsPLY(MyMesh&, string);
    void saveMeshAsOBJ(MyMesh&, string);
    void importOBJAsMesh(string, MyMesh&);
    void printVertexLocation(MyVertex&);
};

int main() {
    #ifdef amanDev
    string fname = "/home/aman/Desktop/FYP-Processing/335.obj";
    #else
    string fname = "/Users/waleedzafar/projects/fyp/one/335.obj";
    #endif
    //cout << "Input filename: ";
    //cin >> fname;
    ProcessXYZ processing;
    processing.processModel(fname);
    return 0;
}

void ProcessXYZ::processModel(string filename) {
    this->importOBJAsMesh(filename);
    this->holeFillTrivialEar(this->mesh);
//    this->ransacTest(this->mesh);
    // this->saveMeshAsOBJ(this->mesh, this->objFolder + "ransacTest.obj");
//    this->saveMeshAsOBJ(this->mesh, this->objFolder + "meshOrig.obj");
//    this->importOBJAsPSD(filename);
//    this->viewModel();
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr = this->cloud.makeShared();
//    this->saveModelAsPLY(this->plyFolder + "orig.ply");
//    this->statisticalOutlierRemoval(cloudPtr);
//    this->radiusOutlierRemoval(cloudPtr);
//    this->conditionalOutlierRemoval(cloudPtr);
}

void ProcessXYZ::conditionalOutlierRemoval(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ConditionAnd<pcl::PointXYZ>::Ptr rangeCond(new pcl::ConditionAnd<pcl::PointXYZ>());
    rangeCond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::GT, 0.0)));
    rangeCond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ>("Z", pcl::ComparisonOps::LT, 0.8)));
    pcl::ConditionalRemoval<pcl::PointXYZ> cor;
    cor.setCondition(rangeCond);
    cor.setInputCloud(cloud);
    cor.setKeepOrganized(true);
    cor.filter(*filtered);
    this->saveModelAsPLY(*filtered, this->plyFolder + "corFiltered.ply");
}

void ProcessXYZ::radiusOutlierRemoval(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;
    ror.setInputCloud(cloud);
    ror.setRadiusSearch(0.8);
    ror.setMinNeighborsInRadius(2);
    ror.filter(*filtered);
    this->saveModelAsPLY(*filtered, this->plyFolder + "rorFiltered.ply");
}

void ProcessXYZ::statisticalOutlierRemoval(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor (false);
    sor.setInputCloud(cloud);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*filtered);
    this->saveModelAsPLY(*filtered, this->plyFolder + "sorFiltered.ply");
    
    cout << "Completed sor." << endl;
}

// unused 
void ProcessXYZ::importOBJAsPSD(string filename) {
    int imp = pcl::io::loadOBJFile(filename, this->cloud);
    this->cloudPtr = this->cloud.makeShared();
    if (imp == 0)
        cout << "Imported file at " << filename << endl;
}

void ProcessXYZ::importOBJAsMesh(string filename) {
    this->importOBJAsMesh(filename, this->mesh);
}

void ProcessXYZ::importOBJAsMesh(string filename, MyMesh& mesh) {
    int zero;
    if (vcg::tri::io::ImporterOBJ<MyMesh>::Open(mesh, filename.c_str(), zero) == vcg::tri::io::ImporterOBJ<MyMesh>::E_NOERROR) {
        cout << "Imported OBJ as Mesh from " << filename << endl;
    }
    else {
        cout << "Error importing file: " << filename << endl;
    }
}

void ProcessXYZ::saveMeshAsPLY(MyMesh &mesh, string filename) {
    //TODO: Fix this. Leads to some library error or sth.
//    int err = vcg::tri::io::ExporterPLY<MyMesh>::Save(mesh, filename.c_str(), 0);
    int err=-1;
    if (err == 0)
        cout << "Saved mesh at " << filename << endl;
    else
        cout << "Error saving mesh at " << filename << endl;
}

void ProcessXYZ::saveMeshAsOBJ(MyMesh& mesh, string filename) {
    int err = vcg::tri::io::ExporterOBJ<MyMesh>::Save(mesh, filename.c_str(), 0);
    if (err == 0)
        cout << "Saved mesh at " << filename << endl;
    else
        cout << "Error saving mesh at " << filename << endl;
}

void ProcessXYZ::viewModel(pcl::PointCloud<pcl::PointXYZ> cloud) {
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ> (this->cloudPtr, "one");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "one");
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
}

void ProcessXYZ::viewModel() {
    this->viewModel(this->cloud);
}

void ProcessXYZ::saveModelAsPLY(string filepath) {
    this->saveModelAsPLY(this->cloud, filepath);
}

void ProcessXYZ::saveModelAsPLY(pcl::PointCloud<pcl::PointXYZ> cloud, string filepath) {
    pcl::PLYWriter w;
    w.write(filepath, cloud);
    cout << "Saved model at " << filepath << endl;
}

void ProcessXYZ::setOBJFolder(string fpath) {
    this->objFolder = fpath;
}

void ProcessXYZ::setPLYFolder(string fpath) {
    this->plyFolder = fpath;
}

MyMesh& ProcessXYZ::getMesh() {
    return this->mesh;
}

void ProcessXYZ::ransacTest(MyMesh & mesh) {
//    cout << "Mesh has " << mesh.VN() << " vertices, " << mesh.FN() << " faces and " << mesh.EN() << " edges." << endl;
//    vcg::tri::UpdateBounding<MyMesh>::Box(mesh);
//
//    vcg::RansacFramework<MyMesh, vcg::BaseFeatureSet<MyMesh> > ran;
//    vcg::RansacFramework<MyMesh, vcg::BaseFeatureSet<MyMesh> >::Param params;
//    vcg::BaseFeatureSet<MyMesh>::Param bParams;
//    params.samplingRadiusPerc = 0.008;
//    params.evalSize = 50;
//    params.inlierRatioThr = 0.5;
//    params.iterMax = 400;
//    params.maxMatchingFeatureNum = 500;
//    bParams.featureSampleRatio = 0.5;
//    MyMesh mesh2;
//    this->importOBJAsMesh(this->fname, mesh2);
//    ran.Init(this->mesh, mesh2, params, bParams);
//    cout << "Ran Init" << endl;
}

void ProcessXYZ::createBoundingBox(MyMesh & mesh) {
    vcg::Box3<float> boundingBox;
    
}

void ProcessXYZ::printVertexLocation(MyVertex & vertex) {
    
}

//template <class MESH>
bool ProcessXYZ::normalTest(typename vcg::face::Pos<MyMesh::FaceType> pos) {
    MyMesh::ScalarType thr = 0.0f;
    MyMesh::CoordType NdP = vcg::TriangleNormal<MyMesh::FaceType>(*pos.f);
    MyMesh::CoordType tmp, oop, soglia = MyMesh::CoordType(thr, thr, thr);
    vcg::face::Pos<MyMesh::FaceType> aux = pos;
    do {
        aux.FlipF();
        aux.FlipE();
        oop = vcg::Abs(tmp - ::vcg::TriangleNormal<MyMesh::FaceType>(*pos.f));
        if (oop < soglia)
            return false;
    } while ((aux != pos) && (!aux.IsBorder()));
    return true;
}

void ProcessXYZ::holeFillTrivialEar(MyMesh & mesh) {
    int holeSize = 5;

    //Update face-face topology
    vcg::tri::UpdateTopology<MyMesh>::FaceFace(mesh);

    vcg::tri::UpdateNormal<MyMesh>::PerVertexClear(mesh);
    vcg::tri::UpdateNormal<MyMesh>::PerVertex(mesh);
    // THIS IS NOT WOTKING ... Unable to UpdateNormal using Faces.
    // vcg::tri::UpdateNormal<MyMesh>::PerFace(mesh);
    // vcg::tri::UpdateNormal<MyMesh>::PerVertexPerFace(mesh);

    vcg::tri::UpdateFlags<MyMesh>::FaceBorderFromFF(mesh);
    assert(vcg::tri::Clean<MyMesh>::IsFFAdjacencyConsistent(mesh));
    
    //Compute the average of face areas
    float avg, sumA = 0.0f;
    int numA = 0, indices;
    indices = mesh.face.size();
    MyMesh::FaceIterator fi;
    for (fi = mesh.face.begin(); fi != mesh.face.end(); ++fi) {
        sumA += vcg::DoubleArea(*fi)/2;
        numA++;
        for (int ind=0; ind<3; ++ind) {
            fi->V(ind)->InitIMark();
        }
    }
    avg = sumA / numA;

    cout << "Average: " << avg << endl;


    //Algo
    vcg::tri::Hole<MyMesh>::EarCuttingIntersectionFill<vcg::tri::SelfIntersectionEar<MyMesh> >(mesh, holeSize, false);
    vcg::tri::UpdateFlags<MyMesh>::FaceBorderFromFF(mesh);
    assert(vcg::tri::Clean<MyMesh>::IsFFAdjacencyConsistent(mesh));
    
    //Start refining
    MyMesh::VertexIterator vi;
    MyMesh::FaceIterator f;
    vector<MyMesh::FacePointer> vf;
    f = mesh.face.begin();
    f += indices;
    for (; f != mesh.face.end(); ++f) {
        if (!f->IsD()) {
            f->SetS();
        }
    }

    vector<MyMesh::FacePointer *> FPP;
    vector<MyMesh::FacePointer> added;
    vector<MyMesh::FacePointer>::iterator vfit;
    int i = 1;
    for (f = mesh.face.begin(); f != mesh.face.end(); ++f) {
        if (!(*f).IsD()) {
            if (f->IsS()) {
                f->V(0)->IsW();
                f->V(1)->IsW();
                f->V(2)->IsW();
            }
            else {
                f->V(0)->ClearW();
                f->V(1)->ClearW();
                f->V(2)->ClearW();
            }
        }
    }
    
    vcg::BaseParameterClass pp;
    vcg::LocalOptimization<MyMesh> Fs(mesh, &pp);
    Fs.SetTargetMetric(0.0f);
    Fs.Init<MyDelaunayFlip>();
    Fs.DoOptimization();
    
    do {
        // not working 
        vf.clear();
        f = mesh.face.begin();
        f += indices;
        for (; f != mesh.face.end(); ++f) {
            if (f->IsS()) {
                bool test = true;
                for (int ind = 0; ind < 3; ++ind) {
                    f->V(ind)->InitIMark();
                }
                test = (vcg::DoubleArea<MyMesh::FaceType>(*f)/2) > avg;
                if (test)
                    vf.push_back(&(*f));
            }
        }
        printf("\r Refining [%d] -> %d", i, int(vf.size()));
        i++;
        
        FPP.clear();
        added.clear();
        for (vfit = vf.begin(); vfit != vf.end(); ++vfit) {
            FPP.push_back(&(*vfit));
        }
        int toAdd = vf.size();
        MyMesh::FaceIterator f1, f2;
        f2 = vcg::tri::Allocator<MyMesh>::AddFaces(mesh, toAdd*2, FPP);
        MyMesh::VertexIterator vertp = vcg::tri::Allocator<MyMesh>::AddVertices(mesh, toAdd);
        vector<MyMesh::FacePointer> added;
        added.reserve(toAdd);
        vfit = vf.begin();
        
        for (int i=0; i<toAdd; ++i, f2++, vertp++) {
            f1 = f2;
            f2++;
            TriSplit<MyMesh, CenterPointBarycenter<MyMesh> >::Apply(vf[i], &(*f1), &(*f2), &(*vertp), CenterPointBarycenter<MyMesh>());
            f1->SetS();
            f2->SetS();
            for (int itr=0; itr<3; itr++) {
                f1->V(itr)->SetW();
                f2->V(itr)->SetW();
            }
            added.push_back(&(*f1));
            added.push_back(&(*f2));
        }
        vcg::BaseParameterClass pp;
        vcg::LocalOptimization<MyMesh> FlippingSession(mesh, &pp);
        FlippingSession.SetTargetMetric(0.0f);
        FlippingSession.Init<MyDelaunayFlip>();
        FlippingSession.DoOptimization();
    } while (!vf.empty());
    
    vcg::LocalOptimization<MyMesh> Fiss(mesh, &pp);
    Fiss.SetTargetMetric(0.0f);
    Fiss.Init<MyDelaunayFlip>();
    Fiss.DoOptimization();
    
//    vcg::tri::io::ExporterOBJ<MyMesh>::Save(mesh, "trivialEar.obj", false);
    int UBIT = MyMesh::VertexType::NewBitFlag();
    f = mesh.face.begin();
    f += indices;
    for (; f != mesh.face.end(); ++f) {
        if (f->IsS()) {
            for (int ind=0; ind<3; ++ind) {
                if (this->normalTest(vcg::face::Pos<MyMesh::FaceType>(&(*f), ind))) {
                    f->V(ind)->SetUserBit(UBIT);
                }
            }
            f->ClearS();
        }
    }
    for (vi = mesh.vert.begin(); vi != mesh.vert.end(); ++vi) {
        if (!(*vi).IsD()) {
            if (vi->IsUserBit(UBIT)) {
                (*vi).SetS();
                vi->ClearUserBit(UBIT);
            }
        }
    }
    
    vcg::tri::Smooth<MyMesh>::VertexCoordLaplacianBlend(mesh, 1, true);
    printf("Completed. Saving...\n");
    string fpath = this->objFolder + "trivialEar.obj";
    vcg::tri::io::ExporterOBJ<MyMesh>::Save(mesh, fpath.c_str(), false);
}