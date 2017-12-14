#include <iostream>
#include <stdio.h>
#include <string>
#include <vector>
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
#include <vcg/complex/complex.h>
#include <wrap/io_trimesh/import.h>

using namespace std;

class MyVertex;
class MyEdge;
class MyFace;

struct MyUsedTypes : public vcg::UsedTypes<vcg::Use<MyVertex> ::AsVertexType, vcg::Use<MyEdge> ::AsEdgeType, vcg::Use<MyFace> ::AsFaceType> {};

class MyVertex : public vcg::Vertex<MyUsedTypes, vcg::vertex::Coord3f, vcg::vertex::Normal3f, vcg::vertex::BitFlags> {};

class MyEdge : public vcg::Edge<MyUsedTypes> {};

class MyFace : public vcg::Face<MyUsedTypes, vcg::face::FFAdj, vcg::face::VertexRef, vcg::face::BitFlags> {};

class MyMesh : public vcg::tri::TriMesh<std::vector<MyVertex>, std::vector<MyFace>, std::vector<MyEdge> > {};

class ProcessXYZ {
private:
    //Members
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr;
    string plyFolder = "/Users/waleedzafar/projects/fyp/one/models/PLY/";
    MyMesh mesh;
    
    //Functions
    void importOBJAsPSD(string);
    void importOBJAsMesh(string);
    void saveModelAsPLY(string);
    void statisticalOutlierRemoval(pcl::PointCloud<pcl::PointXYZ>::ConstPtr);
    void radiusOutlierRemoval(pcl::PointCloud<pcl::PointXYZ>::ConstPtr);
    void conditionalOutlierRemoval(pcl::PointCloud<pcl::PointXYZ>::ConstPtr);
public:
    //Functions
    void viewModel();
    void viewModel(pcl::PointCloud<pcl::PointXYZ>);
    void processModel(string);
    void saveModelAsPLY(pcl::PointCloud<pcl::PointXYZ>, string);
};

int main() {
    string fname = "/Users/waleedzafar/projects/fyp/one/335.obj";
    //cout << "Input filename: ";
    //cin >> fname;
    ProcessXYZ processing;
    processing.processModel(fname);
    return 0;
}

void ProcessXYZ::processModel(string filename) {
    this->importOBJAsMesh(filename);
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

void ProcessXYZ::importOBJAsPSD(string filename) {
    int imp = pcl::io::loadOBJFile(filename, this->cloud);
    this->cloudPtr = this->cloud.makeShared();
    if (imp == 0)
        cout << "Imported file at " << filename << endl;
}

void ProcessXYZ::importOBJAsMesh(string filename) {
    int zero;
    if (vcg::tri::io::ImporterOBJ<MyMesh>::Open(this->mesh, filename.c_str(), zero) == vcg::tri::io::ImporterOBJ<MyMesh>::E_NOERROR) {
        cout << "Imported OBJ as Mesh from " << filename << endl;
    }
    else {
        cout << "Error importing file: " << filename << endl;
    }
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

















