#include <iostream>
#include <stdio.h>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common_headers.h>
#include <boost/thread/thread.hpp>

using namespace std;

class ProcessXYZ {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    const pcl::PointCloud<pcl::PointXYZ> *cloudPtr = &cloud;
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloudCPtr;
    string plyFolder = "/Users/waleedzafar/projects/fyp/one/models/PLY/";
private:
    void importOBJModel(string);
    void saveModelAsPLY(string);
public:
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
    this->importOBJModel(filename);
//    this->viewModel();
    this->saveModelAsPLY(this->plyFolder + "orig.ply");
    cout << "Saved Model\n";
}

void ProcessXYZ::importOBJModel(string filename) {
    int imp = pcl::io::loadOBJFile(filename, this->cloud);
    if (imp == 0) {
        cout << "Loaded file at " + filename << endl;
    }
}

void ProcessXYZ::viewModel(pcl::PointCloud<pcl::PointXYZ> cloud) {
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ> (pcl::PointCloud<pcl::PointXYZ>::ConstPtr(this->cloudPtr), "one");
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

















