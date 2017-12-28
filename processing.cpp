//Standard
#include <iostream>
#include <stdio.h>
#include <string>
#include <vector>
#include <processing.h>

using namespace std;

int main() {
    #ifndef amanDev
        string fname = "/Users/waleedzafar/projects/fyp/one/335.obj";
    #else
        string fname = "/home/aman/Desktop/FYP-Processing/models/335.obj";
    #endif
    //cout << "Input filename: ";
    //cin >> fname;
    ProcessXYZ processor;
    processor.processModel(fname);
    return 0;
}

void ProcessXYZ::processModel(string filename) {
      this->vcgProcessor.importOBJAsMesh(filename);
      this->vcgProcessor.performProcess();
//    this->holeFillTrivialEar(this->mesh);
//    this->ransacTest(this->mesh);
//    this->vcgProcessor.saveMeshAsOBJ(this->mesh, this->objFolder + "ransacTest.obj");
//    this->saveMeshAsOBJ(this->mesh, this->objFolder + "meshOrig.obj");
//      this->pclProcessor.importOBJAsPSD(filename);
//    this->viewModel();
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr = this->cloud.makeShared();
//    this->saveModelAsPLY(this->plyFolder + "orig.ply");
//    this->statisticalOutlierRemoval(cloudPtr);
//    this->radiusOutlierRemoval(cloudPtr);
//    this->conditionalOutlierRemoval(cloudPtr);
}