
#include "processing.h"
#define amanDev
using namespace std;

int main() {
    #ifndef amanDev
        string fname = "/Users/waleedzafar/projects/fyp/one/models/335_Blob.ply";
    #else
        string fname = "/home/aman/Desktop/FYP-Processing/models/335_Blob.ply";
    #endif
    ProcessXYZ processor;
    processor.processModel(fname);
    return 0;
}

void ProcessXYZ::processModel(string filename) {
//      this->vcgProcessor.importOBJAsMesh(filename);
//      this->vcgProcessor.performProcess();
//    this->holeFillTrivialEar(this->mesh);
//    this->ransacTest(this->mesh);
//    this->vcgProcessor.saveMeshAsOBJ(this->mesh, this->objFolder + "ransacTest.obj");
//    this->saveMeshAsOBJ(this->mesh, this->objFolder + "meshOrig.obj");
//    this->pclProcessor.importOBJAsPSD(filename);
//    this->pclProcessor.performProcess();
//    this->cgalProcessor.testOBJ();
//    this->cgalProcessor.shapeDetection();
//    this->cgalProcessor.readPlyFile(filename);
//    modelInfo m = readPlyFile(filename);
//    printf("Faces: %lu, Vertices: %lu, Normals: %lu\n", m.faces.size(), m.vertices.size(), m.normals.size());
    
    CGALProcessing::PointVector points;
    vector<vector<size_t> > faces;
    this->cgalProcessor.inputTest(filename);
    
//    this->viewModel();
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr = this->cloud.makeShared();
//    this->saveModelAsPLY(this->plyFolder + "orig.ply");
//    this->statisticalOutlierRemoval(cloudPtr);
//    this->radiusOutlierRemoval(cloudPtr);
//    this->conditionalOutlierRemoval(cloudPtr);
}
