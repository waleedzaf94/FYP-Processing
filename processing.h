//#include "vcgProcessing.cpp"
//#include "pclProcessing.cpp"
//#include "cgalProcessing.cpp"
#include "vcgProcessing.h"
#include "pclProcessing.h"
#include "cgalProcessing.h"


class ProcessXYZ {
private:
    #ifdef amanDev
    string plyFolder = "/home/aman/Desktop/FYP-Processing/models/PLY";
    string objFolder = "/home/aman/Desktop/FYP-Processing/models/OBJ";
    string fname = "/home/aman/Desktop/FYP-Processing/335.obj";
    #else
    string plyFolder = "/Users/waleedzafar/projects/fyp/one/models/PLY/";
    string objFolder = "/Users/waleedzafar/projects/fyp/one/models/OBJ/";
    string fname = "/Users/waleedzafar/projects/fyp/one/335.obj";
    #endif
    string modelFName = "335";
    struct vertexInfo {
        float x, y, z;
    };
    struct planeInfo {
        float x, y, z, d;
    };
    struct modelInfo {
        planeInfo floor;
        planeInfo roof;
        std::vector<planeInfo> walls;
        std::vector<vertexInfo> corners;
    };
    

public:
    void processModel(std::string);
    VCGProcessing vcgProcessor;
    PCLProcessing pclProcessor;
    CGALProcessing cgalProcessor;
};
