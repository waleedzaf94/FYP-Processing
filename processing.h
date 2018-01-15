#include "vcgProcessing.cpp"
// #include "pclProcessing.cpp"
#include "cgalProcessing.cpp"

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

public:
    void processModel(string);
    VCGProcessing vcgProcessor;
    // PCLProcessing pclProcessor;
    CGALProcessing cgalProcessor; 
};