
//#ifndef PROCESS_XYZ_H
//#define PROCESS_XYZ_H

#include <iostream>
#include <stdio.h>
#include <string>
#include <vector>

#include "vcgProcessing.h"
#include "pclProcessing.h"
#include "cgalProcessing.h"
#include "processingIO.hpp"


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

public:
    
    void processModel(std::string);
    VCGProcessing vcgProcessor;
    PCLProcessing pclProcessor;
    CGALProcessing cgalProcessor;
};

//#endif //PROCESS_XYZ_H


