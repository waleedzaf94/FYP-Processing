#include "Flags.hh"

#include <cstdint>
#include <iostream>
#include <sys/stat.h>
#include <unistd.h>
#include <fstream>
#include <vector>
#include <sstream>
#include <boost/algorithm/string.hpp>
#include "cgalProcessing.h"
#include "processingIO.hpp"
#include <stdexcept>



class ProcessXYZ {

private:
    std::string inputFilePath;
    std::string inputFileName;
    std::string outputFilePath;
    std::string outputFileName;
    const static std::string DEFAULT_PATH;
    std::vector<std::string> functionList;
    bool auxiliaryCalls;
    CGALProcessing cgalProcessor;

public:
    ProcessXYZ() {
        auxiliaryCalls = false;
    };
    void SetInput();
    void SetOutput();
    std::string GetInputFileName();
    std::string GetOutputFileName();
    std::string GetInputFilePath();    
    std::string GetOutputFilePath();
    std::string GetFileName();
    void SetFileName();
    void SetOutputFilePath();
    void ParseFunctions();
    void ProcessModel();
    void SaveFinalModel();

    std::string inputFile;
    std::string functions;
    std::string outputFile;
    std::string ofp;
    bool runAllFlag;
};