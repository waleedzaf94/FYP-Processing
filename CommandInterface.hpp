#ifndef CommandInterface_hpp
#define CommandInterface_hpp
#include "cgalProcessing.h"

#include "Flags.hh"
#include <cstdint>
#include <iostream>
#include <sys/stat.h>
#include <unistd.h>
#include <fstream>
#include <vector>
#include <sstream>
#include <boost/algorithm/string.hpp>
#include <stdexcept>


class ProcessXYZ {
private:
    std::string inputFilePath;
    std::string inputFileName;
    std::string outputFilePath;
    std::string outputFileName;
    const static std::string DEFAULT_PATH;
    std::vector<std::string> functionList;
    std::vector<double> afArgList;
    bool auxiliaryCalls;
    CGALProcessing cgalProcessor;
    std::string fileType;

public:
    ProcessXYZ() {
        auxiliaryCalls = false;
    };
    void setInput();
    void setOutput();
    std::string getInputFileName();
    std::string getOutputFileName();
    std::string getInputFilePath();
    std::string getOutputFilePath();
    void setFileName();
    void setOutputFilePath();
    void parseFunctions();
    void processModel();
    void saveFinalModel();
    void parseAFArguments();

    std::string inputFile;
    std::string functions;
    std::string outputFile;
    std::string ofp;
    std::string afArgs;
    bool runAllFlag;
};

#endif
