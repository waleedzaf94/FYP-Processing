#include "CommandInterface.hpp"

bool file_exists(const std::string &name)
{
    std::ifstream f(name.c_str());
    return f.good();
}

static std::string getFileName(const std::string filePath)
{  
    char sep = '/';
    #ifdef _WIN32
    sep = '\\';
    #endif
    size_t i = filePath.rfind(sep, filePath.length());
    if (i != std::string::npos) {
        std::string fn = (filePath.substr(i+1, filePath.length() - i));
        return fn;
    }
   return(filePath);
}

static std::string getFileDirectory(const std::string filePath)
{  
    char sep = '/';
    #ifdef _WIN32
    sep = '\\';
    #endif
    size_t i = filePath.rfind(sep, filePath.length());
    if (i != std::string::npos) {
        std::string fn = (filePath.substr(0,i));
        return fn;
    }
   return("");
}

//TODO Maybe add ifdef _WIN32 for this too?
const std::string ProcessXYZ::DEFAULT_PATH = "./";

void ProcessXYZ::setInput()
{
    if (file_exists(this->inputFile))
    {
        std::string ftype = this->inputFile.substr(this->inputFile.length() -3);
        if (!((ftype == "obj") || (ftype == "off") || (ftype == "ply")))
            throw std::invalid_argument("Invalid File Type. Only PLY, OBJ, OFF are supported.");
        this->fileType = ftype;
        this->inputFileName = getFileName(this->inputFile);
        this->inputFilePath = getFileDirectory(this->inputFile).empty() ? DEFAULT_PATH : getFileDirectory(this->inputFile);
        return;
    }
    std::string err = "Invalid File. File doesn't exist.";
    throw std::invalid_argument(err);
}

void ProcessXYZ::setOutput()
{
    this->outputFilePath = this->ofp.empty() ? DEFAULT_PATH : this->ofp;
    this->outputFileName = this->outputFile.empty() ? this->inputFileName : this->outputFile;
}

std::string ProcessXYZ::getInputFileName()
{
   return this->inputFileName;   
}


std::string ProcessXYZ::getInputFilePath()
{
   return this->inputFilePath;   
}

std::string ProcessXYZ::getOutputFilePath()
{
   return this->outputFilePath;   
}

std::string ProcessXYZ::getOutputFileName()
{
   return this->outputFileName;   
}

void ProcessXYZ::parseFunctions()
{
    if (!this->functions.empty())
    {
        this->auxiliaryCalls = true;
        boost::split(this->functionList, this->functions, boost::is_any_of("_"));
        for (std::vector<std::string>::iterator it = this->functionList.begin(); it != this->functionList.end(); ++it)
        {
            std::cout << "Running function: " << *it << std::endl;
        }
    } 
}

void ProcessXYZ::parseAFArguments() {
    if (!this->afArgs.empty()) {
        std::vector<std::string> args;
        boost::split(args, this->afArgs, boost::is_any_of("_"));
        if (args.size() != 5) return;
        for (std::string arg: args) {
            this->afArgList.push_back(stod(arg));
        }
    }
}

void ProcessXYZ::processModel()
{
    // input file into cgal
    std::string filePath = this->inputFilePath + "/" + this->inputFileName;
    std::cout << filePath <<std::endl;
    cgalProcessor.inputPolyhedron(filePath, this->fileType);
    
    if (this->runAllFlag)
    {
        //  call cgalProcessAll
        std::cout << "running all functions" << std::endl;
        cgalProcessor.advancingFrontWrapper();
        cgalProcessor.shapeDetectionWrapper();
        cgalProcessor.poissonReconstructionWrapper();
    }
    if (this->auxiliaryCalls)
    {
        for (std::string fun: this->functionList) {
            printf("Running function: %s\n", fun.c_str());
            if (fun == "AFR")
                cgalProcessor.advancingFrontWrapper();
            else if (fun == "PSD") {
                std::string f = this->inputFile.substr(0, this->inputFile.length() - 3);
                cgalProcessor.shapeDetectionWrapper(this->ofp + "/" + f + "_PSD.ply");
            }
            else if (fun == "PSR")
                cgalProcessor.poissonReconstructionWrapper();
            else {
                std::cerr << "Invalid function name: " << fun << std::endl;
            }
        }
    }
}

void ProcessXYZ::saveFinalModel()
{
    std::string filePath = this->outputFilePath + "/" + this->outputFileName;
    cgalProcessor.outputPolyhedron(filePath, this->fileType);
}



void runAll() {
    string inputs[18] = {"310B_Info", "312BL", "312BR", "312TL", "312TR", "335 - Old", "335", "Chi_11", "CORD1", "CORD2", "CORDL1_N", "CORDL1", "CORDL2", "CORDM", "CORDR1", "CORDR1R2", "DotNet", "mesh_20180411T2315"};
//    string inputs[2] = {"310B_Info", "mesh_20180411T2315"};
    vector<string> ins, outs;
    string fpath = "/Users/waleedzafar/Projects/FYP/one/models/";
    string outPath = "/Users/waleedzafar/Projects/FYP/one/models/OBJ";
    int len = sizeof(inputs) / sizeof(*inputs);
    for (int i=0; i<len; i++) {
        ins.push_back(fpath + inputs[i] + ".obj");
        outs.push_back(inputs[i] + "_AFR.obj");
}
    for (int i=0; i<len; i++) {
        ProcessXYZ processor;
        printf("Starting %s.obj\n", inputs[i].c_str());
        processor.inputFile = ins[i];
        processor.outputFile = outs[i];
        processor.ofp = outPath;
        processor.runAllFlag = false;
//        processor.functions = "PSD";
        processor.functions = "AFR_PSD";
        processor.setInput();
        processor.setOutput();
        processor.parseFunctions();
        processor.parseAFArguments();
        processor.processModel();
        processor.saveFinalModel();
        printf("Completed %s.obj\n", inputs[i].c_str());
    }
}

int main(int argc, char **argv)
{
    runAll();
    return 0;
    bool help;
    ProcessXYZ processor;
    
    Flags flags;
    flags.Var(processor.inputFile, 'f', "inputPath", std::string(""), "Path to the input file", "File");
    flags.Var(processor.outputFile, 'o', "outputFile", std::string(""), "Output File Name", "File");
    flags.Var(processor.ofp, 'p', "outputFilePath", std::string(""), "Output file directory", "File");
    flags.Var(processor.functions, 't', "transform", std::string(""), "Run specific transformations on the input file submit in order (separated by '_'). Options Include Advancing Front (AFR), Pointset Shape Detection (PSD), Poisson Surface Reconstruction (PSR)... etc usage example: -t AFR_PSD_PSR. With the -a flag, transformations will run after all the preconfigured mesh operations finish", "Functions");
    flags.Bool(processor.runAllFlag, 'a', "all", "Run the entire mesh generation process", "Functions");
    flags.Bool(help, 'h', "help", "show this help and exit", "Help");
    flags.Var(processor.afArgs, 'b', "advancingFrontArguments", std::string(""), "Parameters for Advancing Front. Probability (0.05), Minimum Points (100), Epsilon (0.005), Cluster Epsilon (0.05), Minimum Threshold (0.8). Input values separated by '_' in order). Usage example: -b 0.05_100_0.005_0.05_0.8.", "FunctionArguments");

    if (!flags.Parse(argc, argv))
    {
        flags.PrintHelp(argv[0]);
        return 1;
    }
    else if (help)
    {
        flags.PrintHelp(argv[0]);
        return 0;
    }
    processor.setInput();
    processor.setOutput();
    processor.parseFunctions();
    processor.parseAFArguments();
    processor.processModel();
    processor.saveFinalModel();
    return 0;
}
