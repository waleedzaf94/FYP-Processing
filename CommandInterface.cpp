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

const std::string ProcessXYZ::DEFAULT_PATH = "./";

void ProcessXYZ::setInput()
{
    if (file_exists(this->inputFile))
    {
        if (this->inputFile.substr(this->inputFile.length() - 3) == "obj")
        {
            this->fileType = "obj";
            this->inputFileName = getFileName(this->inputFile);
            this->inputFilePath = getFileDirectory(this->inputFile).empty() ? DEFAULT_PATH : getFileDirectory(this->inputFile);
            return;
        }
        if (this->inputFile.substr(this->inputFile.length() - 3) == "off")
        {
            this->fileType = "off";
            this->inputFileName = getFileName(this->inputFile);
            this->inputFilePath = getFileDirectory(this->inputFile).empty() ? DEFAULT_PATH : getFileDirectory(this->inputFile);
            return;
        }
            if (this->inputFile.substr(this->inputFile.length() - 3) == "ply")
        {
            this->fileType = "ply";
            this->inputFileName = getFileName(this->inputFile);
            this->inputFilePath = getFileDirectory(this->inputFile).empty() ? DEFAULT_PATH : getFileDirectory(this->inputFile);
            return;
        }
    }
    throw std::invalid_argument( "Invalid File" );
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
    // Using std library
    // std::stringstream functionsStream(this->functions);
    // std::string segment;
    // while (std::getline(functionsStream, segment, '_'))
    // {
    //     fList.push_back(segment);
    // }
    // Using boost library
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
    // input fileinto cgal
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
        for (std::vector<std::string>::iterator it = this->functionList.begin(); it != this->functionList.end(); ++it)
        {
            std::cout << "Running function: " << *it << std::endl;
            // Call each function separately
            // switch(*it)
            // {
            //     default:

            // }
        }

    }
    return;
}

void ProcessXYZ::saveFinalModel()
{
    std::string filePath = this->outputFilePath + "/" + this->outputFileName;
    cgalProcessor.outputPolyhedron(filePath, this->fileType);
}

int main(int argc, char **argv)
{
    bool help;
    ProcessXYZ processor;
    
    Flags flags;
    flags.Var(processor.inputFile, 'f', "inputPath", std::string(""), "Path to the input file", "File");
    flags.Var(processor.outputFile, 'o', "outputFile", std::string(""), "Output File Name", "File");
    flags.Var(processor.ofp, 'p', "outputFilePath", std::string(""), "Output file directory", "File");
    flags.Var(processor.functions, 't', "transform", std::string(""), "Run specific transformations on the input file submit in order (separated by '_'). Options Include Advancing Front (AF), Pointset Shape Detection (PSP)... etc usage example: -t AF_PSP. With the -a flag, transformations will run after all the preconfigured mesh operations finish", "Functions");
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
