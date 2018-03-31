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
void ProcessXYZ::SetInput()
{
    if (file_exists(this->inputFile))
    {
        if (this->inputFile.substr(this->inputFile.length() - 3) == "obj")
        {
            this->inputFileName = getFileName(this->inputFile);
            this->inputFilePath = getFileDirectory(this->inputFile).empty() ? DEFAULT_PATH : getFileDirectory(this->inputFile);
            return;
        }
    }
    throw std::invalid_argument( "Invalid File" );
}

void ProcessXYZ::SetOutput()
{
    this->outputFilePath = this->ofp.empty() ? DEFAULT_PATH : this->ofp;
    this->outputFileName = this->outputFile.empty() ? this->inputFileName : this->outputFile;
}

std::string ProcessXYZ::GetInputFileName()
{
   return this->inputFileName;   
}


std::string ProcessXYZ::GetInputFilePath()
{
   return this->inputFilePath;   
}

std::string ProcessXYZ::GetOutputFilePath()
{
   return this->outputFilePath;   
}

std::string ProcessXYZ::GetOutputFileName()
{
   return this->outputFileName;   
}

void ProcessXYZ::ParseFunctions()
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

void ProcessXYZ::ProcessModel()
{
    // input fileinto cgal
    if (this->runAllFlag)
    {
        //  call cgalProcessAll
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

void ProcessXYZ::SaveFinalModel()
{
    // call cgalSave with OutputFileName and Path;
}

int main(int argc, char **argv)
{
    bool help;
    ProcessXYZ processor;

    Flags flags;
    flags.Var(processor.inputFile, 'f', "inputPath", std::string(""), "Path to the input file", "File");
    flags.Var(processor.outputFile, 'o', "outputFile", std::string(""), "Output File Name", "File");
    flags.Var(processor.ofp, 'p', "outputFilePath", std::string(""), "Output file directory", "File");
    flags.Var(processor.functions, 't', "transform", std::string("Run All"), "Run specific transfomrations on the input file submit in order (separated by '_'). Options Include Advancing Front (AF), Pointset Shape Detection (PSP)... etc usage example: -t AF_PSP. With the -a flag, transformations will run after all the preconfigured mesh operations finish", "Functions");
    flags.Bool(processor.runAllFlag, 'a', "all", "Run the entire mesh generation process", "Functions");
    flags.Bool(help, 'h', "help", "show this help and exit", "Help");

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
    processor.SetInput();
    // std::cout << processor.GetInputFileName() << std::endl;
    // std::cout << processor.GetInputFilePath() << std::endl;
    processor.SetOutput();
    processor.ParseFunctions();
    processor.ProcessModel();
    processor.SaveFinalModel();
    return 0;
}