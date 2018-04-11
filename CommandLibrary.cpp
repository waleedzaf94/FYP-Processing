#include "stdafx.h"
#include "CommandLibrary.h"

static bool file_exists(const std::string &name)
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
		std::string fn = (filePath.substr(i + 1, filePath.length() - i));
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
		std::string fn = (filePath.substr(0, i));
		return fn;
	}
	return("");
}

CommandLibrary::~CommandLibrary()
{
}

void CommandLibrary::Hello(void) {
	// Call the Windows API (unmanaged) MessageBox
	std::cout << "Hello World" << std::endl;
}

void CommandLibrary::SaveHello(std::string filepath)
{
	std::ofstream out(filepath);
	if (out.is_open()) {
		out << "Finally Aman!! \n";
		out.close();
	}
}

bool CommandLibrary::HelloBool(void)
{
	return true;
}

const std::string CommandLibrary::DEFAULT_PATH = ".\\";

void CommandLibrary::SetInput(std::string inputFile)
{
	if (file_exists(inputFile))
	{
		this->inputFileName = getFileName(inputFile);
		this->inputFilePath = getFileDirectory(inputFile).empty() ? DEFAULT_PATH : getFileDirectory(inputFile);
		this->fileType = inputFile.substr(inputFile.length() - 3);
		return;
		// TODO: Check file type 
		//std::string filet = inputFile.substr(inputFile.length() - 3);
		//if ()
		//{
		//case "obj":
		//	this->fileType = "obj";
		//	return;
		//case "off":
		//	this->fileType = "off";
		//	return;
		//case "ply":
		//	this->fileType = "ply";
		//	return;
		//}
	}
	throw std::invalid_argument("Invalid File");
}

void CommandLibrary::SetOutput(std::string outputFile, std::string ofp)
{
	this->outputFilePath = ofp.empty() ? DEFAULT_PATH : ofp;
	this->outputFileName = outputFile.empty() ? this->inputFileName : outputFile;
}

std::string CommandLibrary::GetInputFileName()
{
	return inputFileName;
}

std::string CommandLibrary::GetOutputFileName()
{
	return outputFileName;
}

std::string CommandLibrary::GetInputFilePath()
{
	return inputFilePath;
}

std::string CommandLibrary::GetOutputFilePath()
{
	return outputFilePath;
}


void CommandLibrary::ParseFunctions(std::string functions)
{
	// Using std library
	std::stringstream functionsStream(functions);
	std::string segment;
	while (std::getline(functionsStream, segment, '_'))
	{
		functionList.push_back(segment);
	}
	// Using boost library
	// if (!functions.empty())
	// {
	//     this->auxiliaryCalls = true;
	//     boost::split(this->functionList, this->functions, boost::is_any_of("_"));
	//     for (std::vector<std::string>::iterator it = this->functionList.begin(); it != this->functionList.end(); ++it)
	//     {
	//         std::cout << "Running function: " << *it << std::endl;
	//     }
	// } 
}

void CommandLibrary::ProcessModel(bool runAllFlag)
{
	// input fileinto cgal
	std::string filePath = this->inputFilePath + "\\" + this->inputFileName;
	std::cout << filePath << std::endl;
	//cgalProcessor.inputPolyhedron(filePath, this->fileType);
	if (runAllFlag)
	{
		//  call cgalProcessAll
		std::cout << "running all functions" << std::endl;
		//cgalProcessor.AdvancingFrontWrapper();
		//cgalProcessor.ShapeDetectionWrapper();
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

void CommandLibrary::SaveFinalModel(void)
{
	std::string filePath = this->outputFilePath + "/" + this->outputFileName;
	//cgalProcessor.outputPolyhedron(filePath, this->fileType);
}
