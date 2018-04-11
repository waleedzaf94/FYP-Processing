#pragma once
class CommandLibrary
{
	public:
		CommandLibrary() {
			auxiliaryCalls = false;
		};
		~CommandLibrary();
		void Hello(void);
		bool HelloBool(void);
		void SaveHello(std::string);
		void SetInput(std::string );
		void SetOutput(std::string outputFile, std::string ofp);
		std::string GetInputFileName(void);
		std::string GetOutputFileName(void);
		std::string GetInputFilePath(void);
		std::string GetOutputFilePath(void);
		void ParseFunctions(std::string);
		void ProcessModel(bool);
		void SaveFinalModel(void);


	private:
		std::string inputFilePath;
		std::string inputFileName;
		std::string outputFilePath;
		std::string outputFileName;
		const static std::string DEFAULT_PATH;
		std::vector<std::string> functionList;
		bool auxiliaryCalls;
		//CGALProcessing cgalProcessor;
		std::string fileType;
};
