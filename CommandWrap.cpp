// This is the main DLL file.

#include "stdafx.h"

#include "CommandLibrary.h"
#include "CommandWrap.h"


static std::string ToStdString(System::String^ str)
{
	return msclr::interop::marshal_as<std::string>(str);
};

void Processing::CommandWrap::Hello()
{
	// Call the numanged function
	pu->Hello();
}

void Processing::CommandWrap::SaveHello(System::String ^ filepath)
{
	std::string standardString = ToStdString(filepath);
	pu->SaveHello(standardString);
}

bool Processing::CommandWrap::HelloBool(void)
{
	return pu->HelloBool();
}

void Processing::CommandWrap::SetInput(System::String ^ filepath)
{
	std::string standardString = ToStdString(filepath);
	pu->SetInput(standardString);
}

void Processing::CommandWrap::SetOutput(System::String ^ filename, System::String ^ filepath)
{
	std::string fn = ToStdString(filename);
	std::string ofp = ToStdString(filepath);
	pu->SetOutput(fn, ofp);
}

void Processing::CommandWrap::ProcessModel(bool runAllFlag, System::String ^ functions)
{
	std::string func = ToStdString(functions);
	pu->ParseFunctions(func);
	pu->ProcessModel(runAllFlag);
}

bool Processing::CommandWrap::SaveFinalModel(void)
{
	pu->SaveFinalModel();
	return true;
}
