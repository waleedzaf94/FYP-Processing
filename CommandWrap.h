// UnmanagedWrap.h

#pragma once

using namespace System;

namespace Processing {

	// Class1 is a managed class that can be used bt managed code (such as C#)
	public ref class CommandWrap
	{
	public:
		CommandLibrary *pu;	// pointer to the Unmanaged class

		// the constructor will allocate the pointer pu
		CommandWrap() : pu(new CommandLibrary()) {};

		void Hello();
		void SaveHello(System::String^ filepath);
		bool HelloBool(void);

		void SetInput(System::String^ filepath);
		void SetOutput(System::String^ filename, System::String^ filepath);
		void ProcessModel(bool runAllFlag, System::String^ functions);
		bool SaveFinalModel(void);
		
	};
}
