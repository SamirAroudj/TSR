/*
 * Copyright (C) 2017 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */
#include "Platform/ResourceManagement/MemoryManager.h"
#include "SurfaceKernelColoring/SurfaceKernelColoring.h"
#include "Utilities/HelperFunctions.h"

using namespace Platform;
using namespace std;

#ifdef _WINDOWS
	int32 WINAPI WinMain(HINSTANCE applicationHandle, HINSTANCE unused, LPSTR commandLineArguments, int32 windowShowState)
	{
#else
	int main(int argumentCount, const char *commandLineArguments[])
	{
#endif // _WINDOWS

	// do cool stuff
	{
		vector<string> arguments;
		#ifdef _WINDOWS
			Utilities::getCommandLineArguments(arguments, commandLineArguments);	
		#else
			Utilities::getCommandLineArguments(arguments, commandLineArguments, argumentCount);
		#endif // _WINDOWS
			
		SurfaceKernelColoring application
		(
			#ifdef _WINDOWS
				applicationHandle,
			#endif // _WINDOWS
			arguments
		);
	
		application.run();
	}

	#ifdef MEMORY_MANAGEMENT
		ResourceManagement::MemoryManager::shutDown();
	#endif /// MEMORY_MANAGEMENT
	return 0;
}
