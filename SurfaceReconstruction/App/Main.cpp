/*
 * Copyright (C) 2018 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */
#include "App/TSR.h"
#include "Platform/ResourceManagement/MemoryManager.h"
#include "Utilities/HelperFunctions.h"

using namespace Platform;
using namespace std;

#ifdef MEMORY_MANAGEMENT
	const uint32 ResourceManagement::DEFAULT_POOL_BUCKET_NUMBER = 5;
	const uint16 ResourceManagement::DEFAULT_POOL_BUCKET_CAPACITIES[DEFAULT_POOL_BUCKET_NUMBER] = { 1024, 1024, 1024, 1024, 1024 };
	const uint16 ResourceManagement::DEFAULT_POOL_BUCKET_GRANULARITIES[DEFAULT_POOL_BUCKET_NUMBER] = { 16, 32, 64, 128, 256 };
#endif /// MEMORY_MANAGEMENT

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
			
		TSR application
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
