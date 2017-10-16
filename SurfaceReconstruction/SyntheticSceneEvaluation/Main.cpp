/*
 * Copyright (C) 2017 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */
#include "Evaluator.h"
#include "Platform/ResourceManagement/MemoryManager.h"

int main(int argc, char *argv[])
{
	#ifdef _WINDOWS
		#ifdef _DEBUG
			_CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);
		#endif // _DEBUG
	#endif // _WINDOWS
	

	Evaluator evaluator(argv[1], argv[2]);
	evaluator.saveResults(argv[3]);

	#ifdef MEMORY_MANAGEMENT
		ResourceManagement::MemoryManager::shutDown();
	#endif /// MEMORY_MANAGEMENT

	return 0;
}
