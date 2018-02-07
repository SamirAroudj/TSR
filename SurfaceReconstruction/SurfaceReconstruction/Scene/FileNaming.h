/*
* Copyright (C) 2017 by Author: Aroudj, Samir
* TU Darmstadt - Graphics, Capture and Massively Parallel Computing
* All rights reserved.
*
* This software may be modified and distributed under the terms
* of the BSD 3-Clause license. See the License.txt file for details.
*/

#ifndef _FILE_NAMING_H_
#define _FILE_NAMING_H_

namespace SurfaceReconstruction
{
	class FileNaming
	{
	public:
		
		// file endings
		static const char *ENDING_COLOR_IMAGE;
		static const char *ENDING_MESH;
		static const char *ENDING_MVE_IMAGE;
		static const char *ENDING_OCCUPANCY;
		static const char *ENDING_OCTREE;
		static const char *ENDING_PLY;
		static const char *ENDING_SAMPLES;
		static const char *ENDING_VIEWS;
		
		// file name parts
		static const char *FILTERED_SAMPLES;
		static const char *REORDERED_SAMPLES;
		static const char *RESULTS_BEGINNING;
		static const char *RESULTS_FOLDER;

		// image tags for file naming
		static const char *IMAGE_TAG_COLOR;
		static const char *IMAGE_TAG_COLOR_S0;
		static const char *IMAGE_TAG_DEPTH;
	};
}

#endif // _FILE_NAMING_H_