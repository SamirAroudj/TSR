/*
* Copyright (C) 2018 by Author: Aroudj, Samir
* TU Darmstadt - Graphics, Capture and Massively Parallel Computing
* All rights reserved.
*
* This software may be modified and distributed under the terms
* of the BSD 3-Clause license. See the License.txt file for details.
*/

#include "SurfaceReconstruction/Scene/FileNaming.h"

using namespace SurfaceReconstruction;

// file beginnings
const char *FileNaming::BEGINNING_RESULTS = "Scene";
const char *FileNaming::BEGINNING_VIEW_FOLDER = "view_";

// file endings
const char *FileNaming::ENDING_COLOR_IMAGE = ".png";
const char *FileNaming::ENDING_DUAL_CELLS = ".dualCells";
const char *FileNaming::ENDING_LEAVES = ".leaves";
const char *FileNaming::ENDING_MESH = ".mesh";
const char *FileNaming::ENDING_MVE_IMAGE = ".mvei";
const char *FileNaming::ENDING_NODES = ".nodes";
const char *FileNaming::ENDING_OCCUPANCY = ".occupancy";
const char *FileNaming::ENDING_OCTREE = ".octree";
const char *FileNaming::ENDING_PLY = ".ply";
const char *FileNaming::ENDING_SAMPLES = ".samples";
const char *FileNaming::ENDING_VIEWS = ".views";
const char *FileNaming::ENDING_VIEW_FOLDER = ".mve";

// other file names
const char *FileNaming::FILTERED_SAMPLES = "Filtered";
const char *FileNaming::RECONSTRUCTED_MESHES[IReconstructorObserver::RECONSTRUCTION_TYPE_COUNT] =
{
	"Crust", 
	"ReconstructionViaPhotos",
	"ReconstructionViaSamples"
};
const char *FileNaming::REORDERED_SAMPLES = "Reordered";
const char *FileNaming::RESULTS_FOLDER = "Results";

// image tags
const char *FileNaming::IMAGE_TAG_COLOR = "undist";
const char *FileNaming::IMAGE_TAG_COLOR_S0 = "undistorted";
const char *FileNaming::IMAGE_TAG_DEPTH = "depth";