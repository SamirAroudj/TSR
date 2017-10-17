/*
 * Copyright (C) 2017 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */
#include "Graphics/Camera3D.h"
#include "MagicConstants.h"
#include "Platform/ResourceManagement/MemoryManager.h"
#include "SurfaceReconstruction/Image/Image.h"
#include "SurfaceReconstruction/Refinement/FSSFRefiner.h"
#include "SurfaceReconstruction/Refinement/FSSFStatistics.h"
#include "SurfaceReconstruction/Refinement/MeshRefiner.h"
#ifdef PCS_REFINEMENT
	#include "SurfaceReconstruction/Refinement/PCSRefiner.h"
#endif // PCS_REFINEMENT
#include "SurfaceReconstruction/Scene/Samples.h"
#include "SurfaceReconstruction/Scene/Scene.h"
#include "SurfaceReconstruction/Scene/Tree/Tree.h"
#include "SurfaceReconstruction/Scene/SyntheticScene.h"
#include "SurfaceReconstruction/SurfaceExtraction/DualMarchingCells.h"
#include "SurfaceReconstruction/SurfaceExtraction/Occupancy.h"

using namespace Graphics;
using namespace ResourceManagement;
using namespace std;
using namespace SurfaceReconstruction;

#ifdef MEMORY_MANAGEMENT
	const uint32 ResourceManagement::DEFAULT_POOL_BUCKET_NUMBER = 5;
	const uint16 ResourceManagement::DEFAULT_POOL_BUCKET_CAPACITIES[DEFAULT_POOL_BUCKET_NUMBER] = { 1024, 1024, 1024, 1024, 1024 };
	const uint16 ResourceManagement::DEFAULT_POOL_BUCKET_GRANULARITIES[DEFAULT_POOL_BUCKET_NUMBER] = { 16, 32, 64, 128, 256 };
#endif /// MEMORY_MANAGEMENT

// Rendering colors
const Color BACK_BUFFER_CLEAR_COLOR(0.0f, 0.0f, 0.0f, 1.0f);
const Color COLOR_HIGHLIGHTED(1.0f, 1.0f, 1.0f, 1.0f);
const Color COLOR_SAMPLES[3] =	{
									Color(0.25f, 1.0f,  0.15f, 0.7f),
									Color(0.7f,  0.45f, 0.25f, 0.7f),
									Color(1.0f,  1.0f,  0.0f,  1.0f)
								};
const Color COLOR_SURFACES[3] =	{
									Color(0.0f, 1.0f, 1.0f, 0.5f),
									Color(0.5f, 0.5f, 0.5f, 0.5f),
									Color(1.0f, 0.95f, 0.25f, 0.5f)
								};
const Color COLOR_VIEW(0.9f, 0.025f, 0.9f, 1.0f);

// camera parameters
const Real CAMERA_ZOOM_VELOCITY = (Real) 0.5;

// DualMarchingCells
const Color DualMarchingCells::SURFACE_COLOR(0.79f, 0.69f, 0.1f, 0.0f);

// FSSFRefiner
const uint32 FSSFRefiner::MEMORY_ALLOCATION_FACTOR = 0x1 << 4;
const uint32 FSSFRefiner::OMP_PAIR_BATCH_COUNT = 0x1 << 15;
const uint32 FSSFRefiner::OMP_PAIR_BATCH_SIZE = 0x1 << 7;
const uint32 FSSFRefiner::EMBREE_PAIR_BATCH_SIZE = FSSFRefiner::OMP_PAIR_BATCH_COUNT * FSSFRefiner::OMP_PAIR_BATCH_SIZE;
const uint32 FSSFRefiner::EMBREE_RAY_BATCH_SIZE = 0x1 << 16;

// Image
template <>
uint32 VolatileResource<Image>::msMaximumNumber = 0x1 << 10;

// Occupancy
const uint32 Occupancy::OMP_PRIOR_LEAF_BATCH_SIZE = 0x1 << 9;
const uint32 Occupancy::OMP_SAMPLE_BATCH_SIZE = 0x1 << 7;
const uint32 Occupancy::OMP_VIEW_CONE_BATCH_SIZE = 0x1 << 7;

#ifdef PCS_REFINEMENT
	// PCSRefiner
	const Real PCSRefiner::GAUSSIAN_STD_DEV = (Real) 2.0;
	const Real PCSRefiner::HIGH_ERROR = (Real) 10e25;
	const Real PCSRefiner::RELATIVE_STEP_SIZE = (Real) 0.02;
	const Real PCSRefiner::UMBRELLA_SMOOTHING_LAMBDA = (Real) 0.05;

	const uint32 PCSRefiner::ERROR_WINDOW_SIZE = 10;
	const uint32 PCSRefiner::IMAGE_SUBSET_SIZE = 10;
	const uint32 PCSRefiner::ITERATION_COUNT = 1000;
	const uint32 PCSRefiner::SUBDIVISION_PIXEL_TRHESHOLD = 16;

	// PatchStatistics
	const Real PatchStatistics::BAYESIAN_REGULARIZATION_TERM = (Real) 9.0 / (255 * 255);
#endif // PCS_REFINEMENT

// Renderer
const Real Renderer::NORMAL_SIZE = (Real) 0.2;
const Real Renderer::VIEW_SIZE = (Real) 0.1; 

// Scene
const uint32 Scene::REFINEMENT_VIA_PHOTOS_MESH_OUTPUT_FREQUENCY = 25;

// SyntheticScene
const Real SyntheticScene::DEPTH_DIFFERENCE_FACTOR = 5.0f;
const uint32 SyntheticScene::RANDOM_SEED = 0;
