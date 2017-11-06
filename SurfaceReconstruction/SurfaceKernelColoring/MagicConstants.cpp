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

using namespace Graphics;
#ifdef MEMORY_MANAGEMENT
	using namespace ResourceManagement;
#endif // MEMORY_MANAGEMENT
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

// Renderer
const Real Renderer::NORMAL_SIZE = (Real) 0.2;
const Real Renderer::VIEW_SIZE = (Real) 0.1; 

