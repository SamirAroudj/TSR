/*
 * Copyright (C) 2017 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */
#ifndef _SURFACE_KERNEL_COLORING_CONSTANTS_H_
#define _SURFACE_KERNEL_COLORING_CONSTANTS_H_

#include <string>
#include "Graphics/Color.h"
#include "Platform/DataTypes.h"
#include "SurfaceReconstruction/Rendering/Renderer.h"

// Color declarations
/** Color which is used to clear the back buffer. */
extern const Graphics::Color BACK_BUFFER_CLEAR_COLOR;

/** Defines the color of emphasized elements to be rendered specially. */
extern const Graphics::Color COLOR_HIGHLIGHTED;

/** Defines the colors with which captured samples are rendered. ([0]: evaluated samples, [1]: unprocessed samples, [2]: Mean Shift samples) */
extern const Graphics::Color COLOR_SAMPLES[3];

/** Defines the colors with which ground truth surfaces are rendered (actually their subsets). */
extern const Graphics::Color COLOR_SURFACES[3];

/** Defines the color with which views are rendered. */
extern const Graphics::Color COLOR_VIEW;

/** Defines the colors used to render implicit function sampling points with negative, neutral and positive evaluation values. (in this order) */
extern const Graphics::Color LEAF_CLUSTER_RESULTS_COLORS[3];

// camera parameters
/** Defines how fast the camera changes zoom.
	Defines how fast an additional scale factor in the orthogonal projection matrix is changed. */
extern const Real CAMERA_ZOOM_VELOCITY;

#endif // _SURFACE_KERNEL_COLORING_CONSTANTS_H_
