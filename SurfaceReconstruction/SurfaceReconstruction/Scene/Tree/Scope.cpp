/*
 * Copyright (C) 2018 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */

#include "SurfaceReconstruction/Scene/Tree/Nodes.h"
#include "SurfaceReconstruction/Scene/Tree/Scope.h"

using namespace Math;
using namespace SurfaceReconstruction;


Scope::Scope() :
	mPositionWS(-REAL_MAX, -REAL_MAX, -REAL_MAX),
	mSize(-REAL_MAX),
	mIdx(Nodes::INVALID_INDEX)
{

}