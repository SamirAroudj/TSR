/*
 * Copyright (C) 2018 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */

#include "SurfaceReconstruction/Geometry/Surfel.h"

using namespace Math;
using namespace std;
using namespace SurfaceReconstruction;

Surfel &Surfel::operator =(const Surfel &rhs)
{
	if (this == &rhs)
		return *this;

	mNormal = rhs.mNormal;
	mPosition = rhs.mPosition;

	mBaryCoords[0] = rhs.mBaryCoords[0];
	mBaryCoords[1] = rhs.mBaryCoords[1];

	mTriangleIdx = rhs.mTriangleIdx;

	return *this;
}