/*
 * Copyright (C) 2017 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */

#include "SurfaceReconstruction/SurfaceExtraction/EdgeVertexIndex.h"

using namespace Math;
using namespace SurfaceReconstruction;

EdgeVertexIndex::EdgeVertexIndex(const uint32 leafIdx0, const uint32 leafIdx1)
{
	mLeafIndices[0] = (leafIdx0 < leafIdx1 ? leafIdx0 : leafIdx1);
	mLeafIndices[1] = (leafIdx0 < leafIdx1 ? leafIdx1 : leafIdx0);
}

bool EdgeVertexIndex::operator ==(const EdgeVertexIndex &other) const
{
	return (mLeafIndices[1] == other.mLeafIndices[1]) && 
		   (mLeafIndices[0] == other.mLeafIndices[0]);
}

bool EdgeVertexIndex::operator <(const EdgeVertexIndex &other) const
{
	if (mLeafIndices[1] == other.mLeafIndices[1])
		return mLeafIndices[0] < other.mLeafIndices[0];

	return mLeafIndices[1] < other.mLeafIndices[1];
}