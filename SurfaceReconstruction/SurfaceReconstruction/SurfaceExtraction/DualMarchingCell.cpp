/*
 * Copyright (C) 2018 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */

#include "Math/MathHelper.h"
#include "SurfaceReconstruction/Scene/Tree/Tree.h"
#include "SurfaceReconstruction/SurfaceExtraction/DualMarchingCell.h"

using namespace Math;
using namespace std;
using namespace SurfaceReconstruction;

DualMarchingCell::DualMarchingCell()
{		
	clear();
}

void DualMarchingCell::clear()
{
	memset(mCornerPositions, 0, sizeof(Vector3) * Nodes::CHILD_COUNT);
	memset(mCornerDistances, 0, sizeof(Real) * Nodes::CHILD_COUNT);
	memset(mCornerScales, 0, sizeof(Real) * Nodes::CHILD_COUNT);
	memset(mCornerLeafIndices, 0, sizeof(uint32) * Nodes::CHILD_COUNT);

	memset(mInterPositions, 0, sizeof(Vector3) * CELL_EDGE_COUNT);
	memset(mInterScales, 0, sizeof(Real) * CELL_EDGE_COUNT);
}

uint32 DualMarchingCell::getCuttingsIdx() const
{
	// standard cuttings index (MC cell configuration)
	uint32 cuttingsIdx = 0;
	for (uint32 cornerIdx = 0; cornerIdx < Nodes::CHILD_COUNT; ++cornerIdx)
		if (mCornerDistances[cornerIdx] > 0.0f)
			cuttingsIdx |= (1 << cornerIdx);

	return cuttingsIdx;
}

Real DualMarchingCell::getInterpolationFactor(const uint32 edgeIdx) const
{
	const uint32 *ends = MC_EDGE_ENDS[edgeIdx];
	return getInterpolationFactor(mCornerDistances[ends[0]], mCornerDistances[ends[1]]);
}

Real DualMarchingCell::getInterpolationFactor(const Real distance0, const Real distance1) const
{
	// special cases
	// both ends have almost the same value?
	if (Math::EPSILON >= fabsr(distance1 - distance0))
		return 0.5f;

	// is p0 at the ISO surface?
	if (Math::EPSILON >= fabsr(distance0))
		return 0.0f;

	// is p1 at the ISO surface?
	if (Math::EPSILON >= fabsr(distance1))
		return 1.0f;

	assert((distance0 < 0.0f && distance1 > 0.0f) || (distance0 > 0.0f && distance1 < 0.0f));

	// interpolation factor
	const Real f = (0.0f - distance0) / (distance1 - distance0);
	return f;
}

bool DualMarchingCell::isDegeneratedTriangle(const int32 triangleEdges[3]) const
{
	// a triangle is degenerated
	// if one of its edges is collapsed (a single point / start and end leaf indices are equal)
	for (uint32 edgeIdx = 0; edgeIdx < 3; ++edgeIdx)
	{
		const uint32 *ends = MC_EDGE_ENDS[triangleEdges[edgeIdx]];
		if (mCornerLeafIndices[ends[0]] == mCornerLeafIndices[ends[1]])
			return true;
	}

	// or if two edges are equal
	for (uint32 edgeIdx0 = 0; edgeIdx0 < 3; ++edgeIdx0)
	{
		const uint32 edgeIdx1 = (edgeIdx0 + 1) % 3;
		const uint32 *ends0 = MC_EDGE_ENDS[triangleEdges[edgeIdx0]];
		const uint32 *ends1 = MC_EDGE_ENDS[triangleEdges[edgeIdx1]];
		
		if (mCornerLeafIndices[ends0[0]] == mCornerLeafIndices[ends1[0]] &&
			mCornerLeafIndices[ends0[1]] == mCornerLeafIndices[ends1[1]])
				return true;

		if (mCornerLeafIndices[ends0[0]] == mCornerLeafIndices[ends1[1]] &&
			mCornerLeafIndices[ends0[1]] == mCornerLeafIndices[ends1[0]])
				return true;
	}
	
	return false;
}