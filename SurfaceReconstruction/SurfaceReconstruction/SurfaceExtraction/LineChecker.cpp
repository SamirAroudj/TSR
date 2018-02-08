/*
 * Copyright (C) 2018 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */

#include "CollisionDetection/CollisionDetection.h"
#include "SurfaceReconstruction/Scene/Tree/Scope.h"
#include "SurfaceReconstruction/SurfaceExtraction/LineChecker.h"

using namespace CollisionDetection;
using namespace Math;
using namespace SurfaceReconstruction;

LineChecker::LineChecker(const Vector3 &p0, const Vector3 &p1, const uint32 maxDepth, const uint32 *nodeStates, const uint32 requiredFlag) :
	NodesChecker(maxDepth), mNodeStates(nodeStates), mP0(p0), mP1(p1), mRequiredFlag(requiredFlag)
{

}

LineChecker::~LineChecker()
{

}

bool LineChecker::isIgnored(const Scope &scope, const bool isLeaf, const uint32 depth) const
{
	// maximum depth exceeded?
	if (depth > mMaxDepth)
		return true;
	
	if (mNodeStates)
	{
		// only consider nodes whch can potentially contain empty nodes
		const uint32 nodeIdx = scope.getNodeIndex();
		const uint32 state = mNodeStates[nodeIdx];
		if (!(mRequiredFlag & state))
			return true;
	}

	// overlap between scope and line segment [p0, p1] and scope's AABB?
	const Vector3 &min = scope.getMinimumCoordinates();
	const Vector3 &max = scope.getMaximumCoordinates();

	Real entryT, exitT;
	return !intersectAABBWithLineSegment(entryT, exitT, mP0, mP1, min, max);
}