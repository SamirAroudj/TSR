/*
 * Copyright (C) 2017 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */

#include "CollisionDetection/CollisionDetection.h"
#include "SurfaceReconstruction/Scene/Tree/Scope.h"
#include "SurfaceReconstruction/Scene/Tree/TriangleNodesChecker.h"

using namespace CollisionDetection;
using namespace Math;
using namespace SurfaceReconstruction;

TriangleNodesChecker::TriangleNodesChecker(const Vector3 &v0, const Vector3 &v1, const Vector3 &v2, const uint32 maxDepth) :
	NodesChecker(maxDepth), mV0(v0), mV1(v1), mV2(v2)
{

}

TriangleNodesChecker::~TriangleNodesChecker()
{

}

bool TriangleNodesChecker::isIgnored(const Scope &scope, const bool isLeaf, const uint32 depth) const
{	
	// maximum depth exceeded?
	if (depth > mMaxDepth)
		return true;

	// intersection between current node represented by scope and 3D view cone?
	const Vector3 &min = scope.getMinimumCoordinates();
	const Vector3 max = scope.getMaximumCoordinates();

	return !CollisionDetection::intersectAABBWithTriangle(min, max, mV0, mV1, mV2);
}