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
#include "SurfaceReconstruction/Scene/Tree/SphereNodesChecker.h"

using namespace CollisionDetection;
using namespace Math;
using namespace SurfaceReconstruction;

SphereNodesChecker::SphereNodesChecker(const Vector3 &sphereCenter, const Real sphereRadius, const uint32 maxDepth) :
	NodesChecker(maxDepth), mCenter(sphereCenter), mRadius(sphereRadius)
{

}

SphereNodesChecker::~SphereNodesChecker()
{

}

bool SphereNodesChecker::isIgnored(const Scope &scope, const bool isLeaf, const uint32 depth) const
{	
	// maximum depth exceeded?
	if (depth > mMaxDepth)
		return true;

	// intersection between current node represented by scope and 3D sphere?
	const Vector3 min = scope.getMinimumCoordinates();
	const Vector3 max = scope.getMaximumCoordinates();

	return !CollisionDetection::intersectAABBWithSphere(min, max, mCenter, mRadius);
}