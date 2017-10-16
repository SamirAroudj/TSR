/*
 * Copyright (C) 2017 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */

#include "SurfaceReconstruction/Scene/Tree/Scope.h"
#include "SurfaceReconstruction/SurfaceExtraction/SphereNodeStatesChecker.h"

using namespace Math;
using namespace SurfaceReconstruction;

SphereNodeStatesChecker::SphereNodeStatesChecker(const Vector3 &sphereCenter, const Real sphereRadius, const uint32 maxDepth,
	const uint32 *nodeStates, const uint32 requiredFlags) :
	SphereNodesChecker(sphereCenter, sphereRadius, maxDepth), mNodeStates(nodeStates), mRequiredFlags(requiredFlags)
{

}

SphereNodeStatesChecker::~SphereNodeStatesChecker()
{

}

bool SphereNodeStatesChecker::isIgnored(const Scope &scope, const bool isLeaf, const uint32 depth) const
{	
	// ignore if there was no change
	const uint32 state = mNodeStates[scope.getNodeIndex()];
	if (mRequiredFlags != (mRequiredFlags & state))
		return true;

	return SphereNodesChecker::isIgnored(scope, isLeaf, depth);
}