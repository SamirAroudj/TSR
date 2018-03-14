/*
 * Copyright (C) 2018 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */

#include "CollisionDetection/CollisionDetection.h"
#include "SurfaceReconstruction/Scene/Samples.h"
#include "SurfaceReconstruction/Scene/Scene.h"
#include "SurfaceReconstruction/Scene/Tree/Scope.h"
#include "SurfaceReconstruction/SurfaceExtraction/ConeNodesChecker.h"

using namespace CollisionDetection;
using namespace Math;
using namespace SurfaceReconstruction;

ConeNodesChecker::ConeNodesChecker(const uint32 sampleIdx, const Vector3 &apexPosWS, const Real endRadius, const Real length, 
	const uint32 maxDepth, const uint32 *nodeStates, const uint32 requiredFlags) :
	ConeNodesChecker
	(
		apexPosWS, 
		-Scene::getSingleton().getSamples().getNormalWS(sampleIdx),
		Scene::getSingleton().getSamples().getPositionWS(sampleIdx),
		endRadius,
		length,
		maxDepth, nodeStates, requiredFlags
	)
{
		
}

ConeNodesChecker::ConeNodesChecker(const Vector3 &apex,	const Vector3 &endOutNormal, const Vector3 &endCenter, const Real endRadius, const Real length,
	const uint32 maxDepth, const uint32 *nodeStates, const uint32 requiredFlags) :
	NodesChecker(maxDepth),
	ObliqueCircularCone(apex, endOutNormal, endCenter, endRadius, length),
	mNodeStates(nodeStates),
	mRequiredFlags(requiredFlags)
{

}

ConeNodesChecker::~ConeNodesChecker()
{

}

bool ConeNodesChecker::isIgnored(const Scope &scope, const bool isLeaf, const uint32 depth) const
{	
	// maximum depth exceeded?
	if (depth > mMaxDepth)
		return true;

	// does the node contain any evaluation position overlaping with a sample?
	const uint32 nodeIdx = scope.getNodeIndex();
	const uint32 state = mNodeStates[nodeIdx];
	if (mRequiredFlags != (mRequiredFlags & state))
		return true;

	// intersection between current node represented by scope and 3D cone?
	const Vector3 min = scope.getMinimumCoordinates();
	const Real size = scope.getSize();
	return haveSeparatingAxis(min, Vector3(size, size, size));
}