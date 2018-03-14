/*
 * Copyright (C) 2018 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */
#include <cstring>
#include "Platform/FailureHandling/Exception.h"
#include "SurfaceReconstruction/Scene/Camera/Cameras.h"
#include "SurfaceReconstruction/Scene/FileNaming.h"
#include "SurfaceReconstruction/Scene/Samples.h"
#include "SurfaceReconstruction/Scene/Scene.h"
#include "SurfaceReconstruction/Scene/Tree/DualCells.h"
#include "SurfaceReconstruction/Scene/Tree/Leaves.h"
#include "SurfaceReconstruction/Scene/Tree/Nodes.h"
#include "SurfaceReconstruction/Scene/Tree/Tree.h"

using namespace FailureHandling;
using namespace Math;
using namespace std;
using namespace Storage;
using namespace SurfaceReconstruction;

Tree::Tree() :
	mNodes(NULL), mLeaves(NULL), mDualCells(NULL)
{
	cout << "Creating scene tree." << endl;
	clear();

	const Scope rootScope = getRootScope();

	mNodes = new Nodes(rootScope);
	mLeaves = new Leaves(*mNodes, rootScope);
	mDualCells = new DualCells(*mNodes, *mLeaves);
}

Tree::Tree(const Path &filesBeginning) :
	mNodes(NULL), mLeaves(NULL), mDualCells(NULL)
{
	loadFromFile(filesBeginning);
}

Tree::~Tree()
{
	clear();
}

void Tree::eraseSamples(const uint32 *sampleOffsets)
{
	mNodes->eraseSamples(sampleOffsets);
}

bool Tree::eraseSamplesInNodes(uint32 *sampleOffsets, const uint32 *nodeStates, const uint32 oldSampleCount, const uint8 FLAG_OF_DOOM)
{
	return mNodes->eraseSamplesInNodes(sampleOffsets, nodeStates, oldSampleCount, FLAG_OF_DOOM);
}
	
void Tree::getLeaves(vector<uint32> &leaves, const Math::Vector3 &queryPosition, const Real queryRadius) const
{
	return mLeaves->getLeaves(leaves, queryPosition, queryRadius, getRootScope());
}

uint32 Tree::getLeafIndex(const Math::Vector3 &queryPosWS) const
{
	return mLeaves->getLeafIndex(getRootScope(), queryPosWS);
}

Scope Tree::getRootScope() const
{
	const Real rootSize(getRootNodeSize());
	const Real halfRootSize(rootSize * 0.5f);
	const Vector3 center(getCenter());
	const Vector3 rootCoordsWS(center.x - halfRootSize, center.y - halfRootSize, center.z - halfRootSize);

	return Scope(rootCoordsWS, rootSize, 0);
}

Vector3 Tree::getCenter() const
{
	const Vector3 *AABB = Scene::getSingleton().getSamples().getAABBWS();
	return (AABB[0] + AABB[1]) * 0.5f;
}

uint32 Tree::getSampleDepth(const uint32 sampleIdx) const
{
	const Real maxDistance = Scene::getSingleton().getSamples().getMaxSamplingDistance(sampleIdx);
	const Real temp = getRootNodeSize() / maxDistance;
	return (uint32) ceilr(log2r(temp));
}

Real Tree::getRootNodeSize() const
{
	const Samples &samples = Scene::getSingleton().getSamples();

	const Vector3 *AABB = samples.getAABBWS();
	const Math::Vector3	extent = AABB[1] - AABB[0];
	const Real temp	= Math::maximum(extent.x, extent.y);
	return Math::maximum(temp, extent.z);
}

void Tree::clear()
{
	delete mNodes;
	delete mLeaves;
	delete mDualCells;

	mNodes = NULL;
	mLeaves = NULL;
	mDualCells = NULL;
}

uint32 Tree::getNodeDepth(const uint32 nodeIdx) const
{	
	return mNodes->getDepth(nodeIdx);
}
	
void Tree::loadFromFile(const Path &filesBeginning)
{
	cout << "Loading scene octree." << endl;
	clear();

	mNodes = new Nodes(Path::extendLeafName(filesBeginning, FileNaming::ENDING_NODES));
	mLeaves = new Leaves(*mNodes, Path::extendLeafName(filesBeginning, FileNaming::ENDING_LEAVES));
	mDualCells = new DualCells(*mNodes, *mLeaves, Path::extendLeafName(filesBeginning, FileNaming::ENDING_DUAL_CELLS));
}

void Tree::saveToFiles(const Path &filesBeginning) const
{
	mNodes->saveToFile(Path::extendLeafName(filesBeginning, FileNaming::ENDING_NODES));
	mLeaves->saveToFile(Path::extendLeafName(filesBeginning, FileNaming::ENDING_LEAVES));
	mDualCells->saveToFile(Path::extendLeafName(filesBeginning, FileNaming::ENDING_DUAL_CELLS));
}