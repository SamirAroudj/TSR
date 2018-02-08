/*
 * Copyright (C) 2018 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */

#include "Platform/Storage/File.h"
#include "SurfaceReconstruction/Scene/Tree/Leaves.h"
#include "SurfaceReconstruction/Scene/Tree/Nodes.h"

using namespace Math;
using namespace Platform;
using namespace std;
using namespace Storage;
using namespace SurfaceReconstruction;

const uint32 Leaves::FILE_VERSION = 0;
const uint32 Leaves::INVALID_INDEX = (uint32) -1;

Leaves::Leaves(const Nodes &nodes, const Scope &rootScope) :
	Leaves(nodes)
{
	// leaf neighbors graph nodes & edges
	createScopes(rootScope);
	createNeighborsEdges(rootScope);
}

Leaves::Leaves(const Nodes &nodes, const Path &fileName) :
	Leaves(nodes)
{
	loadFromFile(fileName);
}

Leaves::Leaves(const Nodes &nodes) :
	mNodes(nodes),
	mNeighborsOffsets(NULL),
	mNeighbors(NULL)
{

}

void Leaves::createScopes(const Scope &rootScope)
{
	// reserve memory
	const uint32 nodeCount = mNodes.getCount();
	mScopes.reserve(nodeCount);
	mNodeToLeafLinks.resize(nodeCount);

	// initial node to leaf links
	std::fill(mNodeToLeafLinks.begin(), mNodeToLeafLinks.end(), INVALID_INDEX);

	// get leaves
	gatherScopes(rootScope);
	mScopes.shrink_to_fit();
}

void Leaves::gatherScopes(const Scope &scope)
{
	// anchor: is this a leaf?
	if (mNodes.isLeaf(scope.getNodeIndex()))
	{
		mNodeToLeafLinks[scope.getNodeIndex()] = getCount();
		mScopes.push_back(scope);
		return;
	}

	// gather leaves in all child sub trees
	const uint32 child0 = mNodes.getChildBlock(scope.getNodeIndex());
	const Real childSize = scope.getSize() * 0.5f;
	for (uint32 i = 0; i < Nodes::CHILD_COUNT; ++i)
	{
		Scope childScope(
			Nodes::getCoords(scope.getMinimumCoordinates(),	childSize, i),
			childSize, child0 + i);

		gatherScopes(childScope);
	}
}

void Leaves::createNeighborsEdges(const Scope &root)
{
	// compute the start indices of the leaf neighbors buffers
	const uint32 leafCount = getCount();
	mNeighborsOffsets = new uint32[leafCount + 1];
	memset(mNeighborsOffsets, 0, sizeof(uint32) * (leafCount + 1));

	for (uint32 leafIdx = 0; leafIdx < leafCount; ++leafIdx)
	{
		const Scope &leaf = mScopes[leafIdx];

		for (uint32 sideIdx = 0; sideIdx < Nodes::SIDE_COUNT; ++sideIdx)
		{
			const uint32 nodeIdx = getUnresponsibleEdgeNeighbor(root, leaf, sideIdx);
			if (Nodes::isInvalidNodeIdx(nodeIdx))
				continue;

			const uint32 neighborIdx = mNodeToLeafLinks[nodeIdx];
			++(mNeighborsOffsets[leafIdx]);
			++(mNeighborsOffsets[neighborIdx]);
		}

	}

	// prefix sum of leaf neighbors sizes to get start indices of leaf neighbors buffers
	uint32 edgeCount = 0;
	for (uint32 i = 0; i < leafCount + 1; ++i)
	{
		uint32 size = mNeighborsOffsets[i];
		mNeighborsOffsets[i] = edgeCount;
		edgeCount += size;
	}

	// create neighbors buffers containing the edges in form of simple uint32 variable references
	mNeighbors = new uint32[edgeCount];
	memset(mNeighbors, INVALID_INDEX, sizeof(uint32) * edgeCount);

	for (uint32 leafIdx = 0; leafIdx < leafCount; ++leafIdx)
	{
		const Scope &leaf = mScopes[leafIdx];
		for (uint32 sideIdx = 0; sideIdx < Nodes::SIDE_COUNT; ++sideIdx)
		{
			const uint32 nodeIdx = getUnresponsibleEdgeNeighbor(root, leaf, sideIdx);
			if (Nodes::isInvalidNodeIdx(nodeIdx))
				continue;

			const uint32 neighborIdx = mNodeToLeafLinks[nodeIdx];
			insertNeighborsEdge(leafIdx, neighborIdx);
			insertNeighborsEdge(neighborIdx, leafIdx);
		}
	}

	#ifdef _DEBUG
		for (uint32 edgeIdx = 0; edgeIdx < edgeCount; ++edgeIdx)
			assert(mNeighbors[edgeIdx] != INVALID_INDEX);
	#endif // _DEBUG
}

bool Leaves::insertNeighborsEdge(const uint32 leafIdx, const uint32 leafNeighborIdx)
{
	const uint32 start = mNeighborsOffsets[leafIdx];
	const uint32 end = mNeighborsOffsets[leafIdx + 1];

	for (uint32 edgeIdx = start; edgeIdx < end; ++edgeIdx)
	{
		uint32 &otherNode = mNeighbors[edgeIdx];
		if (!Nodes::isInvalidNodeIdx(otherNode))
			continue;

		otherNode = leafNeighborIdx;
		return true;
	}

	assert(false);
	return false;
}

uint32 Leaves::getUnresponsibleEdgeNeighbor(const Scope &root, const Scope &leaf, const uint32 sideIdx) const
{
	// start at root & find the neighbor with minimum size leaf.mSize which contains targetPosWS
	const Vector3 targetPosWS = Nodes::getSideCenterPositionWS(leaf, sideIdx, 0.5f * leaf.getSize());
	const uint32 maxDepth = mNodes.getDepth(leaf.getNodeIndex());
	Scope neighbor(root);

	uint32 neighborDepth = mNodes.getNode(neighbor, targetPosWS, maxDepth);
	assert(neighbor.getNodeIndex() != leaf.getNodeIndex());

	if (Nodes::isInvalidNodeIdx(neighbor.getNodeIndex()))
		return neighbor.getNodeIndex();

	if (!mNodes.isLeaf(neighbor.getNodeIndex()))
		return INVALID_INDEX;

	// both at the same level? don't count the links twice, count links only if leaf index is smaller
	if ((neighborDepth == maxDepth) && (neighbor.getNodeIndex() < leaf.getNodeIndex()))
		return INVALID_INDEX;

	return neighbor.getNodeIndex();
}

Leaves::~Leaves()
{
	clear();
}

void Leaves::clear()
{
	// links
	mNodeToLeafLinks.clear();

	// leaves
	mScopes.clear();

	// neighbors
	delete [] mNeighborsOffsets;
	delete [] mNeighbors;
	mNeighborsOffsets = NULL;
	mNeighbors = NULL;
}

uint32 Leaves::getLeafIndex(const Scope &rootScope, const Vector3 &queryPosWS) const
{
	// find the node which contains queryPosWS
	Scope scope(rootScope);
	mNodes.getNode(scope, queryPosWS, (uint32) -1);
	
	// return the corresponding leaf idx if it exists
	const uint32 nodeIdx = scope.getNodeIndex();
	if (Nodes::isInvalidNodeIdx(nodeIdx))
		return INVALID_INDEX;
	else
		return mNodeToLeafLinks[nodeIdx];
}

void Leaves::getLeaves(vector<uint32> &leaves,
	const Vector3 &queryPosition, const Real queryRadius, const Scope &scope) const
{
	mNodes.getLeafNodes(leaves, queryPosition, queryRadius, scope);

	// exchange node with leaf indices
	const uint32 leafCount = (uint32) leaves.size();
	for (uint32 localLeafIdx = 0; localLeafIdx < leafCount; ++localLeafIdx)
	{
		const uint32 nodeIdx = leaves[localLeafIdx];
		leaves[localLeafIdx] = mNodeToLeafLinks[nodeIdx];
	}
}

void Leaves::loadFromFile(const Path &fileName)
{
	clear();

	File file(fileName, File::OPEN_READING, true, FILE_VERSION);

	// load element counts: node count, leaf count, total neighbor count
	uint32 counts[3];
	file.read(counts, sizeof(uint32) * 3, sizeof(uint32), 3);

	// reserve memory for links, scopes and neighbors
	const uint32 nodeCount = counts[0];
	const uint32 leafCount = counts[1];
	const uint32 totalNeighborCount = counts[2];
	mNodeToLeafLinks.resize(nodeCount);
	mScopes.resize(leafCount);
	mNeighborsOffsets = new uint32 [leafCount + 1];
	mNeighbors = new uint32 [totalNeighborCount];

	//  read links
	file.read(mNodeToLeafLinks.data(), sizeof(uint32) * nodeCount, sizeof(uint32), nodeCount);

	// read leaves
	file.read(mScopes.data(), sizeof(Scope) * leafCount, sizeof(Scope), leafCount);

	// read leaf neighbohoods
	const uint32 offsetCount = leafCount + 1;
	file.read(mNeighborsOffsets, sizeof(uint32) * offsetCount, sizeof(uint32), offsetCount);
	file.read(mNeighbors, sizeof(uint32) * totalNeighborCount, sizeof(uint32), totalNeighborCount);
}

void Leaves::saveToFile(const Path &fileName) const
{
	// is there anything to save?
	if (mScopes.empty())
		return;

	// create file with version & AABB, mOrderedSamples
	File file(fileName, File::CREATE_WRITING, true, FILE_VERSION);

	// save element counts
	const uint32 nodeCount = mNodes.getCount();
	const uint32 leafCount = getCount();
	const uint32 totalNeighborCount = mNeighborsOffsets[leafCount];
	uint32 counts[3] =
	{
		nodeCount,
		leafCount,
		totalNeighborCount
	};

	file.write(counts, sizeof(uint32), 3);

	// save links
	file.write(mNodeToLeafLinks.data(), sizeof(uint32), nodeCount);

	// save leaves
	file.write(mScopes.data(), sizeof(Scope), leafCount);

	// save leaf neighbohoods
	file.write(mNeighborsOffsets, sizeof(uint32), leafCount + 1);
	file.write(mNeighbors, sizeof(uint32), totalNeighborCount);
}