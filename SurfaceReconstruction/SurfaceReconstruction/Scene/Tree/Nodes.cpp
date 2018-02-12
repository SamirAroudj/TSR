/*
 * Copyright (C) 2018 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */

#include "CollisionDetection/CollisionDetection.h"
#include "Platform/FailureHandling/Exception.h"
#include "Platform/Storage/File.h"
#include "SurfaceReconstruction/Scene/Samples.h"
#include "SurfaceReconstruction/Scene/Scene.h"
#include "SurfaceReconstruction/Scene/Tree/Nodes.h"
#include "SurfaceReconstruction/Scene/View/View.h"
#include "SurfaceReconstruction/SurfaceExtraction/Occupancy.h"

// todo comments

using namespace FailureHandling;
using namespace Math;
using namespace std;
using namespace Storage;
using namespace SurfaceReconstruction;

const uint32 Nodes::FILE_VERSION = 0;
const uint32 Nodes::INVALID_INDEX = (uint32) -1;

uint32 Nodes::getChildIndexForOffset(const Vector3 &offset)
{
	uint32 indexOffset = 0;

	if (offset.x >= 0.0f)
		indexOffset |= (1 << 0);
	if (offset.y >= 0.0f)
		indexOffset |= (1 << 1);
	if (offset.z >= 0.0f)
		indexOffset |= (1 << 2);

	return indexOffset;
}

Vector3 Nodes::getCoords(const Vector3 &nodeCoordsWS, const Real stepSize, uint32 index)
{
	Vector3 coordsWS(nodeCoordsWS.x, nodeCoordsWS.y, nodeCoordsWS.z);

	if (index & (1 << 0))
		coordsWS.x += stepSize;
	if (index & (1 << 1))
		coordsWS.y += stepSize;
	if (index & (1 << 2))
		coordsWS.z += stepSize;

	return coordsWS;
}

Vector3 Nodes::getSideCenterPositionWS(const Scope &scope, const uint32 sideIdx, const Real stepSize)
{
	// compute a position at the { left, right, top, bottom } side one node size step along a single axis away from the scope node center
	const Real halfSize = scope.mSize * 0.5f;

	// center position
	Vector3 sidePosition(scope.mPositionWS.x + halfSize,
						 scope.mPositionWS.y + halfSize,
						 scope.mPositionWS.z + halfSize);

	// side + stepSize position
	const Real distance = halfSize + stepSize;
	switch (sideIdx)
	{
		case 0: sidePosition.x -= distance; break;
		case 1: sidePosition.x += distance; break;
		case 2: sidePosition.y -= distance; break;
		case 3: sidePosition.y += distance; break;
		case 4: sidePosition.z -= distance; break;
		case 5: sidePosition.z += distance; break;
		default: assert(false); break;
	}

	return sidePosition;
}

bool Nodes::isContainer(const Scope &scope, const uint32 sampleIdx)
{
	const Samples &samples = Scene::getSingleton().getSamples();

	// tree node container criterion like in FSSR: the sample belongs into the containing node
	// the sample position is in the node and
	const Real scopeSize = scope.getSize();
	if (!Nodes::contains(samples.getPositionWS(sampleIdx), scope.getMinimumCoordinates(), scopeSize))
		return false;

	// if the sample support range fits to the node size:
	const Real maxSamplingRange = samples.getMaxSamplingDistance(sampleIdx);
	return (scopeSize <= maxSamplingRange && maxSamplingRange < 2.0f * scopeSize);
}

void Nodes::clear()
{
	// nodes' data
	mChildren.clear();
	mParents.clear();
	mSamplesPerNodes.clear();
	mSampleStartIndices.clear();
}

void Nodes::getLeafNodes(vector<uint32> &leaves,
	const Vector3 &queryPosition, const Real queryRadius, const Scope &scope) const
{
	// is it a leaf?
	if (isLeaf(scope.mIdx))
	{
		leaves.push_back(scope.mIdx);
		return;
	}

	// intersection between current node and sample support volume?
	if(!CollisionDetection::intersectAABBWithSphere(scope.mPositionWS, scope.getMaximumCoordinates(), queryPosition, queryRadius))
		return;
	
	// add sampleness to children
	const Real childSize = 0.5f * scope.mSize;
	for (uint32 childOffset = 0; childOffset < CHILD_COUNT; ++childOffset)
	{
		const Scope childScope = getChildScope(scope, childOffset);
		getLeafNodes(leaves, queryPosition, queryRadius, childScope);
	}
}

uint32 Nodes::getDepth(const uint32 nodeIdx) const
{
	assert(!Nodes::isInvalidNodeIdx(nodeIdx));

	// go to root & count steps
	uint32 depth = 0;
	for (uint32 currentIdx = mParents[nodeIdx]; !isInvalidNodeIdx(currentIdx); currentIdx = mParents[currentIdx])
		++depth;

	return depth;
}

uint32 Nodes::getNode(Scope &scope,	const Vector3 &queryPosWS, const uint32 maxDepth) const
{
	// check start node:
	// cannot find a containing node if the start node is already too deep or does not contain targetPosWS
	uint32 depth = getDepth(scope.mIdx);
	if (depth > maxDepth || !Nodes::contains(queryPosWS, scope.mPositionWS, scope.mSize))
	{
		scope.mIdx = INVALID_INDEX;
		return depth;
	}

	// go down the tree until a leaf containing targetPosWS is reached or minSize is not met anymore
	while (true)
	{
		// stop if leaf
		if (isLeaf(scope.mIdx))
			return depth;

		// stop if too deep
		if (depth == maxDepth)
			return depth;

		// step into containing child
		for (uint32 childOffset = 0; childOffset < CHILD_COUNT; ++childOffset)
		{
			// outside child?
			const Real childSize = scope.mSize * 0.5f;
			const Vector3 childCoords = getCoords(scope.mPositionWS, childSize, childOffset);
			if (!Nodes::contains(queryPosWS, childCoords, childSize))
				continue;

			// fitting child node -> go down here
			scope.mIdx = mChildren[scope.mIdx] + childOffset;
			scope.mPositionWS = childCoords;
			scope.mSize = childSize;
			++depth;
			break;
		}
	}
}

Nodes::Nodes(Samples *&reorderedSamples, const Scope &rootScope)
{
	clear();

	createNodes(rootScope);
	balance(rootScope); // have at maximum 1 depth level difference for each pair of adjacent leaf nodes
	reorder();
	reorderedSamples = reorderSamplesAndFillNodes(rootScope);
}

void Nodes::createNodes(const Scope &rootScope)
{
	// get sample count & reserver memory
	const Samples &samples = Scene::getSingleton().getSamples();
	const uint32 sampleCount = samples.getCount();

	// create nodes for samples, start with root
	clear();
	resize(1);

	// create nodes which surround samples
	for (uint32 sampleIdx = 0; sampleIdx < sampleCount; ++sampleIdx)
	{
		Scope scope(rootScope);
		createNodesForContainment(scope, sampleIdx);
	}

	// create nodes which make sure that there is no aliasing w.r.t. implicit surface representation sampling
	for (uint32 sampleIdx = 0; sampleIdx < sampleCount; ++sampleIdx)
	{
		// make sure that there are nodes with close enough centers and small enough sizes at both sides of sample to sample it later
		createNodesForSampling(rootScope, true, sampleIdx);
		createNodesForSampling(rootScope, false, sampleIdx);
	}
}

void Nodes::createNodesForContainment(Scope &scope, const uint32 sampleIdx)
{
	// build the tree structure until there is a node which contains sample
	while (!Nodes::findContainer(scope, sampleIdx))
		Nodes::createChildren(scope.getNodeIndex());

	++mSamplesPerNodes[scope.getNodeIndex()];
}

bool Nodes::findContainer(Scope &scope, const uint32 sampleIdx) const
{
	const Samples &samples = Scene::getSingleton().getSamples();
	const Vector3 &samplePositionWS = samples.getPositionWS(sampleIdx);
	const Real maxSamplingRange = samples.getMaxSamplingDistance(sampleIdx);

	if (!Nodes::contains(samplePositionWS, scope.mPositionWS, scope.mSize, Math::EPSILON))
		throw Exception("Cannot find a container node for a position which is not contained by the start node for the search.");

	// descend in the tree until the sample containing node is reached or a tree end is found
	while (true)
	{
		#ifdef _DEBUG
			// due to numerical errors: only check that distance < epsilon - todo: better solution?
			const Vector3 scopeMax = scope.getMaximumCoordinates();
			const Real distance = CollisionDetection::getDistanceToAABB(samplePositionWS, scope.mPositionWS, scopeMax);
			assert(distance < Math::EPSILON);
		#endif // _DEBUG

		if (scope.mSize <= maxSamplingRange)
			return true;

		// the sample must be inserted into a child node -> any children?
		if (isLeaf(scope.getNodeIndex()))
			return false;

		// the sample belongs to the containing child node
		// find fitting child
		const Real childSize = scope.mSize * 0.5f;
		const Vector3 centerWS(scope.mPositionWS.x + childSize, scope.mPositionWS.y + childSize, scope.mPositionWS.z + childSize); 
		const Vector3 sampleOffset = samplePositionWS - centerWS;
		const uint32 childIdxOffset = getChildIndexForOffset(sampleOffset);

		scope = getChildScope(scope, childIdxOffset, childSize);
	}

	assert(false);
	return false;
}

uint32 Nodes::createNodesForSampling(Scope scope, const bool positiveSide, const uint32 sampleIdx)
{
	// does a fitting node already exist?
	uint32 nodeIdx = findReliableSamplingNode(scope, positiveSide, sampleIdx);
	if (!Nodes::isInvalidNodeIdx(nodeIdx))
		return nodeIdx;

	// sample data
	const Samples &samples = Scene::getSingleton().getSamples();
	const Vector3 &samplePosWS = samples.getPositionWS(sampleIdx);
	const Vector3 &sampleNormal = samples.getNormalWS(sampleIdx);
	const Real maxSamplingRange = samples.getMaxSamplingDistance(sampleIdx);

	// this is only called if there is no reliable sampling node for (sample, positiveSide)
	while (true)
	{
		// node with ID scope.mIdx == most reasonable node that should contain the sampling position

		// no children -> create children to get closer to the required reliable sampling node
		if (isLeaf(scope.getNodeIndex()))
		{
			Nodes::createChildren(scope.getNodeIndex());

			// are we done?
			const uint32 nodeIdx = Nodes::findReliableSamplingNode(scope, positiveSide, sampleIdx);
			if (!Nodes::isInvalidNodeIdx(nodeIdx))
				return nodeIdx;
		}

		// find the child node which covers most of the considered = ${positiveSide} half space of the sample
		const Real childSize = scope.mSize * 0.5f;
		assert(childSize > 0.0f);
		if (0.0f == childSize)
			throw Exception("Nodes::createNodesForSampling: Reached a tree node of size 0 due to finite numerical precision!");

		Real score = -REAL_MAX;
		uint32 bestChild = INVALID_INDEX;

		for (uint32 childIdx = 0; childIdx < CHILD_COUNT; ++childIdx)
		{
			// intersect the line segment LS along positive/negative sample normal direction starting at sample position and ending at search radius with child childIdx
			const Vector3 AABBMin = getCoords(scope.mPositionWS, childSize, childIdx);
			const Vector3 AABBMax = Vector3(AABBMin.x + childSize, AABBMin.y + childSize, AABBMin.z + childSize);
			const Vector3 endPoint = samplePosWS + (sampleNormal * (positiveSide ? maxSamplingRange : -maxSamplingRange));

			// intersection?
			Real entryT = REAL_MAX;
			Real exitT = -REAL_MAX;
			if (!CollisionDetection::intersectAABBWithLineSegment(entryT, exitT, samplePosWS, endPoint, AABBMin, AABBMax))
				continue;

			// compute line segment length -> choose the child with the largest overlap
			const Real overlapScore = exitT - entryT;
			if (overlapScore <= score)
				continue;

			score = overlapScore;
			bestChild = childIdx;
			if (overlapScore >= 1.0f - Math::EPSILON)
				break;
		}

		// step down into the found child and continue there
		assert(bestChild < CHILD_COUNT);
		scope = getChildScope(scope, bestChild, childSize);
	}
}

void Nodes::balance(const Scope &rootScope)
{
	// split nodes and create new leaves until maximum tree depth difference for pairs of adjacent leaves is 1
	while (true)
		if (!createNodesForBalancing(0, rootScope, rootScope))
			break;
}

bool Nodes::createNodesForBalancing(const uint32 depth, const Scope &scope, const Scope &rootScope)
{
	// current node has children?
	if (!isLeaf(scope.mIdx))
	{
		// enforce balancing criterium for all sub trees of this node
		const uint32 child0 = mChildren[scope.mIdx];
		const Real childSize = 0.5f * scope.mSize;
		bool createdNode = false;

		for (uint32 childIdx = 0; childIdx < CHILD_COUNT; ++childIdx)
		{
			const Scope childScope = getChildScope(scope, childIdx, childSize);
			createdNode |= createNodesForBalancing(depth + 1, childScope, rootScope);
		}

		return createdNode;
	}
	
	// leaf at level depth -> check balancing criterium
	// is there a direct neighbor at a tree level > depth + 1?
	bool createdNode = false;
	for (uint32 sideIdx = 0; sideIdx < SIDE_COUNT; ++sideIdx)
	{
		const Vector3 targetPosWS = Nodes::getSideCenterPositionWS(scope, sideIdx, scope.mSize * 0.5f);
		Scope neighbor(rootScope);

		// get the neighbor at maximum depth depth
		uint32 neighborDepth = Nodes::getNode(neighbor, targetPosWS, depth);
		if (Nodes::isInvalidNodeIdx(neighbor.mIdx))
			continue;

		// balancing criterium is enforced via leaves
		if (!isLeaf(neighbor.mIdx))
			continue;

		// this node is too deep? -> create children for neighbor
		while (depth > neighborDepth + 1)
		{
			Nodes::createChildren(neighbor.mIdx);
			neighborDepth = Nodes::getNode(neighbor, targetPosWS, depth);
			createdNode = true;

			assert(!Nodes::isInvalidNodeIdx(neighbor.mIdx));
		}
	}

	return createdNode;
}

void Nodes::createChildren(const uint32 nodeIdx)
{
	// new node count must be smaller than largest possible number for uint32 variables as uint32 indices are used to refer to nodes
	const uint32 oldNodeCount = getCount();
	size_t newNodeCount = (size_t) oldNodeCount + CHILD_COUNT;
	assert((uint32) -1 > newNodeCount);
	if ((size_t) ((uint32) -1) <= newNodeCount)
	{
		throw Exception("Required scene tree node count is larger than supported. Up to 2^32 - 2 nodes are supported.");
		return;
	}

	// create children and set corresponding links
	resize((uint32) newNodeCount);

	mChildren[nodeIdx] = oldNodeCount;
	for (uint32 childIdx = 0; childIdx < CHILD_COUNT; ++childIdx)
		mParents[oldNodeCount + childIdx] = nodeIdx;
}

void Nodes::reserve(const uint32 nodeCount)
{
	mChildren.reserve(nodeCount);
	mParents.reserve(nodeCount);
	mSamplesPerNodes.reserve(nodeCount);
	mSampleStartIndices.reserve(nodeCount);
}

void Nodes::resize(const uint32 newNodeCount)
{
	mChildren.resize(newNodeCount, INVALID_INDEX);
	mParents.resize(newNodeCount, INVALID_INDEX);
	mSamplesPerNodes.resize(newNodeCount, 0);
	mSampleStartIndices.resize(newNodeCount, Samples::INVALID_INDEX);
}

uint32 Nodes::findReliableSamplingNode(const Scope &scope, const bool positiveSide, const uint32 sampleIdx) const
{
	// reliable sampling node s if (s.center at right side of sample & within max relative sampling distance) && if s is subdivided -> one of s direct children fullfills sampling criteria
	// if subdivision criterion is fullfilled then there are always children centers which are close enough and at the right side for any further subdivisions
	const Real childSize = scope.mSize * 0.5f;

	// this is a leaf -> fitting sampling position = node center & also = any child node center if subdivided?
	if (isLeaf(scope.mIdx))
	{
		const bool fitting			= Nodes::fittingSamplingNodeCenter(scope.mPositionWS, childSize, positiveSide, sampleIdx);
		const Real grandChildSize	= childSize  * 0.5f;

		if (fitting)
			for (uint32 i = 0; i < CHILD_COUNT; ++i)
				if (Nodes::fittingSamplingNodeCenter(getCoords(scope.mPositionWS, childSize, i), grandChildSize, positiveSide, sampleIdx))
					return scope.mIdx;

		return INVALID_INDEX;
	}

	// check whether children can provide a reliable node for sampling
	const uint32 child0 = mChildren[scope.mIdx];
	for (uint32 relativeChildIdx = 0; relativeChildIdx < CHILD_COUNT; ++relativeChildIdx)
	{
		// overlaping child?
		const Scope childScope = getChildScope(scope, relativeChildIdx, childSize);
		if (!Nodes::intersect(childScope, positiveSide, sampleIdx))
			continue;

		// test this overlaping child node
		const uint32 nodeIdx = Nodes::findReliableSamplingNode(childScope, positiveSide, sampleIdx);
		if (!Nodes::isInvalidNodeIdx(nodeIdx))
			return nodeIdx;
	}

	return INVALID_INDEX;
}

bool Nodes::fittingSamplingNodeCenter(const Vector3 &nodeCoordsWS, const Real childSize, const bool positiveSide, const uint32 sampleIdx) const
{
	// get sample data
	const Samples &samples = Scene::getSingleton().getSamples();
	const Vector3 &samplePosWS = samples.getPositionWS(sampleIdx);

	// world space center within max radius?
	const Vector3 centerWS(nodeCoordsWS.x + childSize, nodeCoordsWS.y + childSize, nodeCoordsWS.z + childSize);
	const Real distanceSq = (centerWS - samplePosWS).getLengthSquared();
	const Real maxSamplingRange = samples.getMaxSamplingDistance(sampleIdx);

	if (distanceSq > maxSamplingRange * maxSamplingRange)
		return false;

	// sampling position is at the desired sample side?
	bool onPositiveSide = (samples.getDistanceToPlane(centerWS, sampleIdx) > 0.0f);
	return onPositiveSide == positiveSide;
}

bool Nodes::intersect(const Scope &scope, bool positiveSide, const uint32 sampleIdx) const
{
	// get sample data
	const Samples &samples = Scene::getSingleton().getSamples();
	const Vector3 &samplePosWS = samples.getPositionWS(sampleIdx);

	// all node vertices in the wrong half space of sample where no sample point is searched for?
	bool allOnWrongSide = true;
	for (uint32 i = 0; i < CHILD_COUNT; ++i)
	{
		const Vector3 corner = getCoords(scope.mPositionWS, scope.mSize, i);
		const bool  inPositive = (samples.getDistanceToPlane(corner, sampleIdx) > 0.0f);
		if (inPositive != positiveSide)
			continue;

		allOnWrongSide = false;
		break;
	}
	if (allOnWrongSide)
		return false;

	// continue with normal collision test between sphere and AABB
	const Real sphereRadius	= samples.getMaxSamplingDistance(sampleIdx);
	const Vector3 &min = scope.getMinimumCoordinates();
	const Vector3 max = scope.getMaximumCoordinates();

	return CollisionDetection::intersectAABBWithSphere(min, max, samplePosWS, sphereRadius);
}

void Nodes::reorder()
{
	assert((uint32) -1 > getCount());
	const uint32 nodeCount = getCount();
	vector<uint32> newOrder(nodeCount, INVALID_INDEX);

	// compute spatially coherent nodes ordering
	newOrder[0] = 0;
	computeReorderedAddresses(newOrder, 0, 1);
	reorder(newOrder);
}

uint32 Nodes::computeReorderedAddresses(vector<uint32> &newOrder, const uint32 nodeIdx, const uint32 nextFreeAddress) const
{
	// anchor: no children to be placed
	if (isLeaf(nodeIdx))
		return nextFreeAddress;

	// enforce the new ordering for all further descendants
	const uint32 childrenBlockIdx = mChildren[nodeIdx];
	uint32 newNextFreeAddress = nextFreeAddress + CHILD_COUNT;

	for (uint32 relativeChildIdx = 0; relativeChildIdx < CHILD_COUNT; ++relativeChildIdx)
	{	
		const uint32 childIdx = childrenBlockIdx + relativeChildIdx;
		newOrder[childIdx] = nextFreeAddress + relativeChildIdx;
		newNextFreeAddress = computeReorderedAddresses(newOrder, childIdx, newNextFreeAddress);
	}
	
	return newNextFreeAddress;
}

void Nodes::reorder(const vector<uint32> &newOrder)
{
	// all nodes must follow the NEW ORDER
	// (NEW ORDER must be read with a dark and low voice)
	const uint32 nodeCount = getCount();
	vector<uint32> *nodesData[2] = { &mChildren, &mParents};
	vector<uint32> orderedAttributes(nodeCount);
	
	// reorder links: mChildren, mParents
	for (uint32 type = 0; type < 2; ++type)
	{
		vector<uint32> &links = *(nodesData[type]);

		#pragma omp parallel for
		for (int64 sourceIdx = 0; sourceIdx < nodeCount; ++sourceIdx)
		{
			const uint32 targetIdx = newOrder[sourceIdx];
			const uint32 oldIdx = links[sourceIdx];
			const uint32 newIdx = (INVALID_INDEX == oldIdx ? oldIdx : newOrder[oldIdx]);
			orderedAttributes[targetIdx] = newIdx;
		}
		links.swap(orderedAttributes);
	}

	// reorder mSamplesPerNodes
	#pragma omp parallel for
	for (int64 sourceIdx = 0; sourceIdx < nodeCount; ++sourceIdx)
		orderedAttributes[newOrder[sourceIdx]] = mSamplesPerNodes[sourceIdx];
	mSamplesPerNodes.swap(orderedAttributes);

	// mSampleStartIndices = prefix sum(mSamplesPerNodes)
	for (uint32 nodeIdx = 0, nextStart = 0; nodeIdx < nodeCount; ++nodeIdx)
	{
		mSampleStartIndices[nodeIdx] = nextStart;
		nextStart += mSamplesPerNodes[nodeIdx];
		mSamplesPerNodes[nodeIdx] = 0;
	}
}

Samples *Nodes::reorderSamplesAndFillNodes(const Scope &rootScope)
{
	// even all samples must follow the NEW ORDER
	// (remember that NEW ORDER must be read with a dark and low voice)
	Scene &scene = Scene::getSingleton();
	const Samples &oldSamples = scene.getSamples();
	const uint32 sampleCount = oldSamples.getCount();

	// the samples within a single node are moved so that they are all within a single memory block
	Samples *newSamples = new Samples(oldSamples.getViewsPerSample(), sampleCount, oldSamples.getAABBWS());
	
	#pragma omp parallel for
	for (int64 sampleIdx = 0; sampleIdx < sampleCount; ++sampleIdx)
	{
		const uint32 oldSampleIdx = (uint32) sampleIdx;
		
		// get the node containing the sample
		//there must be a containing node (otherwise the tree construction implementation is wrong)
		Scope scope = rootScope;
		const bool found = findContainer(scope, oldSampleIdx);
		const uint32 nodeIdx = scope.getNodeIndex();
		if (!found)
			throw Exception("There is a sample which was not inserted into the Tree object!");
		
		// update tree & newSamples
		uint32 offset = Samples::INVALID_INDEX;
		uint32 &samplesInNode = mSamplesPerNodes[nodeIdx];
		#pragma omp critical (reorderedSamplesTargetIndex)
			offset = samplesInNode++;

		const uint32 targetIdx = mSampleStartIndices[nodeIdx] + offset;
		newSamples->set(targetIdx, oldSamples, oldSampleIdx);
	}
	
	// All samples follow the NEW ORDER!
	checkSampleIndicesOfNodes();
	return newSamples;
}

uint32 Nodes::getSamples(uint32 &sampleCount, const uint32 nodeIdx) const
{
	sampleCount = mSamplesPerNodes[nodeIdx];
	return mSampleStartIndices[nodeIdx];
}

void Nodes::eraseSamples(const uint32 *sampleOffsets)
{
	cout << "Erasing samples from scene tree." << endl;

	// for each node: possibly new start index & sample count
	const int64 nodeCount = getCount();

	#pragma omp parallel for
	for (int64 nodeIdx = 0; nodeIdx < nodeCount; ++nodeIdx)
	{
		// get node data
		const uint32 start = mSampleStartIndices[nodeIdx];
		const uint32 end = start + mSamplesPerNodes[nodeIdx];
		
		// #deleted samples of node
		uint32 deletedCount = 0;
		for (uint32 sampleIdx = start; sampleIdx < end; ++sampleIdx)
			if (sampleOffsets[sampleIdx] != sampleOffsets[sampleIdx + 1])
				++deletedCount;

		// update node
		assert(deletedCount <= mSamplesPerNodes[nodeIdx]);
		mSamplesPerNodes[nodeIdx] -= deletedCount;
		mSampleStartIndices[nodeIdx] = start - sampleOffsets[start];
	}

	cout << "Finished scene tree update." << endl;
}

bool Nodes::eraseSamplesInNodes(uint32 *sampleOffsets, const uint32 *nodeStates, const uint32 oldSampleCount, const uint8 FLAG_OF_DOOM)
{
	// each sample gets an offset which defines how far it must be  moved towards the relative memory block beginning for compaction and removal of doomed samples
	sampleOffsets[0] = 0;

	// go through all nodes and compute "increased" offsets for samples in doomed nodes
	const size_t nodeCount = getCount();
	uint32 processedSampleCount = 0;
	uint32 newSampleStartIdx = 0;

	for (size_t nodeIdx = 0; nodeIdx < nodeCount; ++nodeIdx)
	{
		// update node
		mSampleStartIndices[nodeIdx] = newSampleStartIdx;
		if (0 == mSamplesPerNodes[nodeIdx])
			continue;

		// keep samples of this node?
		if (!(FLAG_OF_DOOM & nodeStates[nodeIdx]))
		{	
			// no compaction - don't increase offsets
			const uint32 blockStartOffset = sampleOffsets[processedSampleCount];
			uint32 *start = sampleOffsets + processedSampleCount + 1;
			for (uint32 i = 0; i < mSamplesPerNodes[nodeIdx]; ++i)
				start[i] = blockStartOffset;

			newSampleStartIdx += mSamplesPerNodes[nodeIdx];
			processedSampleCount += mSamplesPerNodes[nodeIdx];
			continue;
		}

		// This node and its samples are DOOMED! Increase offheads for the doomed ones! // "OFF WITH THEIR HEADS!" - The Queen of Hearts. https://www.youtube.com/watch?v=CtCQHCOls2E
		uint32 *start = sampleOffsets + processedSampleCount;
		for (uint32 i = 0; i < mSamplesPerNodes[nodeIdx]; ++i)
			start[i + 1] = start[i] + 1;

		processedSampleCount += mSamplesPerNodes[nodeIdx];
		mSamplesPerNodes[nodeIdx] = 0;
	}

	// any samples to be deleted?
	return (sampleOffsets[oldSampleCount] != 0);
}

void Nodes::eraseSamplesInNodesInSitu(Samples &samples, const uint32 *nodeStates, const uint8 FLAG_OF_DOOM)
{
	vector<uint32> doomedSamples;
	uint32 nextSampleRemovalIdx = 0;
	uint32 newSampleStartIdx = 0;
	uint32 doomedSampleCount = 0;
	bool contiguousRemove = false;

	const size_t nodeCount = getCount();
	for (size_t nodeIdx = 0; nodeIdx < nodeCount; ++nodeIdx)
	{
		// update node
		mSampleStartIndices[nodeIdx] = newSampleStartIdx;
		if (0 == mSamplesPerNodes[nodeIdx])
			continue;

		// keep samples of this node?
		if (!(FLAG_OF_DOOM & nodeStates[nodeIdx]))
		{		
			newSampleStartIdx += mSamplesPerNodes[nodeIdx];
			nextSampleRemovalIdx += mSamplesPerNodes[nodeIdx];
			contiguousRemove = false;
			continue;
		}

		// The samples in this node are DOOMED!
		if (contiguousRemove)
		{
			doomedSamples.back() += mSamplesPerNodes[nodeIdx];
		}
		else
		{
			doomedSamples.push_back(nextSampleRemovalIdx);
			doomedSamples.push_back(mSamplesPerNodes[nodeIdx]);
		}

		nextSampleRemovalIdx += mSamplesPerNodes[nodeIdx];
		doomedSampleCount += mSamplesPerNodes[nodeIdx];
		mSamplesPerNodes[nodeIdx] = 0;
		contiguousRemove = true;
	}
	doomedSamples.push_back(samples.getCount());

	// Oblivion awaits.
	if (doomedSamples.size() > 1)
		samples.erase(doomedSamples, doomedSampleCount);
}

Nodes::Nodes(const Path &fileName)
{
	loadFromFile(fileName);
}

void Nodes::loadFromFile(const Path &fileName)
{
	// clean start & open file
	clear();
	File file(fileName, File::OPEN_READING, true, FILE_VERSION);

	// load node count & reserve memory
	uint32 nodeCount;
	file.read(&nodeCount, sizeof(uint32), sizeof(uint32), 1);
	resize(nodeCount);
	
	// read nodes
	const size_t nodeArraySize = sizeof(uint32) * nodeCount;
	file.read(mChildren.data(), nodeArraySize, sizeof(uint32), nodeCount);
	file.read(mParents.data(), nodeArraySize, sizeof(uint32), nodeCount);
	file.read(mSamplesPerNodes.data(), nodeArraySize, sizeof(uint32), nodeCount);
	file.read(mSampleStartIndices.data(), nodeArraySize, sizeof(uint32), nodeCount);
}

void Nodes::saveToFile(const Path &fileName) const
{
	// is there anything to save?
	if (mParents.empty())
		return;

	// create file with version & AABB, mOrderedSamples
	File file(fileName, File::CREATE_WRITING, true, FILE_VERSION);

	// save node count
	const uint32 nodeCount = getCount();
	file.write(&nodeCount, sizeof(uint32), 1);

	// save nodes
	file.write(mChildren.data(), sizeof(uint32), nodeCount);
	file.write(mParents.data(), sizeof(uint32), nodeCount);
	file.write(mSamplesPerNodes.data(), sizeof(uint32), nodeCount);
	file.write(mSampleStartIndices.data(), sizeof(uint32), nodeCount);
}

void Nodes::checkSampleIndicesOfNodes() const
{		
	#ifdef _DEBUG
		const uint32 sampleCount = Scene::getSingleton().getSamples().getCount();
		uint32 nextSampleIdx = 0;

		// check that sample counts fit to sample start indices
		const uint32 nodeCount = getCount();
		for (uint32 nodeIdx = 0; nodeIdx < nodeCount; ++nodeIdx)
		{
			const uint32 count = mSamplesPerNodes[nodeIdx];
			const uint32 startIdx = mSampleStartIndices[nodeIdx];

			assert(nextSampleIdx == startIdx);
			nextSampleIdx += count;
		}

		assert(nextSampleIdx == sampleCount);
	#endif // _DEBUG
}

void Nodes::checkSamplesOrder(const Scope &rootScope) const
{
	#ifdef _DEBUG
		const Scene &scene = Scene::getSingleton();
		const Samples &samples = scene.getSamples();
		const uint32 sampleCount = samples.getCount();

		for (uint32 sampleIdx = 0; sampleIdx < sampleCount; ++sampleIdx)
		{
			// get the node which contains sample
			Scope scope = rootScope;
			bool found = findContainer(scope, sampleIdx);
			if (!found)
				throw Exception("Sample wasn't entered into a container!");

			// sample idx must be within node's range
			const uint32 containerIdx = scope.getNodeIndex();
			const uint32 start = mSampleStartIndices[containerIdx];
			const uint32 end = start + mSamplesPerNodes[containerIdx];

			const bool inRange = (mSampleStartIndices[containerIdx] <= sampleIdx && sampleIdx < end);
			if (!inRange)
				throw Exception("Sample wasn't ordered in memory correctly!");
		}
	#endif // _DEBUG
}
