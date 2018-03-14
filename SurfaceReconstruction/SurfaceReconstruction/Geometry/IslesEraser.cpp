/*
 * Copyright (C) 2018 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */

#include <atomic>
#include "Math/MathHelper.h"
#include "Platform/FailureHandling/Exception.h"
#include "SurfaceReconstruction/Geometry/FlexibleMesh.h"
#include "SurfaceReconstruction/Geometry/IslesEraser.h"

using namespace FailureHandling;
using namespace Math;
using namespace std;
using namespace SurfaceReconstruction;

const uint32 IslesEraser::INVALID_TRIANGLE_ISLAND = (uint32) -1;

IslesEraser::IslesEraser(const FlexibleMesh &originalMesh, const uint8 *vertexFlags, const uint8 considerFlags) :
	mVerticesToIsles(NULL), mEdgeOffsets(NULL), mTriangleOffsets(NULL), mVertexOffsets(NULL), mMinIsleSize(0),
	mMesh(originalMesh), mVertexFlags(vertexFlags), mConsiderFlags(considerFlags)
{
	clear();
	computeVerticesToIsles();
}

void IslesEraser::computeVerticesToIsles()
{
	cout << "Finding triangle isles. " << endl;

	// allocate memory
	const uint32 oldVertexCount = mMesh.getVertexCount();
	delete [] mVerticesToIsles;
	mVerticesToIsles = new atomic<uint32>[oldVertexCount];

	// init mVerticesToIsles: each vertex = its own isle
	#pragma omp parallel for
	for (int64 i = 0; i < oldVertexCount; ++i)
		mVerticesToIsles[i] = (ignore((uint32) i) ? INVALID_TRIANGLE_ISLAND : (uint32) i);

	mergeIsles();
	computeIsleSizes();
}

void IslesEraser::mergeIsles()
{
	// merge isles connected via edges iteratively
	while (connectNeighbors())
		propagateLowestIsleIDs();
}

bool IslesEraser::connectNeighbors()
{
	// mesh data
	const vector<uint32> *verticesToEdges = mMesh.getVerticesToEdges();
	const uint32 vertexCount = mMesh.getVertexCount();

	// connect neighbors
	bool connected = false;
	#pragma omp parallel for
	for (int64 i = 0; i < vertexCount; ++i)
	{
		// get & check vertex vertexIdx
		const uint32 vertexIdx = (uint32) i;
		const uint32 oldIsle = mVerticesToIsles[vertexIdx];	
		if (INVALID_TRIANGLE_ISLAND == mVerticesToIsles[vertexIdx])
			continue;

		// get isle & edges
		const vector<uint32> &edges = verticesToEdges[vertexIdx];
		const uint32 neighborCount = (uint32) edges.size();

		// find best within own local area	
		uint32 bestVertexToIsleID = oldIsle;
		bool betterNeighbor = false;

		for (uint32 localEdgeIdx = 0; localEdgeIdx < neighborCount; ++localEdgeIdx)
		{
			// get & check global neighbor index
			const uint32 globalEdgeIdx = edges[localEdgeIdx];
			const Edge &edge = mMesh.getEdge(globalEdgeIdx);
			const uint32 neighborVertexIdx = edge.getOtherVertex(vertexIdx);
			const uint32 neighborIsleID = mVerticesToIsles[neighborVertexIdx];

			// don't connect them?
			if (INVALID_TRIANGLE_ISLAND == neighborIsleID)
				continue;
			if (neighborIsleID >= bestVertexToIsleID)
				continue;

			bestVertexToIsleID = neighborIsleID;
			betterNeighbor = true;
		}

		// isle for connecting?
		if (!betterNeighbor)
			continue;

		connected = true;
		setInParallel(oldIsle, bestVertexToIsleID);
		setInParallel(vertexIdx, bestVertexToIsleID);
	}

	return connected;
}

void IslesEraser::propagateLowestIsleIDs()
{	
	// propagate connections within groups
	const uint32 vertexCount = mMesh.getVertexCount();

	for (bool propagating = true; propagating; )
	{
		propagating = false;
	
		#pragma omp parallel for
		for (int64 i = 0; i < vertexCount; ++i)
		{
			// get & check vertex
			const uint32 vertexIdx = (uint32) i;
			const uint32 oldIsle = mVerticesToIsles[vertexIdx];
			if (INVALID_TRIANGLE_ISLAND == oldIsle)
				continue;

			// propagate within group / "shorten links / steps" to the one with the lowest index
			const uint32 bestVertexToIsle = setInParallel(vertexIdx, mVerticesToIsles[oldIsle]);
			if (bestVertexToIsle < oldIsle)
				propagating = true;
		}
	}
}

uint32 IslesEraser::setInParallel(const uint32 vertexIdx, const uint32 bestVertexToIsle)
{
	// try to write bestVertexToIsle value until success or until some other thread wrote a better bestVertexToIsle
	atomic<uint32> &link = mVerticesToIsles[vertexIdx];
	uint32 current = link.load();
	if (bestVertexToIsle >= current)
		return current;

	while (!link.compare_exchange_weak(current, bestVertexToIsle))
		if (bestVertexToIsle >= current)
			return current;

	return bestVertexToIsle;
}

void IslesEraser::computeIsleSizes()
{
	const uint32 oldVertexCount = mMesh.getVertexCount();
	mIsleSizes.clear();

	// count: how many vertices per isle?
	for (uint32 vertexIdx = 0; vertexIdx < oldVertexCount; ++vertexIdx)
	{
		const uint32 isleID = mVerticesToIsles[vertexIdx];
		if (INVALID_TRIANGLE_ISLAND == isleID)
			continue;

		auto it = mIsleSizes.insert(make_pair(isleID, 0));
		uint32 &isleSize = it.first->second;
		++isleSize;
	}
}

bool IslesEraser::hasFoundTooSmallIsle(const uint32 minIsleSize)
{
	mMinIsleSize = minIsleSize;

	// any isle which is too small?
	for (map<uint32, uint32>::const_iterator isleIt = mIsleSizes.begin(); isleIt != mIsleSizes.end(); ++isleIt)
		if (isleIt->second < mMinIsleSize)
			return true;

	return false;
}

bool IslesEraser::isBad(const uint32 vertexIdx) const
{		
	if (ignore(vertexIdx))
		return false;

	const uint32 isleID = mVerticesToIsles[vertexIdx];
	const uint32 count = mIsleSizes.find(isleID)->second;
	return (count < mMinIsleSize);
}

bool IslesEraser::computeOffsets(const uint32 minIsleSize)
{
	mMinIsleSize = minIsleSize;
	if (!hasFoundTooSmallIsle(minIsleSize))
		return false;

	// allocate and compute offsets for filtering isolated data
	const uint32 oldVertexCount = mMesh.getVertexCount();
	const uint32 oldEdgeCount = mMesh.getEdgeCount();
	const uint32 oldTriangleCount = mMesh.getTriangleCount();
	mVertexOffsets = new uint32[oldVertexCount + 1];
	mEdgeOffsets = new uint32[oldEdgeCount + 1];
	mTriangleOffsets = new uint32[oldTriangleCount + 1];

	FlexibleMesh::computeOffsetsForFiltering(mVertexOffsets, mEdgeOffsets, mTriangleOffsets,
		oldVertexCount, oldEdgeCount, oldTriangleCount,
		mMesh.getVerticesToEdges(), mMesh.getEdges(), mMesh.getIndices(), *this);

	return true;
}

const std::vector<std::vector<uint32>> &IslesEraser::computeRingBordersOfDoomedIsles()
{
	//cout << "Finding borders of isles which will be deleted." << endl;
	assert(mTriangleOffsets);
	assert(!mIsleSizes.empty());

	// number of isles & mapping of  isle ID to index w.r.t. smallIslesBorders
	vector<uint32> mapping;
	mapping.reserve(100);
	for (map<uint32, uint32>::const_iterator it = mIsleSizes.begin(); it != mIsleSizes.end(); ++it)
	{
		if (it->second >= mMinIsleSize)
			continue;
		mapping.push_back(it->first);
	}

	// reserve memory
	mIslesBorders.clear();
	mIslesBorders.resize(mapping.size());

	gatherIsleBorders(mapping);
	FlexibleMesh::updateVertexIndices(mIslesBorders, mVertexOffsets);
	
	//cout << "Number of isles border rings: " << mIslesBorders.size() << endl;	
	return mIslesBorders;
}

void IslesEraser::gatherIsleBorders(const vector<uint32> &mapping)
{
	const uint32 triangleCount = mMesh.getTriangleCount();

	// gather isle borders
	#pragma omp parallel for
	for (int64 i = 0; i < triangleCount; ++i)
	{
		// kept triangle?
		if (mTriangleOffsets[i] == mTriangleOffsets[i + 1])
			continue;
		processDoomedTriangleForBorder(mapping, (uint32) i);
	}

	findBorderRings();
}

void IslesEraser::processDoomedTriangleForBorder(const vector<uint32> &mapping, const uint32 triangleIdx)
{
	// triangle triangleIdx is at isle border?
	const uint32 *triangle = mMesh.getTriangle(triangleIdx);
		
	// test edges: at border?
	for (uint32 edgeIdx = 0; edgeIdx < 3; ++edgeIdx)
	{
		// edge within the isle?
		const uint32 v0 = triangle[edgeIdx];
		const uint32 v1 = triangle[(edgeIdx + 1) % 3];
		if (!ignore(v0) || !ignore(v1))
			continue;

		// border edge!
		const uint32 interriorVertex = triangle[(edgeIdx + 2) % 3];
		const uint32 isleID = mVerticesToIsles[interriorVertex];
		addIsleBorderEdge(v0, v1, isleID, mapping);
		return;
	}
}

void IslesEraser::addIsleBorderEdge(const uint32 edgeVIdx0, const uint32 edgeVIdx1, const uint32 isleID, const vector<uint32> &mapping)
{
	// where to add the border edge?
	const uint32 borderCount = (uint32) mapping.size();
	for (uint32 borderIdx = 0; (borderIdx < borderCount); ++borderIdx)
	{
		if (isleID != mapping[borderIdx])
			continue;

		// add the edge to this isle
		vector<uint32> &targetBorder = mIslesBorders[borderIdx];
		#pragma omp critical (OMPPushBackHoleBorderEdge)
		{
			// IMPORTANT: keep this relative order of these two edge vertices - this is expected by findBorderRings
			targetBorder.push_back(edgeVIdx0);
			targetBorder.push_back(edgeVIdx1);
		}

		return;
	}

	throw Exception("Wrong implementation.");
}

void IslesEraser::findBorderRings()
{
	//cout << "Finding completely connected rings." << endl;

	for (uint32 borderIdx = 0; borderIdx < mIslesBorders.size(); ++borderIdx)
		removeTwoTimesPassedEdges(borderIdx);

	FlexibleMesh::findBorderRings(mIslesBorders);
}

void IslesEraser::removeTwoTimesPassedEdges(const uint32 borderIdx)
{
	vector<uint32> &edges = mIslesBorders[borderIdx];
	for (uint32 processedCount = 0; processedCount < edges.size(); processedCount += 2)
	{
		const uint32 processedCount1 = processedCount + 1;

		for (uint32 v0Idx = processedCount + 2; v0Idx < edges.size(); v0Idx += 2)
		{
			// same edge but passed along the other direction?
			const uint32 v1Idx = v0Idx + 1;
			if (edges[processedCount] != edges[v1Idx] || edges[processedCount1] != edges[v0Idx])
				continue;

			// remove both edges
			const uint32 leftCount = (uint32) edges.size();
			edges[v0Idx] = edges[leftCount - 2];
			edges[v1Idx] = edges[leftCount - 1];
			edges.pop_back();
			edges.pop_back();

			edges[processedCount] = edges[leftCount - 4];
			edges[processedCount1] = edges[leftCount - 3];
			edges.pop_back();
			edges.pop_back();

			// test next edge which is now at processedCount, processedCount + 1
			v0Idx = processedCount;
		}
	}
}

IslesEraser::~IslesEraser()
{
	clear();
}

void IslesEraser::clear()
{
	mIslesBorders.clear();
	mIsleSizes.clear();

	delete [] mVerticesToIsles;
	mVerticesToIsles = NULL;

	delete [] mEdgeOffsets;
	mEdgeOffsets = NULL;

	delete [] mTriangleOffsets;
	mTriangleOffsets = NULL;

	delete [] mVertexOffsets;
	mVertexOffsets = NULL;
}

const uint32 IslesEraser::getNewIndexCount() const
{
	const uint32 oldIndexCount = mMesh.getIndexCount();
	const uint32 doomedTriangleCount = mTriangleOffsets[oldIndexCount / 3];

	return oldIndexCount - 3 * doomedTriangleCount;
}

const uint32 IslesEraser::getNewVertexCount() const
{
	const uint32 oldVertexCount = mMesh.getVertexCount();
	return oldVertexCount - mVertexOffsets[oldVertexCount];
}

			//cout << "Ring completed:\n";
			//for (uint32 i = 0; i < ringSize; ++i)
			//	cout << mIslesBorders[borderIdx][i] << " ";
			//cout << endl;

	//// for debugging
	//{
	//	FlexibleMesh copy(mMesh);

	//	// color hole borders
	//	const Vector3 borderColor(1.0f, 1.0f, 1.0f);
	//	const uint32 holeCount = (uint32) mIslesBorders.size();
	//	
	//	for (uint32 holeIdx = 0; holeIdx < holeCount; ++holeIdx)
	//	{
	//		const vector<uint32> &hole = mIslesBorders[holeIdx];
	//		const uint32 holeSize = (uint32) hole.size();
	//		for (uint32 holeVertex = 0; holeVertex < holeSize; ++holeVertex)
	//		{
	//			const uint32 vertexIdx = hole[holeVertex];
	//			copy.setColor(borderColor, vertexIdx);
	//		}
	//	}

	//	copy.saveToFile("EraserWithColoredBorders", true, false);
	//}