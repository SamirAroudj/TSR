/*
 * Copyright (C) 2018 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */

#include <algorithm>
#include "Math/MathHelper.h"
#include "SurfaceReconstruction/Geometry/FlexibleMesh.h"
#include "SurfaceReconstruction/Refinement/MeshDijkstra.h"

using namespace Math;
using namespace std;
using namespace SurfaceReconstruction;

const uint32 MeshDijkstra::INVALID_NODE = (uint32) -1;

MeshDijkstra::MeshDijkstra() :
	mMesh(NULL), mTriangleNormals(NULL),
	mVertexNeighborsOffsets(NULL), mVertexNeighbors(NULL)
{
	const uint32 count = 1000;
	mOrder.reserve(count);
	mVertices.reserve(count);
	mWorkingSet.reserve(count);
}

void MeshDijkstra::findVertices(const FlexibleMesh *mesh, const Vector3 *triangleNormals,
	const uint32 *vertexNeighbors, const uint32 *vertexNeighborsOffsets,
	const Vector3 &referenceNormal, const Vector3 &referencePosition,
	const Vector3 *startNormals, const uint32 *startVertices, const uint32 startVertexCount,
	const Real maxCosts, const Real maxAngleDifference, const Real angularCostsFactor)
{
	// set mesh data
	mMesh = mesh;
	mTriangleNormals = triangleNormals;
	mVertexNeighbors = vertexNeighbors;
	mVertexNeighborsOffsets = vertexNeighborsOffsets;

	// set search configuration data
	mAngularCostsFactor = angularCostsFactor;
	mMaxAngleDifference = maxAngleDifference;
	mMaxCosts = maxCosts;

	// clean start for MeshDijkstra search
	const Vector3 *positions = mMesh->getPositions();
	clear();
	buildStartSet(startNormals, startVertices, startVertexCount, referenceNormal, referencePosition);

	// for each iteration: put next best neighbor into mNeighbors & process its direct neighbors
	while (!mWorkingSet.empty())
	{
		// get best next sample & pop it from working set
		const uint32 nextBestLocalIdx = mWorkingSet[0];
		if (mVertices.at(nextBestLocalIdx).getCosts() > maxCosts)
			break;

		// remove next best from working set
		mOrder.push_back(nextBestLocalIdx);
		pop_heap(mWorkingSet.begin(), mWorkingSet.end(), MeshDijkstra::Comparer(*this));
		mWorkingSet.pop_back();

		// process all neighbor triangles of the triangle of vertex nextBest
		const uint32 nextBestGlobalIdx = mVertices.at(nextBestLocalIdx).getGlobalVertexIdx();
		const uint32 start = mVertexNeighborsOffsets[nextBestGlobalIdx];
		const uint32 end = mVertexNeighborsOffsets[nextBestGlobalIdx + 1];
		const uint32 neighborCount = end - start;
		const uint32 *neighbors = mVertexNeighbors + start;

		for (uint32 localNeighborIdx = 0; localNeighborIdx < neighborCount; ++localNeighborIdx)	
		{
			const uint32 globalNeighborIdx = neighbors[localNeighborIdx];
			processNeighbor(nextBestLocalIdx, globalNeighborIdx, referenceNormal, positions);
		}
	}
}

void MeshDijkstra::processNeighbor(const uint32 sourceLocalIdx, const uint32 targetGlobalIdx,
	const Vector3 &referenceNormal, const Vector3 *positions)
{
	// get next best ranged vertex and the costs between it and vertex targetGlobalIdx
	const RangedVertexIdx &source = mVertices.at(sourceLocalIdx);
	Vector3 n, pSource, pTarget;
	if (!getStepGeometry(n, pSource, pTarget, positions, source.getGlobalVertexIdx(), targetGlobalIdx))
		return;

	// was targetGlobalIdx already processed or is it new?
	const uint32 targetLocalIdx = (uint32) mVertices.size();
	const auto result = mVisitedVertices.insert(pair<uint32, uint32>(targetGlobalIdx, targetLocalIdx));

	// new / !visited if successfully inserted (result.second = true) -> add neighbor to samples & working set
	if (result.second)
	{
		// create ranged vertex index
		const RangedVertexIdx rangedVertexIdx(source.getCosts(), referenceNormal, n, pSource, pTarget,
			targetGlobalIdx, sourceLocalIdx, mMaxAngleDifference, mAngularCostsFactor);

		mVertices.push_back(rangedVertexIdx);
		addToWorkingSet(targetLocalIdx);
		return;
	}

	// vertex targetGlobalIdx was already visited - does it need an update?
	const uint32 localNeighborIdx = result.first->second;
	RangedVertexIdx &target = mVertices.at(localNeighborIdx);
	if (target.getCosts() < source.getCosts())
		return; 
	
	// neighbor must be in the working set since its costs are larger than source's costs
	const Real deltaCosts = RangedVertexIdx::getDeltaCosts(referenceNormal, n, pSource, pTarget, mMaxAngleDifference, mAngularCostsFactor);
	if (REAL_MAX == deltaCosts || isNaN(deltaCosts))
		return;
	
	// go over old predecessor or sourceLocalIdx?
	const Real newCosts = source.getCosts() + deltaCosts;
	if (target.getCosts() <= newCosts) 
		return;

	// better connection -> go over sourceLocalIdx instead of previously set predecessor
	target.setCosts(newCosts);
	target.setPredecessor(sourceLocalIdx);
	make_heap(mWorkingSet.begin(), mWorkingSet.end(), MeshDijkstra::Comparer(*this));
}

bool MeshDijkstra::getStepGeometry(Vector3 &n, Vector3 &p0, Vector3 &p1,
	const Vector3 *positions, const uint32 v0Idx, const uint32 v1Idx) const
{
	// edge exists?
	const uint32 edgeIdx = mMesh->getEdgeIndex(v0Idx, v1Idx);
	if (Edge::INVALID_IDX == edgeIdx)
		return false;

	// get & check edge triangles
	const Edge &edge = mMesh->getEdges()[edgeIdx];
	const uint32 *triangles = edge.getTriangleIndices();
	if (Triangle::INVALID_IDX == triangles[0] || Triangle::INVALID_IDX == triangles[1])
		return false;

	// get adjacent triangle normals
	const Vector3 &n0 = mTriangleNormals[triangles[0]];
	const Vector3 &n1 = mTriangleNormals[triangles[1]];
	
	// set normal & positions
	n = n0 + n1;
	n.normalize();
	p0 = positions[v0Idx];
	p1 = positions[v1Idx];
	return true;
}

void MeshDijkstra::buildStartSet(const Vector3 *startNormals, const uint32 *startVertices, const uint32 startVertexCount,
	const Vector3 &referenceNormal, const Vector3 &referencePosition)
{
	const Vector3 *positions = mMesh->getPositions();

	for (uint32 localVertexIdx = 0; localVertexIdx < startVertexCount; ++localVertexIdx)
	{
		// create node
		const uint32 globalVertexIdx = startVertices[localVertexIdx];
		if (Vertex::INVALID_IDX == globalVertexIdx)
			continue;

		const Vector3 &p = positions[globalVertexIdx];
		const Vector3 &n = startNormals[localVertexIdx];
		const Real costs = RangedVertexIdx::getDeltaCosts(referenceNormal, n, referencePosition, p, mMaxAngleDifference, mAngularCostsFactor);
		const uint32 newVertexIdx = (uint32) mVertices.size();

		// new or duplicate vertex index?
		pair<map<uint32, uint32>::iterator, bool> result = mVisitedVertices.insert(make_pair(globalVertexIdx, newVertexIdx));
		if (!result.second)
		{
			RangedVertexIdx &vIdx = mVertices[result.first->second];
			if (costs < vIdx.getCosts())
				vIdx.setCosts(costs);
			continue;
		}
		
		// new vertex idx add to vertices, working & visited set
		const RangedVertexIdx rangedVertexIdx(costs, globalVertexIdx, INVALID_NODE);
		mVertices.push_back(rangedVertexIdx);
		addToWorkingSet(newVertexIdx);
	}
}

//uint32 MeshDijkstra::getNextBest() const
//{
//	// find the vertex with the lowest cost
//	uint32 bestInWorkingSet = 0;
//	Real minimum = REAL_MAX;
//
//	// linear search to find the vertex in the working set with lowest costs
//	const uint32 count = (uint32) mWorkingSet.size();
//	for (uint32 i = 0; i < count; ++i)
//	{
//		// get candidate costs
//		const uint32 candidateIdx = mWorkingSet[i];
//		const RangedVertexIdx &candidate = mVertices[candidateIdx];
//		const Real costs = candidate.getCosts();
//
//		// better?
//		if (costs >= minimum)
//			continue;
//		bestInWorkingSet = i;
//		minimum = costs;
//	}
//
//	return bestInWorkingSet;
//}

void MeshDijkstra::clear()
{
	mOrder.clear();
	mVertices.clear();
	mVisitedVertices.clear();
	mWorkingSet.clear();
}
