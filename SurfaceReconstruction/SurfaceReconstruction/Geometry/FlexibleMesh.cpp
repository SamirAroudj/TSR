/*
 * Copyright (C) 2018 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */

#include "Math/MathHelper.h"
#include "Platform/Utilities/Array.h"
#include "SurfaceReconstruction/Geometry/Edge.h"
#include "SurfaceReconstruction/Geometry/FlexibleMesh.h"
#include "SurfaceReconstruction/Geometry/IslesEraser.h"
#include "SurfaceReconstruction/Geometry/Mesh.h"
#include "SurfaceReconstruction/Geometry/StaticMesh.h"

using namespace FailureHandling;
using namespace Math;
using namespace std;
using namespace Storage;
using namespace SurfaceReconstruction;
using namespace Utilities;

void FlexibleMesh::computeOffsetsForFiltering(uint32 *vertexOffsets, uint32 *edgeOffsets, uint32 *triangleOffsets,
	const uint32 vertexCount, const uint32 edgeCount, const uint32 triangleCount,
	const vector<uint32> *verticesToEdges, const Edge *edges, const uint32 *indices,
	const IVertexChecker &vertexChecker)
{
	memset(vertexOffsets, 0, sizeof(uint32) * (vertexCount + 1));
	memset(edgeOffsets, 0, sizeof(uint32) * (edgeCount + 1));
	memset(triangleOffsets, 0, sizeof(uint32) * (triangleCount + 1));

	uint32 *const doomedVertices = vertexOffsets + 1;
	uint32 *const doomedEdges = edgeOffsets + 1;
	uint32 *const doomedTriangles = triangleOffsets + 1;

	// find vertices, edges and triangles to be deleted
	#pragma omp parallel for
	for (int64 vertexIdx = 0; vertexIdx < vertexCount; ++vertexIdx)
	{
		// don't delete the vertex?
		if (!vertexChecker.isBad((uint32) vertexIdx))
			continue;

		// delete this vertex
		doomedVertices[vertexIdx] = 1;

		// find doomed edge and triangles
		const vector<uint32> &adjacentEdges = verticesToEdges[vertexIdx];
		const uint32 localEdgeCount = (uint32) adjacentEdges.size();
		for (uint32 localEdgeIdx = 0; localEdgeIdx < localEdgeCount; ++localEdgeIdx)
		{
			// doomed edge
			const uint32 globalEdgeIdx = adjacentEdges[localEdgeIdx];
			doomedEdges[globalEdgeIdx] = 1;

			// doomed triangles and doomed edges (owing to no left neighboring triangles)
			const Edge &edge = edges[globalEdgeIdx];
			const uint32 *triangles = edge.getTriangleIndices();
			for (uint32 sideIdx = 0; sideIdx < 2; ++sideIdx)
				if (Triangle::INVALID_INDEX != triangles[sideIdx]) // invalid neighbor triangle anyway?
					doomedTriangles[triangles[sideIdx]] = 1; // delete this triangle as its edge is deleted
		}
	}

	// check for edges which should be deleted since they have no neighbor triangles anymore
	#pragma omp parallel for
	for (int64 triangleIdx = 0; triangleIdx < triangleCount; ++triangleIdx)
	{
		if (!doomedTriangles[triangleIdx])
			continue;
		
		// check for edges which might have no neighbor triangle left when triangle triangleIdx is deleted
		const uint32 *triangle = indices + 3 * triangleIdx;
		for (uint32 triangleEdgeIdx = 0; triangleEdgeIdx < 3; ++triangleEdgeIdx)
		{
			// current edge: (v0, v1) -> find global index via v0's adjacent edges and v1 vertex index
			const uint32 v0 = triangle[triangleEdgeIdx];
			const uint32 v1 = triangle[(triangleEdgeIdx + 1) % 3];
			const vector<uint32> &adjacentEdges = verticesToEdges[v0];
		
			// get edge object for (v0, v1) and check its neighbors for existence
			const uint32 localEdgeCount = (uint32) adjacentEdges.size();
			for (uint32 localEdgeIdx = 0; localEdgeIdx < localEdgeCount; ++localEdgeIdx)
			{
				// correct global edge?
				const uint32 globalEdgeIdx = adjacentEdges[localEdgeIdx];
				const Edge &edge = edges[globalEdgeIdx];
				const uint32 candidateV1 = edge.getOtherVertex(v0);
				if (candidateV1 != v1)
					continue;

				// does the edge have 0 neighbor triangles?
				const uint32 otherTriangleIdx = edge.getOtherTriangle((uint32) triangleIdx);
				if (Triangle::INVALID_INDEX == otherTriangleIdx || doomedTriangles[otherTriangleIdx])
					doomedEdges[globalEdgeIdx] = 1;
			}
		}
	}

	// prefix sum on arrays of doom to get offsets for compaction
	prefixSum(vertexOffsets, edgeOffsets, triangleOffsets, vertexCount, edgeCount, triangleCount);
}

void FlexibleMesh::computeTriangleOffsets(uint32 *triangleOffsets, const uint32 *vertexOffsets, const uint32 *indices, const uint32 indexCount, const bool *additionalSkipTriangles)
{
	const uint32 triangleCount = indexCount / 3;

	// prefix sum to find offsets for compaction of triangles
	triangleOffsets[0] = 0;
	for (uint32 oldTriangleIdx = 0; oldTriangleIdx < triangleCount; ++oldTriangleIdx)
	{
		triangleOffsets[oldTriangleIdx + 1] = triangleOffsets[oldTriangleIdx];

		bool offsetTriangle = (additionalSkipTriangles ? additionalSkipTriangles[oldTriangleIdx] : false);
		if (!offsetTriangle)
		{
			// has one of the triangle vertices been filtered (if true -> filter (offset) the triangle)
			const uint32 oldTriangleStartOffset = 3 * oldTriangleIdx;
			for (uint32 edgeIdx = 0; edgeIdx < 3; ++edgeIdx)
			{
				const uint32 oldVertexIdx = indices[oldTriangleStartOffset + edgeIdx];
				if (vertexOffsets[oldVertexIdx] != vertexOffsets[oldVertexIdx + 1])
				{
					offsetTriangle = true;
					break;
				}
			}
		}

		if (offsetTriangle)
			++triangleOffsets[oldTriangleIdx + 1];
	}
}

void FlexibleMesh::findVertexNeighbors(vector<uint32> *vertexNeighbors, const uint32 *indices, const uint32 indexCount)
{
	// go over all triangles to go over all edges
	for (uint32 startOffset = 0; startOffset < indexCount; startOffset += 3, indices += 3)
	{
		// go over all 3 edges and properly extend the corresponding vertex neighborss
		uint32 edge[2] = { indices[0], indices[1] };
		for (uint32 edgeIdx = 0; edgeIdx < 3; ++edgeIdx)
		{
			addToVertexNeighbors(vertexNeighbors[edge[0]], edge[1]);
			addToVertexNeighbors(vertexNeighbors[edge[1]], edge[0]);

			// next triangle edge
			edge[0] = edge[1];
			edge[1] = indices[(edgeIdx + 2) % 3];
		}
	}
}

void FlexibleMesh::addToVertexNeighbors(vector<uint32> &vertexNeighbood, const uint32 vertexIdx)
{
	// already contained?
	const uint32 count = (uint32) vertexNeighbood.size();
	for (uint32 i = 0; i < count; ++i)
		if (vertexNeighbood[i] == vertexIdx)
			return;

	vertexNeighbood.push_back(vertexIdx);
}

FlexibleMesh::FlexibleMesh(const Path &fileName) :
	FlexibleMesh()
{
	loadFromFile(fileName);
}

void FlexibleMesh::loadFromFile(const Path &fileName)
{
	Mesh::loadFromFile(fileName);
	findAdjacencies();
}

FlexibleMesh::FlexibleMesh(const uint32 vertexCount, const uint32 indexCount) :
	FlexibleMesh()
{
	resize(vertexCount, indexCount);
}

FlexibleMesh::FlexibleMesh()
{
	clear();
}

FlexibleMesh::FlexibleMesh(const Mesh &m, IFlexibleMeshObserver *observer) :
	mColors(m.getColors(), m.getColors() + m.getVertexCount()),
	mNormals(m.getNormals(), m.getNormals() + m.getVertexCount()),
	mPositions(m.getPositions(), m.getPositions() + m.getVertexCount()),
	mScales(m.getScales(), m.getScales() + m.getVertexCount()),
	mIndices(m.getIndices(), m.getIndices() + m.getIndexCount())
{
	registerObserver(observer);
	findAdjacencies();
}

FlexibleMesh::FlexibleMesh(const FlexibleMesh &copy) :
	mVerticesToEdges(copy.mVerticesToEdges),
	mColors(copy.mColors),
	mNormals(copy.mNormals),
	mPositions(copy.mPositions),
	mScales(copy.mScales),
	mEdges(copy.mEdges),
	mIndices(copy.mIndices),
	mEdgeConflicts(copy.mEdgeConflicts)
{

}

FlexibleMesh::~FlexibleMesh()
{
	clear();
}

void FlexibleMesh::clear()
{
	Subject::clear();

	// clear data of vertices
	mVerticesToEdges.clear();
	mColors.clear();
	mNormals.clear();
	mPositions.clear();
	mScales.clear();
		
	// clear data of edges
	mEdges.clear();

	// clear triangle data
	mIndices.clear();
}

void FlexibleMesh::filterTriangles(uint32 *targetIndices, const uint32 *sourceIndices, const uint32 *triangleOffsets, const uint32 sourceIndexCount, const uint32 *vertexOffsets)
{
	const uint32 triangleCount = sourceIndexCount / 3;

	#pragma omp parallel for
	for (int64 i = 0; i < triangleCount; ++i)
	{
		// filter this triangle?
		const uint32 oldTriangleIdx = (uint32) i;
		if (triangleOffsets[oldTriangleIdx] != triangleOffsets[oldTriangleIdx + 1])
			continue;
		
		// copy triangle data
		const uint32 newTriangleIdx = oldTriangleIdx - triangleOffsets[oldTriangleIdx];
		const uint32 newStartOffset = 3 * newTriangleIdx;
		const uint32 oldStartOffset = 3 * oldTriangleIdx;
		for (uint32 cornerIdx = 0; cornerIdx < 3; ++cornerIdx)
		{
			const uint32 oldVertexIndex = sourceIndices[oldStartOffset + cornerIdx];
			targetIndices[newStartOffset + cornerIdx] = oldVertexIndex - vertexOffsets[oldVertexIndex];
		}
	}
}

void FlexibleMesh::computeScalesViaEdgeDistances()
{
	const int64 vertexCount = mPositions.size();
	#pragma omp parallel for
	for (int64 i = 0; i < vertexCount; ++i)
	{
		// get vertex data
		const uint32 vertexIdx = (uint32) i;
		const vector<uint32> &edges = mVerticesToEdges[vertexIdx];
		const uint32 edgeCount = (uint32) edges.size();
		if (0 == edgeCount)
			continue;

		// scale = mean distance to neighbors
		Real &scale = mScales[vertexIdx];
		scale = 0.0f;
		for (uint32 edgeIdx = 0; edgeIdx < edgeCount; ++edgeIdx)
		{
			// get vertex indices
			const Edge &edge = mEdges[edges[edgeIdx]];
			const uint32 v0 = edge.getVertexIndices()[0];
			const uint32 v1 = edge.getVertexIndices()[1];
			assert(v0 == vertexIdx || v1 == vertexIdx);

			// add distance to current neighbor
			const Vector3 diff = mPositions[v0] - mPositions[v1];
			scale += diff.getLength();
		}
		scale /= edgeCount;
	}
}

void FlexibleMesh::smoothByUmbrellaOp(Vector3 *vectorField, const vector<uint32> &vertices, const Real lambda)
{
	const uint32 vertexCount = (uint32) vertices.size();

	// compute movements
	for (uint32 i = 0; i < vertexCount; ++i)
	{
		const uint32 vertexIdx = vertices[i];
		vectorField[vertexIdx] = computeUmbrellaSmoothingMovement(vertexIdx, lambda);
	}

	// apply movements
	for (uint32 i = 0; i < vertexCount; ++i)
	{
		// get vertex data
		const uint32 vertexIdx = vertices[i];
		const Vector3 &position = getPosition(vertexIdx);
		const Vector3 &movement = vectorField[vertexIdx];

		// move vertex
		const Vector3 newPosition = position + movement;
		setPosition(newPosition, vertexIdx);
	}
}

Vector3 FlexibleMesh::computeUmbrellaSmoothingMovement(const uint32 vertexIdx, const Real lambda) const
{
	Vector3 weightedSum(0.0f, 0.0f, 0.0f);
	Real sumOfWeights = 0.0f;

	// umbrella smoothing operator: condider all direct neighbors of vertex vertexIdx
	const Vector3 &vertexPos = getPosition(vertexIdx);
	const vector<uint32> &edges = mVerticesToEdges[vertexIdx];
	const uint32 edgeCount = (uint32) edges.size();

	// compute a weighted mean position of the neighbors
	for (uint32 localEdgeIdx = 0; localEdgeIdx < edgeCount; ++localEdgeIdx)
	{
		// get neighbor vertex index and its position
		const uint32 globalEdgeIdx = edges[localEdgeIdx];
		const Edge &edge = mEdges[globalEdgeIdx];
		const uint32 neighborVertexIdx = edge.getOtherVertex(vertexIdx);
		const Vector3 &neighborPos = getPosition(neighborVertexIdx);

		// weight & weighted position
		const Real neighborWeight = Mesh::computeLaplacianWeight(vertexPos, neighborPos);
		const Vector3 weightedPosition = neighborPos * neighborWeight;

		// update sums
		sumOfWeights += neighborWeight;
		weightedSum += weightedPosition;
	}
	weightedSum /= sumOfWeights;

	// return final umbrella smoothing movement vector
	const Vector3 laplacian = (weightedSum - vertexPos);
	const Vector3 umbrellaMove = laplacian * lambda;
	return umbrellaMove;
}

void FlexibleMesh::updateVertexIndices(vector<vector<uint32>> &vertexSets, const uint32 *vertexOffsets)
{
	// update vertex indices -> valid after compaction / deletion / subtraction of vertex offsets
	const uint32 setCount = (uint32) vertexSets.size();

	#pragma omp parallel for
	for (int64 setIdx = 0; setIdx < setCount; ++setIdx)
	{
		// get hole
		vector<uint32> &set = vertexSets[setIdx];
		const uint32 setSize = (uint32) set.size();

		// shift vertex indices
		for (uint32 localVertexIdx = 0; localVertexIdx < setSize; ++localVertexIdx)
		{
			const uint32 oldIdx = set[localVertexIdx];
			const uint32 newIdx = oldIdx - vertexOffsets[oldIdx];
			set[localVertexIdx] = newIdx;
		}
	}
}

void FlexibleMesh::mergeEdges(vector<uint32> &edgesWithNewIndices, const vector<uint32> &edgeMergeCandidates)
{
	const uint32 candidateCount = (uint32) edgeMergeCandidates.size();
	if (0 == candidateCount)
		return;
	
	// offset arrays to define what elements are removed
	const uint32 oldVertexCount = getVertexCount();
	const uint32 oldEdgeCount = getEdgeCount();
	const uint32 oldTriangleCount = getTriangleCount();

	clearOffsets(oldVertexCount, oldEdgeCount, oldTriangleCount);
	memset(mVertexOffsets.data(), 0, sizeof(uint32) * (oldVertexCount + 1));
	memset(mEdgeOffsets.data(), 0, sizeof(uint32) * (oldEdgeCount + 1));
	memset(mTriangleOffsets.data(), 0, sizeof(uint32) * (oldTriangleCount + 1));
	
	// merging of edges, data structure compaction by deletion via prefix sum offsets
	mergeEdgesWithoutFiltering(mVertexOffsets.data() + 1, mEdgeOffsets.data() + 1, mTriangleOffsets.data() + 1, edgeMergeCandidates);

	for (uint32 edgeIdx = 0; edgeIdx < oldEdgeCount; ++edgeIdx)
	{
		const Edge &edge = mEdges[edgeIdx];
		if ((Triangle::INVALID_INDEX == edge.getTriangleIndices()[0] && Triangle::INVALID_INDEX == edge.getTriangleIndices()[1]) &&
			(Vertex::INVALID_INDEX != edge.getVertexIndices()[0] && Vertex::INVALID_INDEX != edge.getVertexIndices()[1]))
		{
			cerr << "This is not supposed to happen0: " << edgeIdx << " " << edge.getVertexIndices()[0] << " " << edge.getVertexIndices()[1] << endl;
		}
	}

	prefixSum(mVertexOffsets.data(), mEdgeOffsets.data(), mTriangleOffsets.data(), oldVertexCount, oldEdgeCount, oldTriangleCount);
	deleteGeometry(mVertexOffsets.data(), mEdgeOffsets.data(), mTriangleOffsets.data());
	
	const uint32 newEdgeCount = getEdgeCount();
	for (uint32 edgeIdx = 0; edgeIdx < newEdgeCount; ++edgeIdx)
	{
		const Edge &edge = mEdges[edgeIdx];
		if ((Triangle::INVALID_INDEX == edge.getTriangleIndices()[0] && Triangle::INVALID_INDEX == edge.getTriangleIndices()[1]) &&
			(Vertex::INVALID_INDEX != edge.getVertexIndices()[0] && Vertex::INVALID_INDEX != edge.getVertexIndices()[1]))
		{
			cerr << "This is not supposed to happen1: " << edgeIdx << " " << edge.getVertexIndices()[0] << " " << edge.getVertexIndices()[1] << endl;
		}
	}

	// edgesWithNewIndices = applyEdgeOffsets(edgeMergeCandidates)
	edgesWithNewIndices.clear();
	edgesWithNewIndices.reserve(edgeMergeCandidates.size());
	for (uint32 oldCandidateCount = 0; oldCandidateCount < candidateCount; ++oldCandidateCount)
	{
		const uint32 oldGlobalEdgeIdx = edgeMergeCandidates[oldCandidateCount];
		if (mEdgeOffsets[oldGlobalEdgeIdx] != mEdgeOffsets[oldGlobalEdgeIdx + 1])
			continue;

		const uint32 newGlobalEdgeIdx = oldGlobalEdgeIdx - mEdgeOffsets[oldGlobalEdgeIdx];
		edgesWithNewIndices.push_back(newGlobalEdgeIdx);
	}
}	

void FlexibleMesh::prefixSum(uint32 *vertexOffsets, uint32 *edgeOffsets, uint32 *triangleOffsets, const uint32 vertexCount, const uint32 edgeCount, const uint32 triangleCount)
{
	for (uint32 i = 0; i < vertexCount; ++i)
		vertexOffsets[i + 1] += vertexOffsets[i];
	for (uint32 i = 0; i < edgeCount; ++i)
		edgeOffsets[i + 1] += edgeOffsets[i];
	for (uint32 i = 0; i < triangleCount; ++i)
		triangleOffsets[i + 1] += triangleOffsets[i];
}

void FlexibleMesh::mergeEdgesWithoutFiltering(uint32 *globalDoomedVertices, uint32 *globalDoomedEdges, uint32 *globalDoomedTriangles,
	const vector<uint32> &mergingEdges)
{
	// merge each edge
	const uint32 mergingEdgesCount = (uint32) mergingEdges.size();
	for (uint32 localEdgeIdx = 0; localEdgeIdx < mergingEdgesCount; ++localEdgeIdx)
	{
		// get check global edge index
		const uint32 doomedEdgeIdx = mergingEdges[localEdgeIdx];
		if (globalDoomedEdges[doomedEdgeIdx])
			continue;

		mergeEdge(globalDoomedVertices, globalDoomedEdges, globalDoomedTriangles, doomedEdgeIdx);
	}
}

void FlexibleMesh::mergeEdge(uint32 *globalDoomedVertices, uint32 *globalDoomedEdges, uint32 *globalDoomedTriangles,
	const uint32 doomedEdgeIdx)
{
	// get required indices
	const Edge &doomedEdge = mEdges[doomedEdgeIdx];
	const uint32 doomedT[2] = { doomedEdge.getTriangleIndices()[0], doomedEdge.getTriangleIndices()[1] };
	const uint32 *doomedTris[2] = { getTriangle(doomedT[0]), getTriangle(doomedT[1]) };
	const uint32 doomedVertex = doomedEdge.getVertexIndices()[1];

	// get indices of kept vertices of triangles to be collapsed (keptV[0, 1, 2] stay, doomedV is deleted)
	uint32 keptV[3];
	keptV[0] = doomedEdge.getVertexIndices()[0];
	keptV[1] = Triangle::getOtherVertex(doomedTris[0], keptV[0], doomedVertex);
	keptV[2] = Triangle::getOtherVertex(doomedTris[1], keptV[0], doomedVertex);
	uint32 doomedE[3] = { doomedEdgeIdx, getEdgeIndex(doomedVertex, keptV[1]), getEdgeIndex(doomedVertex, keptV[2])};
	const uint32 keptE[2] = { getEdgeIndex(keptV[0], keptV[1]), getEdgeIndex(keptV[0], keptV[2])};

	if (isInvalidEdgeMerge(keptV[2], keptE, doomedVertex, doomedT))
		return;

	// inform observers & interpolate vertex
	const uint32 observerCount = (uint32) mObservers.size();
	for (uint32 observerIdx = 0; observerIdx < observerCount; ++observerIdx)
		mObservers[observerIdx]->onEdgeMerging(keptV[0], keptV[0], doomedVertex);
	interpolateVertex(keptV[2], keptV[2], doomedVertex);
	
	uint32 doomedV[3];
	uint32 doomedVCount = 1;
	doomedV[0] = doomedVertex;

	if (keptV[1] != keptV[2])
	{
		// update connectivity & mark doomed elements
		updateTriangleIndicesForEdgeMerge(keptV[0], doomedVertex, doomedT);
		updateLinksForEdgeMerge(keptV, keptE, doomedVertex, doomedT, doomedE);
	}
	else
	{
		// special case: 2 invalid triangles sharing the same 3 edges
		// keptV[2] == keptV[1] -> keptE[0] == keptE[1], doomedE[0] == doomedE[1], keptE should be removed
		updateLinksForEdgeMerge(doomedV, doomedVCount, doomedE, keptE, keptV);
	}
	
	markDoomedElements(globalDoomedVertices, globalDoomedEdges, globalDoomedTriangles, 
		doomedV, doomedVCount, doomedE, 3, doomedT, 2);
	
}

bool FlexibleMesh::isInvalidEdgeMerge(const uint32 keptV, const uint32 keptE[2], const  uint32 doomedV, const uint32 doomedT[2]) const
{		
	// edges adjacent to replaced vertex
	const vector<uint32> &doomedVEdges = mVerticesToEdges[doomedV];
	const uint32 doomedVEdgeCount = (uint32) doomedVEdges.size();

	// for each edge affected by replacing vertex doomedV with vertex keptV
	for (uint32 localDoomedEdgeIdx = 0; localDoomedEdgeIdx < doomedVEdgeCount; ++localDoomedEdgeIdx)
	{
		// get affected edge
		const uint32 movingEdgeIdx = doomedVEdges[localDoomedEdgeIdx];
		const Edge &movingEdge = mEdges[movingEdgeIdx];

		// for each adjacent triangle of the affected edge
		for (uint32 sideIdx = 0; sideIdx < 2; ++sideIdx)
		{
			// get triangle to move
			const uint32 triangleIdx = movingEdge.getTriangleIndices()[sideIdx];
			if (triangleIdx == doomedT[0] || triangleIdx == doomedT[1])
				continue;

			// does moving the triangle create an edge conflict? (more than 2 triangles connected to the same edge?)
			// is a triangle corner moved by the replacement?
			const uint32 *oldIndices = mIndices.data() + 3 * triangleIdx;
			uint32 movingCornerIdx = Triangle::getLocalVertexIdx(oldIndices, doomedV);
			if (Vertex::INVALID_INDEX == movingCornerIdx)
				continue;

			// edge conflict?
			const uint32 newEdgeNeighbors[2] = { oldIndices[(movingCornerIdx + 1) % 3], oldIndices[(movingCornerIdx + 2) % 3] };
			for (uint32 localEdgeIdx = 0; localEdgeIdx < 2; ++localEdgeIdx)
			{
				const uint32 globalEdgeIdx = getEdgeIndex(keptV, newEdgeNeighbors[localEdgeIdx]);
				if (Edge::INVALID_INDEX == globalEdgeIdx || keptE[0] == globalEdgeIdx || keptE[1] == globalEdgeIdx)
					continue;

				return true;
			}
		}
	}

	return false;
}

void FlexibleMesh::updateTriangleIndicesForEdgeMerge(const uint32 keptVertex, const uint32 doomedVertex, const uint32 doomedT[2])
{
	// update triangle indices: move from doomedVertex to keptV
	const vector<uint32> &doomedVEdges = mVerticesToEdges[doomedVertex];
	const uint32 doomedVEdgeCount = (uint32) doomedVEdges.size();

	for (uint32 localDoomedEdgeIdx = 0; localDoomedEdgeIdx < doomedVEdgeCount; ++localDoomedEdgeIdx)
	{
		const uint32 movingEdgeIdx = doomedVEdges[localDoomedEdgeIdx];
		const Edge &movingEdge = mEdges[movingEdgeIdx];

		for (uint32 sideIdx = 0; sideIdx < 2; ++sideIdx)
		{
			const uint32 triangleIdx = movingEdge.getTriangleIndices()[sideIdx];
			if (triangleIdx == doomedT[0] || triangleIdx == doomedT[1])
				continue;

			// update triangle corner indices
			uint32 *indices = mIndices.data() + 3 * triangleIdx;
			for (uint32 cornerIdx = 0; cornerIdx < 3; ++cornerIdx)
				if (doomedVertex == indices[cornerIdx])
					indices[cornerIdx] = keptVertex;
		}
	}
}

void FlexibleMesh::updateLinksForEdgeMerge(const uint32 keptV[3], const uint32 keptE[2],
	const uint32 doomedV, const uint32 doomedT[2], const uint32 doomedE[3])
{
	// remove doomed edges from vertices
	removeVertexToEdgeLink(keptV[0], doomedE[0]);
	removeVertexToEdgeLink(keptV[1], doomedE[1]);
	removeVertexToEdgeLink(keptV[2], doomedE[2]);
	removeVertexToEdgeLink(doomedV, doomedE[0]);
	removeVertexToEdgeLink(doomedV, doomedE[1]);
	removeVertexToEdgeLink(doomedV, doomedE[2]);
		
	moveEdges(keptV[0], doomedV);
	
	// update triangle links to go over the 2 merged / removed trianlges
	for (uint32 sideIdx = 0; sideIdx < 2; ++sideIdx)
	{
		Edge &keptEdge = mEdges[keptE[sideIdx]];

		const Edge &doomedEdge = mEdges[doomedE[sideIdx + 1]];
		const uint32 replaced = doomedT[sideIdx];
		const uint32 replacement = doomedEdge.getOtherTriangle(replaced);
		keptEdge.replaceTriangle(replaced, replacement);
	}
}

void FlexibleMesh::markDoomedElements(uint32 *globalDoomedVertices, uint32 *globalDoomedEdges, uint32 *globalDoomedTriangles,
	const uint32 *doomedVertices, const uint32 doomedVertexCount,
	const uint32 *doomedEdges, const uint32 doomedEdgeCount,
	const uint32 *doomedTriangles, const uint32 doomedTriangleCount)
{
	// doomed vertices
	for (uint32 localVertexIdx = 0; localVertexIdx < doomedVertexCount; ++localVertexIdx)
		globalDoomedVertices[doomedVertices[localVertexIdx]] = 1;

	// doomed edges
	for (uint32 localEdgeIdx = 0; localEdgeIdx < doomedEdgeCount; ++localEdgeIdx)
	{
		const uint32 globalIdx = doomedEdges[localEdgeIdx];
		globalDoomedEdges[globalIdx] = 1;

		Edge &edge = mEdges[globalIdx];
		edge.setTriangles(Triangle::INVALID_INDEX, Triangle::INVALID_INDEX);
		edge.setVertices(Vertex::INVALID_INDEX, Vertex::INVALID_INDEX);
	}

	// doomed triangles
	for (uint32 sideIdx = 0; sideIdx < doomedTriangleCount; ++sideIdx)
	{
		const uint32 globalIdx = doomedTriangles[sideIdx];
		globalDoomedTriangles[globalIdx] = 1;

		uint32 *indices = mIndices.data() + 3 * globalIdx;
		indices[0] = indices[1] = indices[2] = Vertex::INVALID_INDEX;
	}
}

void FlexibleMesh::updateLinksForEdgeMerge(uint32 doomedV[3], uint32 &doomedVCount, uint32 doomedE[3],
	const uint32 keptE[2], const uint32 keptV[2])
{
	// keptV[1] == keptV[2] -> keptE[0] == keptE[1], doomedE[1] == doomedE[2], keptE should be removed
	doomedE[2] = keptE[0];

	// remove doomed edges from vertices
	removeVertexToEdgeLink(doomedV[0], doomedE[0]);
	removeVertexToEdgeLink(doomedV[0], doomedE[1]);
	removeVertexToEdgeLink(keptV[0], doomedE[0]);
	removeVertexToEdgeLink(keptV[0], doomedE[2]);
	removeVertexToEdgeLink(keptV[1], doomedE[1]);
	removeVertexToEdgeLink(keptV[1], doomedE[2]);
		
	moveEdges(keptV[0], doomedV[0]);

	// disconnected vertices keptV[0], keptV[1]?
	for (uint32 i = 0; i < 2; ++i)
	{
		if (!mVerticesToEdges[keptV[i]].empty())
			continue;
		doomedV[doomedVCount] = keptV[i];
		++doomedVCount;
	}
}

void FlexibleMesh::moveEdges(const uint32 targetVertex, const uint32 sourceVertex)
{
	// reconnect remaining / kept edges adjacent to doomedV to keptV[2]
	vector<uint32> &sourceEdges = mVerticesToEdges[sourceVertex];
	vector<uint32> &targetEdges = mVerticesToEdges[targetVertex];

	const uint32 sourceCount = (uint32) sourceEdges.size();
	targetEdges.reserve(targetEdges.size() +  sourceCount);

	for (uint32 localEdgeIdx = 0; localEdgeIdx < sourceCount; ++localEdgeIdx)
	{
		const uint32 globalEdgeIdx = sourceEdges[localEdgeIdx];
		Edge &edge = mEdges[globalEdgeIdx];
		edge.replaceVertex(sourceVertex, targetVertex);
		targetEdges.push_back(globalEdgeIdx);
	}

	sourceEdges.clear();
}


void FlexibleMesh::subdivideEdges(const vector<uint32> &edges)
{
	// anything to split?
	const uint32 splitEdgeCount = (uint32) edges.size();
	if (0 == splitEdgeCount)
		return;
	
	// old element counts & reserve 
	const uint32 oldVertexCount = getVertexCount();
	const uint32 oldEdgeCount = getEdgeCount();
	const uint32 oldTriangleCount = getTriangleCount();

	// split edges
	reserveForEdgeSplits(splitEdgeCount);
	for (uint32 localEdgeIdx = 0; localEdgeIdx < splitEdgeCount; ++localEdgeIdx)
	{
		const uint32 globalEdgeIdx = edges[localEdgeIdx];
		subdivideEdge(globalEdgeIdx);
	}

	onNewElements(oldVertexCount, oldEdgeCount, oldTriangleCount);
}

void FlexibleMesh::reserveForEdgeSplits(const uint32 splitEdgeCount)
{
	const uint32 newVerticesPerSplit = 1;
	const uint32 newEdgesPerSplit = 3;
	const uint32 newTrianglesPerSplit = 2;

	// new element counts
	const uint32 newVertexCount = (uint32) (mPositions.size() + newVerticesPerSplit * splitEdgeCount);
	const uint32 newEdgeCount = (uint32) (mEdges.size() + newEdgesPerSplit * splitEdgeCount);
	const uint32 newIndexCount = (uint32) (mIndices.size() + splitEdgeCount * newTrianglesPerSplit * 3);

	// reserve memory for vertices, edges & triangles
	reserve(newVertexCount, newEdgeCount, newIndexCount);
}

void FlexibleMesh::subdivideEdge(const uint32 oldEdgeIdx)
{
	// get edge & new vertex count
	Edge &edge = mEdges[oldEdgeIdx];
	const uint32 newVertexIdx = getVertexCount();

	// indices of old adjacent traiangles & vertices
	const uint32 oldEV[2] = { edge.getVertexIndices()[0], edge.getVertexIndices()[1] };

	// create split vertex
	resize(newVertexIdx + 1, getIndexCount());
	interpolateEdgeSplitVertex(newVertexIdx, oldEV[0], oldEV[1]);

	// shrink edge edgeIdx between 2 old triangles to one of its halves
	removeVertexToEdgeLink(oldEV[1], oldEdgeIdx);
	mVerticesToEdges[newVertexIdx].push_back(oldEdgeIdx);
	mEdges[oldEdgeIdx].replaceVertex(oldEV[1], newVertexIdx);
	
	// for each edge side: create 2 smaller triangles
	const uint32 oldET[2] = { edge.getTriangleIndices()[0], edge.getTriangleIndices()[1] };
	for (uint32 edgeSideIdx = 0; edgeSideIdx < 2; ++edgeSideIdx)
	{
		// get & check triangle idx
		const uint32 oldTriangleIdx = oldET[edgeSideIdx];
		if (Triangle::INVALID_INDEX == oldTriangleIdx)
			continue;
			
		// old triangle & index of its vertex opposite to the split edge
		const uint32 *oldTriangle = getTriangle(oldTriangleIdx);
		uint32 triangleIndices[3] = { oldTriangle[0], oldTriangle[1], oldTriangle[2] };
		const uint32 oldOppoV = Triangle::getOtherVertex(triangleIndices, oldEV[0], oldEV[1]);

		// 2 smaller triangles = old one shrinked + new neighbor triangle
		shrinkOldEdgeSplitTriangles(newVertexIdx, oldEdgeIdx, oldEV, oldOppoV, oldTriangleIdx);
		addNewEdgeSplitTriangles(triangleIndices, newVertexIdx, oldEV[0]);
	}

	//checkConnectivity("FlexibleMesh::subdivideEdge");
}

void FlexibleMesh::shrinkOldEdgeSplitTriangles(const uint32 newVertexIdx, const uint32 oldEdgeIdx,
	const uint32 oldEV[2], const uint32 oldOppoV, const uint32 oldTriangleIdx)
{
	// remove invalid edge connectivity of the old triangle and its old neighbor triangle adjacent to oldEV[1]
	const uint32 invalidOuterEdge = getEdgeIndex(oldEV[1], oldOppoV);
	mEdges[invalidOuterEdge].replaceTriangle(oldTriangleIdx, Triangle::INVALID_INDEX);

	// update indices create 2 new edges
	// reassign / shrink two old triangles
	uint32 *t = mIndices.data() + 3 * oldTriangleIdx;
	for (uint32 cornerIdx = 0; cornerIdx < 3; ++cornerIdx)
	{
		if (t[cornerIdx] != oldEV[1])
			continue;

		t[cornerIdx] = newVertexIdx;
		break;
	}
		
	// create edge between old triangle and not yet existing neighbor split triangle
	createEdge(newVertexIdx, oldOppoV, oldTriangleIdx);
}

void FlexibleMesh::addNewEdgeSplitTriangles(uint32 oldTriangle[3], const uint32 newIdx, const uint32 replacedIdx)
{
	// new triangle within oldTriangleIndices but with one replaced index
	for (uint32 cornerIdx = 0; cornerIdx < 3; ++cornerIdx)
	{
		if (oldTriangle[cornerIdx] != replacedIdx)
			continue;

		oldTriangle[cornerIdx] = newIdx;
		break;
	}

	addTriangle(oldTriangle);
}

void FlexibleMesh::subdivideTriangles(vector<uint32> &doomedOnes, vector<uint32> *possiblyDoomedOnes)
{
	// nothing to do?
	const uint32 doomedCount = (uint32) doomedOnes.size();
	if (0 == doomedCount)
		return;

	// reserve memory vertex & index buffer & possibly doomed triangles
	reserveFor4Splits(doomedCount);

	if (possiblyDoomedOnes)
	{
		const uint32 splitTrianglesPerDoomed = 16; // normal (& maximum count) case: 4 triangles -> 16 triangles
		possiblyDoomedOnes->clear();
		possiblyDoomedOnes->reserve(splitTrianglesPerDoomed * doomedCount);
	}

	// subdivide triangles
	uint32 oldNeighborTriangles[3];
	while (!doomedOnes.empty())
	{
		subdivideTriangle(doomedOnes.back(), oldNeighborTriangles, possiblyDoomedOnes);

		// remove split traingles from doomedOnes to avoid spliting multiple times
		doomedOnes.pop_back();
		for (uint32 i = 0; i < 3; ++i)
		{
			if (Triangle::INVALID_INDEX == oldNeighborTriangles[i])
				continue;

			Array<uint32>::deleteFirstBySwapWithBack(doomedOnes, oldNeighborTriangles[i]);
		}
	}
	cout << "subdivideTriangles" << endl;
	checkEdges();
}

void FlexibleMesh::reserveFor4Splits(const uint32 splitTriangleCount)
{
	// for each split triangle: how many new elements?
	const uint32 newVerticesPerDoomed = 6;
	const uint32 newEdgesPerDoomed = 15;
	const uint32 newTrianglesPerDoomed = 12; // if a triangle is split into 4 and if t-vertices are avoided then there can be up to 12 additional triangles

	// new element counts
	const uint32 newVertexCount = (uint32) (mPositions.size() + splitTriangleCount * newVerticesPerDoomed);
	const uint32 newEdgeCount = (uint32) (mEdges.size() + splitTriangleCount * newEdgesPerDoomed);
	const uint32 newIndexCount = (uint32) (mIndices.size() + splitTriangleCount * newTrianglesPerDoomed * 3);

	// reserve memory for vertices, edges & triangles
	reserve(newVertexCount, newEdgeCount, newIndexCount);
}

void FlexibleMesh::subdivideTriangle(const uint32 triangleIdx, uint32 oldNeighborTriangles[3], vector<uint32> *splitTriangles)
{
	if (Triangle::isInvalidIndex(triangleIdx))
	{
		assert(false);
		throw Exception("FlexibleMesh::subdivideTriangle: Invalid triangle index for subdivision.");
	}
	
	const uint32 oldTriangleCount = (uint32) getTriangleCount();
	uint32 newVertexIndices[6];
	uint32 oldVertexIndices[6];

	gatherIndicesForSplit(newVertexIndices, oldVertexIndices, oldNeighborTriangles, triangleIdx);
	
	createNewVerticesForSubdivision(newVertexIndices, oldVertexIndices);
	createSplitTriangles(newVertexIndices, oldVertexIndices, triangleIdx, oldNeighborTriangles);
	gatherSplitTriangles(splitTriangles, triangleIdx, oldNeighborTriangles, oldTriangleCount);
}

void FlexibleMesh::gatherIndicesForSplit(uint32 newVertexIndices[6], uint32 oldVertexIndices[6], uint32 oldNeighborTriangles[3],
	const uint32 triangleIdx) const
{
	// partition triangle triangleIdx into 4 smaller triangles and also partition neighbor triangles into 4 triangles to avoid t-vertices
	getTriangleNeighbors(oldNeighborTriangles, triangleIdx);

	// get old triangle's vertex indices and indices of vertices opposite to the split edges of old triangle
	// central triangle indices
	const uint32 *oldTriangleStart = mIndices.data() + 3 * triangleIdx;
	for (uint32 i = 0; i < 3; ++i)
		oldVertexIndices[i] = oldTriangleStart[i];

	// indices of vertices opposite to the edges of the old central triangle
	for (uint32 i = 0; i < 3; ++i)
	{
		uint32 &oldIdx = oldVertexIndices[i + 3];
		oldIdx = Vertex::INVALID_INDEX;
		if (Triangle::isInvalidIndex(oldNeighborTriangles[i]))
			continue;

		const uint32 *neighborTriangle = mIndices.data() + oldNeighborTriangles[i] * 3;
		const uint32 edgeVertex0 = oldVertexIndices[i];
		const uint32 edgeVertex1 = oldVertexIndices[(i + 1) % 3];

		oldIdx = Triangle::getOtherVertex(neighborTriangle, edgeVertex0, edgeVertex1);
	}

	// indices of new vertices from edge splits
	const uint32 oldVertexCount = getVertexCount();
	for (uint32 i = 0; i < 6; ++i)
		newVertexIndices[i] = oldVertexCount + i;
}

void FlexibleMesh::createNewVerticesForSubdivision(const uint32 newVertexIndices[6], const uint32 oldVertexIndices[6])
{
	// reserve memory depending on vertex count
	const uint32 newVertexCount = (uint32) (mPositions.size() + 6);
	resize(newVertexCount, getIndexCount());

	// 3 split vertices within the inner triangle
	for (uint32 i = 0; i < 3; ++i)
		interpolateEdgeSplitVertex(newVertexIndices[i], oldVertexIndices[i], oldVertexIndices[(i + 1) % 3]);

	// 3 split vertices at edges from previous 3 vertices and inner triangles 3 outer opposite corner vertices
	for (uint32 i = 0; i < 3; ++i)
		interpolateEdgeSplitVertex(newVertexIndices[i + 3], newVertexIndices[i], oldVertexIndices[i + 3]);
}

void FlexibleMesh::interpolateEdgeSplitVertex(const uint32 targetIdx, const uint32 v0, const uint32 v1, const Real f0)
{
	interpolateVertex(targetIdx, v0, v1, f0);

	const uint32 count = getObserverCount();
	for (uint32 i = 0; i < count; ++i)
		mObservers[i]->onEdgeSplitVertex(targetIdx, v0, v1);
}

void FlexibleMesh::interpolateVertex(const uint32 targetIdx, const uint32 v0, const uint32 v1, const Real f0)
{
	const Real f1 = 1.0f - f0;

	mPositions[targetIdx] = mPositions[v0] * f0 + mPositions[v1] * f1;
	mColors[targetIdx] = mColors[v0] * f0 + mColors[v1] * f1;

	mNormals[targetIdx] = mNormals[v0] * f0 + mNormals[v1] * f1;
	mNormals[targetIdx].normalize();

	mScales[targetIdx] = mScales[v0] * f0 + mScales[v1] * f1;
}

void FlexibleMesh::createSplitTriangles(const uint32 newVertexIndices[6], const uint32 oldVertexIndices[6],
	const uint32 triangleIdx, const uint32 oldNeighborTriangles[3])
{
	// replace central triangle and add 3 new triangles within old central triangle 
	replaceCentralTriangle(newVertexIndices, oldVertexIndices, triangleIdx);
	for (uint32 i = 0; i < 3; ++i)
	{
		const uint32 indices[3] = { oldVertexIndices[i], newVertexIndices[i], newVertexIndices[(i + 2) % 3] };
		addTriangle(indices);
	}

	// replace old neighbors by 4 new smaller triangles
	for (uint32 i = 0; i < 3; ++i)
	{
		const uint32 neighborIdx = oldNeighborTriangles[i];
		if (Triangle::isInvalidIndex(neighborIdx))
			continue;

		replaceNeighborTriangle(neighborIdx,
			oldVertexIndices[i], newVertexIndices[i], oldVertexIndices[(i + 1) % 3],
			newVertexIndices[i + 3], oldVertexIndices[i + 3]);
	}
}

void FlexibleMesh::replaceCentralTriangle(const uint32 newVertexIndices[3], const uint32 oldVertexIndices[3], const uint32 triangleIdx)
{
	// replace central triangle with smaller triangle between new split vertices
	uint32 *targetTriangle = mIndices.data() + 3 * triangleIdx;
	targetTriangle[0] = newVertexIndices[0];
	targetTriangle[1] = newVertexIndices[1];
	targetTriangle[2] = newVertexIndices[2];

	// reset connectivity of triangle triangleIdx
	reassignOldEdges(newVertexIndices, oldVertexIndices, triangleIdx);
}

void FlexibleMesh::reassignOldEdges(const uint32 newVertexIndices[3], const uint32 oldVertexIndices[3], const uint32 triangleIdx)
{
	for (uint32 localEdgeIdx = 0; localEdgeIdx < 3; ++localEdgeIdx)
	{
		const uint32 temp = (localEdgeIdx + 1) % 3;
		const uint32 newV0 = newVertexIndices[localEdgeIdx];
		const uint32 newV1 = newVertexIndices[temp];
		const uint32 oldV0 = oldVertexIndices[localEdgeIdx];
		const uint32 oldV1 = oldVertexIndices[temp];
		const uint32 globalEdgeIdx = getEdgeIndex(oldV0, oldV1);

		// remove edge from old triangle
		removeVertexToEdgeLink(oldV0, globalEdgeIdx);
		removeVertexToEdgeLink(oldV1, globalEdgeIdx);

		// completely reassign edge globalEdgeIdx
		Edge &edge = mEdges[globalEdgeIdx];
		edge.setTriangles(triangleIdx, Triangle::INVALID_INDEX);
		edge.setVertices(newV0, newV1);

		mVerticesToEdges[newV0].push_back(globalEdgeIdx);
		mVerticesToEdges[newV1].push_back(globalEdgeIdx);
	}
}

void FlexibleMesh::removeVertexToEdgeLink(const uint32 vertexIdx, const uint32 globalEdgeIdx)
{
	std::vector<uint32> &edges = mVerticesToEdges[vertexIdx];
	const size_t idx = Array<uint32>::deleteFirstBySwapWithBack(edges, globalEdgeIdx);
	if ((size_t) -1 != idx)
		return;

	// wrong usage
	string text = "Invalid call of FlexibleMesh::removeEdge()";
	cerr << text << endl;
	throw Exception(text);
}

void FlexibleMesh::replaceNeighborTriangle(const uint32 triangleIdx,
	const uint32 splitEdgeV0, const uint32 splitEdgeCenterVertex, const uint32 splitEdgeV1,
	const uint32 newCenterVertexIdx, const uint32 oldOppositeSplitVertex)
{
	if (Triangle::isInvalidIndex(triangleIdx))
		return;

	// get vertex indices & edges
	uint32 *triangle = mIndices.data() + 3 * triangleIdx;
	const uint32 oldTriangleIndices[3] = { triangle[0], triangle[1], triangle[2] };
	uint32 edgeIndices[3];
	getEdgeIndices(edgeIndices, triangleIdx);

	// keep connectivity to one neighbor triangle 
	uint32 replacedVertex = Vertex::INVALID_INDEX;
	uint32 keptEdgeIdx = Edge::INVALID_INDEX;
	for (uint32 localEdgeIdx = 0; localEdgeIdx < 3; ++localEdgeIdx)
	{
		// get & check edge index
		const uint32 edgeIdx = edgeIndices[localEdgeIdx];
		if (Edge::isInvalidIndex(edgeIdx))
			continue;

		// get & check edge
		Edge &edge = mEdges[edgeIdx];
		assert(edge.getTriangleIndices()[0] == triangleIdx || edge.getTriangleIndices()[1] == triangleIdx);

		// only keep connectivity to one neighbor - other two neighbors will be created later
		if (Edge::INVALID_INDEX != keptEdgeIdx)
		{
			// invalidate other edge link
			edge.replaceTriangle(triangleIdx, Triangle::INVALID_INDEX);
			continue;
		}

		// keep this edge and replace its opposite vertex to shrink the triangle
		keptEdgeIdx = localEdgeIdx;

		uint32 temp = (localEdgeIdx + 2) % 3;
		replacedVertex = triangle[temp];
		triangle[temp] = newCenterVertexIdx;
	}
		
	for (uint32 localEdgeIdx = 0; localEdgeIdx < 3; ++localEdgeIdx)
		if (localEdgeIdx != keptEdgeIdx)
			addEdge(triangleIdx, localEdgeIdx);

	// 2 triangles adjacent to the split edge
	addTriangle(splitEdgeCenterVertex, splitEdgeV0, newCenterVertexIdx);
	addTriangle(splitEdgeV1, splitEdgeCenterVertex, newCenterVertexIdx);

	// triangle adjacent oldOppositeSplitVertex and opposite to the kept edge
	uint32 newTriangle[3];
	for (uint32 i = 0; i < 3; ++i)
	{
		newTriangle[i] = oldTriangleIndices[i];
		if (newTriangle[i] != replacedVertex && newTriangle[i] != oldOppositeSplitVertex)
			newTriangle[i] = newCenterVertexIdx;
	}
	addTriangle(newTriangle);
}


bool FlexibleMesh::getAdjacentTriangleNormals(Math::Vector3 &n0, Math::Vector3 &n1, const uint32 edgeVertexIdx0, const uint32 edgeVertexIdx1) const
{
	// does the edge exist?
	const uint32 edgeIdx = getEdgeIndex(edgeVertexIdx0, edgeVertexIdx1);
	if (Edge::INVALID_INDEX == edgeIdx)
		return false;

	// get the edge & its triangle indices
	const Edge &edge = mEdges[edgeIdx];
	const uint32 *indices = edge.getTriangleIndices();

	// set n0
	if (Triangle::isInvalidIndex(indices[0]))
	{
		n0.set(0.0f, 0.0f, 0.0f);
	}
	else
	{
		const uint32 *triangle = getTriangle(indices[0]);
		Math::computeTriangleNormal(n0, mPositions[triangle[0]], mPositions[triangle[1]], mPositions[triangle[2]]);
	}

	// set n1
	if (Triangle::isInvalidIndex(indices[1]))
	{
		n1.set(0.0f, 0.0f, 0.0f);
	}
	else
	{
		const uint32 *triangle = getTriangle(indices[1]);
		Math::computeTriangleNormal(n1, mPositions[triangle[0]], mPositions[triangle[1]], mPositions[triangle[2]]);
	}

	return true;
}

uint32 FlexibleMesh::getEdgeIndex(const uint32 vertexIdx0, const uint32 vertexIdx1) const
{
	// todo optimize this?
	const vector<uint32> &edges = mVerticesToEdges[vertexIdx0];
	const uint32 edgeCount = (uint32) edges.size();

	// find the proper edge
	for (uint32 i = 0; i < edgeCount; ++i)
	{
		const uint32 edgeIdx = edges[i];
		const Edge &edge = mEdges[edgeIdx];
		if (edge.isAdjacentToVertex(vertexIdx1))
			return edgeIdx;
	}

	return Edge::INVALID_INDEX;
}

void FlexibleMesh::getEdgeIndices(uint32 edgeIndices[3], const uint32 triangleIdx) const
{
	const uint32 *t = mIndices.data() + 3 * triangleIdx;

	edgeIndices[0] = getEdgeIndex(t[0], t[1]);
	edgeIndices[1] = getEdgeIndex(t[1], t[2]);
	edgeIndices[2] = getEdgeIndex(t[2], t[0]);
}

void FlexibleMesh::findAdjacencies()
{
	clearAdjacencies();
	
	//cout << "Finding adjacency information of mesh elements." << endl;

	// clear & reserve memory
	const uint32 vertexCount = (uint32) mPositions.size();
	const uint32 triangleCount = (uint32) mIndices.size() / 3;

	mVerticesToEdges.resize(vertexCount);
	mEdges.reserve(2 * vertexCount);

	// for each triangle: add connectivity / edges
	for (uint32 triangleIdx = 0; triangleIdx < triangleCount; ++triangleIdx)	
		for (uint32 edgeIdx = 0; edgeIdx < 3; ++edgeIdx) // process each triangle edge: new or already known?
			addEdge(triangleIdx, edgeIdx);

	//cout << "Computed connectivity of mesh elements." << endl;
}

void FlexibleMesh::clearAdjacencies()
{
	// clean adjacency information (except triangles themselves)
	const uint32 count = (uint32) mVerticesToEdges.size();
	for (uint32 vertexIdx = 0; vertexIdx < count; ++vertexIdx)
		mVerticesToEdges[vertexIdx].clear();

	mEdges.clear();
}

void FlexibleMesh::addTriangle(const uint32 indices[3])
{
	// add triangle neighbor
	const uint32 newTriangleIdx = getTriangleCount();
	const uint32 newTriangleCount = newTriangleIdx + 1;

	// add vertex indices which represent the new triangle
	mIndices.reserve(mIndices.size() + 3);
	for (uint32 cornerIdx = 0; cornerIdx < 3; ++cornerIdx)
		mIndices.push_back(indices[cornerIdx]);

	// add connectivity
	for (uint32 edgeIdx = 0; edgeIdx < 3; ++edgeIdx)
		addEdge(newTriangleIdx, edgeIdx);
}

void FlexibleMesh::addEdge(const uint32 triangleIdx, const uint32 localEdgeIdx)
{
	/// get ordered indices and edge index if it is new
	const uint32 *triangle = mIndices.data() + 3 * triangleIdx;
	const uint32 a = triangle[localEdgeIdx];
	const uint32 b = triangle[(localEdgeIdx + 1) % 3];
	const uint32 v0 = (a < b ? a : b);
	const uint32 v1 = (a < b ? b : a);
	
	// Does the edge <v0, v1> not exist?
	const uint32 globalEdgeIdx = getEdgeIndex(v0, v1);
	if (Edge::isInvalidIndex(globalEdgeIdx))
	{		
		createEdge(v0, v1, triangleIdx);
		return;
	}

	// each triangle can have up to 2 neighbor triangles
	if (!mEdges[globalEdgeIdx].hasTwoNeighbors())
	{	
		Edge &edge = mEdges[globalEdgeIdx];
		edge.addTriangle(triangleIdx);
		return;
	}

	// conflict
	addEdgeConflict(globalEdgeIdx, triangleIdx);
}

uint32 FlexibleMesh::createEdge(const uint32 v0, const uint32 v1, const uint32 triangleIdx)
{
	// new edge -> add it to the vertices' lists of links & edges
	const uint32 nextGlobalEdgeIdx = (uint32) mEdges.size();
	mVerticesToEdges[v0].push_back(nextGlobalEdgeIdx);
	mVerticesToEdges[v1].push_back(nextGlobalEdgeIdx);

	// create and add edge itself
	Edge edge(v0, v1, triangleIdx);
	mEdges.push_back(edge);

	return nextGlobalEdgeIdx;
}

void FlexibleMesh::addEdgeConflict(const uint32 edgeIdx, const uint32 newTriangleIdx)
{
	const Edge &edge = mEdges[edgeIdx];
	const uint32 conflictCount = (uint32) mEdgeConflicts.size();
	for (uint32 i = 0; i < conflictCount; ++i)
	{
		// does conflict edge already exist?
		EdgeConflict &conflict = mEdgeConflicts[i];
		if (edgeIdx != conflict.mEdgeIdx)
			continue;

		// found conflict edge - add triangle
		conflict.mTriangles.push_back(newTriangleIdx);
		return;
	}

	mEdgeConflicts.push_back(EdgeConflict(edge, edgeIdx));
	mEdgeConflicts.back().mTriangles.push_back(newTriangleIdx);

	// debug output for edge and conflict triangle
	cerr << "Edge conflict!\n";
	cerr << "New triangle " << newTriangleIdx << ", Edge " << edgeIdx << "\n";
	cerr << edge << "\n";

	// output adjacent triangle vertices
	for (uint32 sideIdx = 0; sideIdx < 2; ++sideIdx)
	{
		const uint32 *t = getTriangle(edge.getTriangleIndices()[sideIdx]);
		cerr << "Triangle " << sideIdx << ": (" << t[0] << "," << t[1] << "," << t[2] << ")\n";
	}
	cerr << flush;

	throw Exception("BOOM");
}

void FlexibleMesh::getTriangleNeighbors(uint32 neighbors[3], const uint32 triangleIdx) const
{
	const uint32 *indices = mIndices.data() + 3 * triangleIdx;
	for (uint32 localEdgeIdx = 0; localEdgeIdx < 3; ++localEdgeIdx)
	{
		const uint32 v0 = indices[localEdgeIdx];
		const uint32 v1 = indices[(localEdgeIdx + 1) % 3];
		const uint32 globalEdgeIdx = getEdgeIndex(v0, v1);
		const Edge &edge = mEdges[globalEdgeIdx];
		
		neighbors[localEdgeIdx] = edge.getOtherTriangle(triangleIdx);
	}
}

void FlexibleMesh::addVertex(const Math::Vector3 &color, const Math::Vector3 &normal, const Math::Vector3 &position, const Real scale)
{
	const uint32 vertexIndex = getVertexCount();
	resize(vertexIndex + 1, getIndexCount());
	set(color, normal, position, scale, vertexIndex);
}

void FlexibleMesh::gatherSplitTriangles(vector<uint32> *splitTriangles,
	const uint32 triangleIdx, const uint32 oldNeighborTriangles[3], const uint32 oldTriangleCount) const
{
	// gather indices of triangles which were created by the 4 splits
	if (!splitTriangles)
		return;

	const uint32 newTriangleCount = getTriangleCount();
	splitTriangles->reserve(splitTriangles->size() + (newTriangleCount - oldTriangleCount));

	splitTriangles->push_back(triangleIdx);
	for (uint32 i = 0; i < 3; ++i)
		if (!Triangle::isInvalidIndex(oldNeighborTriangles[i]))
			splitTriangles->push_back(oldNeighborTriangles[i]);
	for (uint32 splitTriangleIdx = oldTriangleCount; splitTriangleIdx < newTriangleCount; ++splitTriangleIdx)
		splitTriangles->push_back(splitTriangleIdx);
}

void FlexibleMesh::getVertexNeighbors(vector<uint32> &neighbors, vector<uint32> &offsets) const
{
	// clean start
	neighbors.clear();
	offsets.clear();

	// compute offsets for quick access of actual neighbors
	const uint32 vertexCount = getVertexCount();
	offsets.resize(vertexCount + 1);

	// prefix sum on neighbors sizes
	uint32 totalNeighborCount = 0;
	for (uint32 vertexIdx = 0; vertexIdx < vertexCount; ++vertexIdx)
	{
		offsets[vertexIdx] = totalNeighborCount;

		const vector<uint32> &neighbors =  mVerticesToEdges[vertexIdx];
		const uint32 neighborCount = (uint32) neighbors.size();
		totalNeighborCount += (uint32) neighborCount;
	}
	offsets[vertexCount] = totalNeighborCount;

	// fill neighbors
	neighbors.resize(totalNeighborCount);

	#pragma omp parallel for
	for (int64 vertexIdx = 0; vertexIdx < vertexCount; ++vertexIdx)
	{
		// get neighbors start & end
		const uint32 start = offsets[vertexIdx];
		const uint32 end = offsets[vertexIdx + 1];
		const uint32 count = end - start;

		// copy it to neighbors
		const vector<uint32> &edges = mVerticesToEdges[vertexIdx];
		uint32 *target = neighbors.data() + start;

		for (uint32 localIdx = 0; localIdx < count; ++localIdx)
		{
			const uint32 edgeIdx = edges[localIdx];
			const Edge &edge = mEdges[edgeIdx];
			const uint32 neighborVertex = edge.getOtherVertex((uint32) vertexIdx);

			target[localIdx] = neighborVertex;
		}
	}
}
	
FlexibleMesh &FlexibleMesh::operator =(const FlexibleMesh &rhs)
{
	if (this == &rhs)
		return *this;

	// copy vertex data
	mVerticesToEdges = rhs.mVerticesToEdges;
	mColors = rhs.mColors;
	mNormals = rhs.mNormals;
	mPositions = rhs.mPositions;
	mScales = rhs.mScales;

	// edges & triangles
	mEdges = rhs.mEdges;
	mIndices = rhs.mIndices;

	// remaining data
	mEdgeConflicts = rhs.mEdgeConflicts;

	return *this;
}

void FlexibleMesh::getEdges(uint32 edgeIndices[3], const uint32 triangleIdx) const
{
	const uint32 *triangle = mIndices.data() + 3 * triangleIdx;

	for (uint32 edgeIdx = 0; edgeIdx < 3; ++edgeIdx)
	{
		const uint32 v0 = triangle[edgeIdx];
		const uint32 v1 = triangle[(edgeIdx + 1) % 3];
		edgeIndices[edgeIdx] = getEdgeIndex(v0, v1);
	}
}

void FlexibleMesh::deleteUnsupportedGeometry(const IVertexChecker &vertexChecker)
{
	cout << "FlexibleMesh::deleteUnsupportedGeometry()" << endl;

	const uint32 vertexCount = getVertexCount();
	const uint32 edgeCount =  getEdgeCount();
	const uint32 triangleCount = getTriangleCount();
	clearOffsets(vertexCount, edgeCount, triangleCount);

	// detetect unsupported geometry & delete it
	FlexibleMesh::computeOffsetsForFiltering(mVertexOffsets.data(), mEdgeOffsets.data(), mTriangleOffsets.data(),
		vertexCount, edgeCount, triangleCount,
		mVerticesToEdges.data(), mEdges.data(), mIndices.data(), vertexChecker);
	deleteGeometry(mVertexOffsets.data(), mEdgeOffsets.data(), mTriangleOffsets.data());
}

void FlexibleMesh::clearOffsets(const uint32 vertexCount, const uint32 edgeCount, const uint32 triangleCount)
{
	mVertexOffsets.clear();
	mVertexOffsets.resize(vertexCount + 1);

	mEdgeOffsets.clear();
	mEdgeOffsets.resize(edgeCount + 1);

	mTriangleOffsets.clear();
	mTriangleOffsets.resize(triangleCount + 1);
}

void FlexibleMesh::deleteIsolatedGeometry(const uint32 triangleIslesMinSize)
{
	cout << "FlexibleMesh::deleteIsolatedGeometry()" << endl;

	const uint32 edgeCount = getEdgeCount();
	const uint32 indexCount = getIndexCount();
	const uint32 vertexCount = getVertexCount();

	// detect isles & delete them
	IslesEraser eraser(*this);
	if (!eraser.computeOffsets(triangleIslesMinSize))
		return;

	deleteGeometry(eraser.getVertexOffsets(), eraser.getEdgeOffsets(), eraser.getTriangleOffsets());
	cout << "Finished deletion of small isles (small isolated geometry parts)." << endl;
}

void FlexibleMesh::deleteGeometry(const uint32 *vertexOffsets, const uint32 *edgeOffsets, const uint32 *triangleOffsets)
{
	cout << "FlexibleMesh::deleteGeometry()" << endl;

	const uint32 vertexCount = getVertexCount();
	const uint32 edgeCount = getEdgeCount();
	const uint32 triangleCount = getTriangleCount();

	// delete unsupported geometry
	updateVertices(vertexOffsets, edgeOffsets);
	updateEdges(vertexOffsets, edgeOffsets, triangleOffsets);
	updateTriangles(vertexOffsets, triangleOffsets);

	// update observers
	const uint32 count = getObserverCount();
	for (uint32 i = 0; i < count; ++i)
		mObservers[i]->onFilterData(vertexOffsets, vertexCount, edgeOffsets, edgeCount, triangleOffsets, triangleCount);
}

void FlexibleMesh::updateVertices(const uint32 *vertexOffsets, const uint32 *edgeOffsets)
{
	updateVertexData(vertexOffsets);
	updateVertexToEdgeLinks(edgeOffsets);
}

void FlexibleMesh::updateVertexData(const uint32 *vertexOffsets)
{
	const uint32 oldVertexCount = (uint32) mVerticesToEdges.size();
	const uint32 newVertexCount = oldVertexCount - vertexOffsets[oldVertexCount];

	// filter vertex data
	Array<vector<uint32>>::compaction(mVerticesToEdges, vertexOffsets);
	Array<Vector3>::compaction(mColors, vertexOffsets);
	Array<Vector3>::compaction(mNormals, vertexOffsets);
	Array<Vector3>::compaction(mPositions, vertexOffsets);
	Array<Real>::compaction(mScales, vertexOffsets);
}

void FlexibleMesh::updateVertexToEdgeLinks(const uint32 *edgeOffsets)
{
	// for each vertex: update edge links
	const int64 vertexCount = mPositions.size();

	#pragma omp parallel for
	for (int64 vertexIdx = 0; vertexIdx < vertexCount; ++vertexIdx)
	{
		// process links (vertex -> edges)
		vector<uint32> &adjacentEdges = mVerticesToEdges[vertexIdx];
		for (uint32 localEdgeIdx = 0; localEdgeIdx < adjacentEdges.size(); )
		{
			// keep and update link to edge?
			const uint32 oldGlobalEdgeIdx = adjacentEdges[localEdgeIdx];
			if (edgeOffsets[oldGlobalEdgeIdx] == edgeOffsets[oldGlobalEdgeIdx + 1])
			{
				adjacentEdges[localEdgeIdx] = oldGlobalEdgeIdx - edgeOffsets[oldGlobalEdgeIdx];
				++localEdgeIdx;
				continue;
			}

			// delete edge link
			adjacentEdges[localEdgeIdx] = adjacentEdges.back();
			adjacentEdges.pop_back();
		}
	}
}

void FlexibleMesh::updateEdges(const uint32 *vertexOffsets, const uint32 *edgeOffsets, const uint32 *triangleOffsets)
{
	updateEdgeData(edgeOffsets);
	updateEdgeLinks(vertexOffsets, triangleOffsets);
}

void FlexibleMesh::updateEdgeData(const uint32 *edgeOffsets)
{
	// remove deleted geometry
	Array<Edge>::compaction(mEdges, edgeOffsets);
}

void FlexibleMesh::updateEdgeLinks(const uint32 *vertexOffsets, const uint32 *triangleOffsets)
{
	// for each edge: update links to vertices and triangles
	const int64 edgeCount = mEdges.size();

	#pragma omp parallel for
	for (int64 edgeIdx = 0; edgeIdx < edgeCount; ++edgeIdx)
	{
		Edge &edge = mEdges[edgeIdx];

		// update links to vertices
		const uint32 *oldVertices = edge.getVertexIndices();
		edge.setVertices(oldVertices[0] - vertexOffsets[oldVertices[0]],
						 oldVertices[1] - vertexOffsets[oldVertices[1]]);

		// update links to triangles
		const uint32 *oldTriangles = edge.getTriangleIndices();
		uint32 newTriangleIndices[2] = { Triangle::INVALID_INDEX, Triangle::INVALID_INDEX };
		
		for (uint32 sideIdx = 0; sideIdx < 2; ++sideIdx)
		{
			const uint32 old = oldTriangles[sideIdx];
			if (Triangle::INVALID_INDEX == old)
				continue;

			if (triangleOffsets[old] == triangleOffsets[old + 1])
				newTriangleIndices[sideIdx] = old - triangleOffsets[old];
		}

		edge.setTriangles(newTriangleIndices[0], newTriangleIndices[1]);
	}
}

void FlexibleMesh::updateTriangles(const uint32 *vertexOffsets, const uint32 *triangleOffsets)
{
	updateTriangleData(vertexOffsets, triangleOffsets);
}

void FlexibleMesh::updateTriangleData(const uint32 *vertexOffsets, const uint32 *triangleOffsets)
{
	// memory for new triangles
	const uint32 oldIndexCount = (uint32) mIndices.size();
	const uint32 oldTriangleCount = oldIndexCount / 3;
	const uint32 newTriangleCount = oldTriangleCount - triangleOffsets[oldTriangleCount];
	const uint32 newIndexCount = 3 * newTriangleCount;

	// filter triangle indices
	vector<uint32> newIndices(newIndexCount);
	FlexibleMesh::filterTriangles(newIndices.data(), mIndices.data(), triangleOffsets, oldIndexCount, vertexOffsets);
	mIndices.swap(newIndices);
}

void FlexibleMesh::set(const Vector3 &color, const Vector3 &normal, const Vector3 &position, const Real scale,
	const uint32 vertexIdx)
{
	if (vertexIdx >= getVertexCount())
	{
		// todo log this
		return;
	}
	
	mColors[vertexIdx] = color;
	mNormals[vertexIdx] = normal;
	mPositions[vertexIdx] = position;
	mScales[vertexIdx] = scale;
}

void FlexibleMesh::setIndices(const uint32 *indices, const uint32 indexCount)
{
	mIndices.resize(indexCount);
	memcpy(mIndices.data(), indices, sizeof(uint32) * indexCount);
	findAdjacencies();
}

void FlexibleMesh::fillHoles()
{
	vector<vector<uint32>> holeBorders(1);
	vector<uint32> &border = holeBorders[0];
	border.reserve(5000);

	// find all edges at hole borders
	const uint32 edgeCount = getEdgeCount();
	#pragma omp parallel for
	for (int64 i = 0; i < edgeCount; ++i)
	{
		// get edge data
		const uint32 edgeIdx = (uint32) i;
		const Edge &edge = mEdges[edgeIdx];
		const uint32 *triangles = edge.getTriangleIndices();

		// not a hole border edge?
		if (Triangle::INVALID_INDEX != triangles[0] && Triangle::INVALID_INDEX != triangles[1])
			continue;

		const uint32 existingSide = (triangles[1] != Triangle::INVALID_INDEX);
		const uint32 *triangle = getTriangle(triangles[existingSide]);
		uint32 orderedVertices[2];
		Triangle::getVerticesInWindingOrder(orderedVertices, triangle, edge.getVertexIndices());

		#pragma omp critical (OMPPushBackHoleBorderEdge)
		{
			border.push_back(orderedVertices[0]);
			border.push_back(orderedVertices[1]);
		}
	}

	// find connected rings & fill holes
	findBorderRings(holeBorders);
	fillHoles(holeBorders);
}

void FlexibleMesh::findBorderRings(vector<vector<uint32>> &holeBorderEdges, const uint32 *vertexOffsets)
{
	// remove emtpy border edge sets
	for (uint32 borderIdx = 0; borderIdx < holeBorderEdges.size(); )
	{
		if (!holeBorderEdges[borderIdx].empty())
		{
			++borderIdx;
			continue;
		}
		
		holeBorderEdges[borderIdx].swap(holeBorderEdges.back());
		holeBorderEdges.pop_back();
	}

	// for each set of hole border edges: find all contained rings & store them separately in original/entered passing order
	holeBorderEdges.resize(holeBorderEdges.size() + 1); 

	for (uint32 borderIdx = 0; borderIdx < holeBorderEdges.size() - 1; ++borderIdx)
	{
		vector<uint32> &potentiallyNewBorder = holeBorderEdges.back();
		findBorderRing(potentiallyNewBorder, holeBorderEdges[borderIdx]);

		if (!potentiallyNewBorder.empty())
			holeBorderEdges.resize(holeBorderEdges.size() + 1);
	}

	if (holeBorderEdges.back().empty())
		holeBorderEdges.pop_back();

	removeDuplicatesInRings(holeBorderEdges, vertexOffsets);
}

void FlexibleMesh::findBorderRing(vector<uint32> &remainingEdges, vector<uint32> &edgeSet)
{
	assert(remainingEdges.empty());
	remainingEdges.clear();

	const uint32 ringStart = edgeSet[0];
	uint32 ringSize = 2;
	//cout << "Processing border edges, edge count: " << edgeSet.size() << endl;

	// find completely connected rings of border edges
	const uint32 edgeCount = (uint32) edgeSet.size();
	for (uint32 v0Idx = ringSize; v0Idx < edgeCount; )
	{
		//cout << "v0Idx, ring size, edge count << " << v0Idx << "," << ringSize << "," << edgeCount << endl;
		const uint32 v1Idx = v0Idx + 1;
		const uint32 edge[2] = { edgeSet[v0Idx], edgeSet[v1Idx] };
		const uint32 ringEnd = edgeSet[ringSize - 1];

		// does edge do not fit?
		if (edge[0] != ringEnd)
		{
			v0Idx += 2;
			continue;
		}

		extendBorderRing(ringSize, edgeSet, edge, v0Idx, v1Idx);
		v0Idx = ringSize;
			
		// no ring closure?
		if (edge[1] != ringStart)
			continue;
		
		// more than 1 ring?
		const uint32 remainingEdgeCount = edgeCount - ringSize;
		if (0 == remainingEdgeCount)
			return;

		remainingEdges.reserve(remainingEdgeCount);
		remainingEdges.insert(remainingEdges.end(), edgeSet.begin() + ringSize, edgeSet.end());
		edgeSet.resize(ringSize);		
		return;
	}
}

void FlexibleMesh::extendBorderRing(uint32 &size, vector<uint32> &edgeSet,
	const uint32 edge[2], const uint32 v0Idx, const uint32 v1Idx)
{
	// exchange edges & extend border ring

	// preserve the edge at ring end memory location
	const uint32 ringEnd0 = size;
	const uint32 ringEnd1 = size + 1;
	edgeSet[v0Idx] = edgeSet[ringEnd0];
	edgeSet[v1Idx] = edgeSet[ringEnd1];
	
	// extend ring
	size += 2;
	edgeSet[ringEnd0] = edge[0];
	edgeSet[ringEnd1] = edge[1];
}

void FlexibleMesh::removeDuplicatesInRings(vector<vector<uint32>> &holeBorders, const uint32 *vertexOffsets)
{
	// skip each second vertex and all vertices with vertexOffsets[vIdx] !+ vertexOffsets[vIdx + 1]
	const uint32 ringCount = (uint32) holeBorders.size();

	for (uint32 ringIdx = 0; ringIdx < ringCount; ++ringIdx)
	{
		vector<uint32> &ring = holeBorders[ringIdx];
		const uint32 ringSize = (uint32) ring.size();
		uint32 targetIdx = 0;

		for (uint32 sourceIdx = 0; sourceIdx < ringSize; sourceIdx += 2)
		{
			const uint32 vIdx = ring[sourceIdx];
			if (vertexOffsets && vertexOffsets[vIdx] != vertexOffsets[vIdx + 1])
				continue;
			ring[targetIdx++] = ring[sourceIdx];
		}

		ring.resize(targetIdx);
	}
}

void FlexibleMesh::fillHoles(const vector<vector<uint32>> &holeBorderRings)
{
	cout << "Filling surface holes." << endl;

	// old element counts
	const uint32 oldVertexCount = getVertexCount();
	const uint32 oldEdgeCount = getEdgeCount();
	const uint32 oldTriangleCount = getTriangleCount();

	// reserve memory for new element counts
	reserveForHoleFilling(holeBorderRings);

	// fill each hole with a single vertex as mean of all corresponding hole border vertices
	const uint32 holeCount = (uint32) holeBorderRings.size();
	for (uint32 holeIdx = 0; holeIdx < holeCount; ++holeIdx)
	{
		const vector<uint32> &holeBorder = holeBorderRings[holeIdx];
		const uint32 borderVertexCount = (uint32) holeBorder.size();

		// hole center position
		Vector3 center;
		for (uint32 i = 0; i < borderVertexCount; ++i)
			center += mPositions[holeBorder[i]];
		center /= (Real) borderVertexCount;

		fillHole(holeBorder.data(), borderVertexCount, center);
	}

	onNewElements(oldVertexCount, oldEdgeCount, oldTriangleCount);
	//checkConnectivity("FlexibleMesh::fillHoles");
}

void FlexibleMesh::fillHole(const uint32 *borderVertices, const uint32 borderEdgeCount, const Vector3 &center)
{
	// debug output
	//cout << "fillHole" << (borderVertices ? " with borderVertices" : "") << ", size: " << borderEdgeCount << endl;
	//if (borderVertices)
	//{
	//	for (uint32 i = 0; i < borderEdgeCount; ++i)
	//		cout << borderVertices[i] << " ";
	//	cout << endl;
	//}

	if (borderEdgeCount <= 5)
	{
		fillHole(borderVertices, borderEdgeCount);
		return;
	}
	
	// vertex counts
	const uint32 oldVertexCount = getVertexCount();
	const uint32 additionalVertexCount = borderEdgeCount / 2;
	const uint32 newVertexCount = oldVertexCount + additionalVertexCount;

	createHoleFillingRingVertices(borderVertices, borderEdgeCount, newVertexCount, center);
	connectHoleFillingRings(borderVertices, borderEdgeCount, oldVertexCount, additionalVertexCount);

	// do the same for a smaller problem size
	fillHole(NULL, additionalVertexCount, center);
}

void FlexibleMesh::createHoleFillingRingVertices(const uint32 *borderVertices, const uint32 borderEdgeCount,
	const uint32 newVertexCount, const Vector3 &center)
{
	// create a smaller, inner ring and connect it to border
	const uint32 oldVertexCount = getVertexCount();
	resize(newVertexCount, getIndexCount());

	// distance of new ring relative to last ring and center?
	uint32 remainingRingCount = 0;
	for (uint32 temp = borderEdgeCount; temp; temp >>= 1)
		++remainingRingCount;
	remainingRingCount -= 1;
	const Real f0 = (remainingRingCount - 1) / (Real) remainingRingCount;
	const Real f1 = 1.0f / remainingRingCount;

	// position the ring
	const uint32 previousRingStart = oldVertexCount - borderEdgeCount;
	const uint32 lastVertex = newVertexCount - 1;

	uint32 borderVertex = (borderVertices ? 0 : previousRingStart);
	for (uint32 vertexIdx = oldVertexCount; vertexIdx < newVertexCount; ++vertexIdx)
	{
		const uint32 v0 = (borderVertices ? borderVertices[borderVertex] : borderVertex);
		++borderVertex;
		const uint32 v1 = (borderVertices ? borderVertices[borderVertex] : borderVertex);
		++borderVertex;

		uint32 v2;
		if (vertexIdx == lastVertex)
			v2 = (borderVertices ? borderVertices[0] : previousRingStart);
		else
			v2 = (borderVertices ? borderVertices[borderVertex] : borderVertex);
		
		// set p to center of 2 edges & somewhat towars hole center
		Vector3 &p = mPositions[vertexIdx];
		p = mPositions[v0] + mPositions[v1] + mPositions[v2];
		p = (p * (f0 / 3.0f) + center * f1);
	}
}

void FlexibleMesh::connectHoleFillingRings(const uint32 *borderVertices, const uint32 borderEdgeCount,
	const uint32 oldVertexCount, const uint32 additionalVertexCount)
{
	// connect the ring new ring [oldVertexCount, newVertexCount) to the previous ring borderVertices or [oldVertexCount - borderEdgeCount, oldVertexCount)

	// triangles with outer ring edges
	//cout << "Adding outer triangles of new triangle ring.\n";
	const uint32 outerRingStart = (borderVertices ? 0 : oldVertexCount - borderEdgeCount);
	const uint32 lastInnerOffset = additionalVertexCount - 1;
	const bool odd = (1 == (borderEdgeCount & 1));
	
	for (uint32 outerOffset = outerRingStart, innerOffset = 0; innerOffset < additionalVertexCount; ++innerOffset)
	{
		const uint32 outerV0 = (borderVertices ? borderVertices[outerOffset] : outerOffset);
		++outerOffset;
		const uint32 outerV1 = (borderVertices ? borderVertices[outerOffset] : outerOffset);
		++outerOffset;

		uint32 outerV2 ;
		if (!odd && innerOffset == lastInnerOffset)
			outerV2 = (borderVertices ? borderVertices[outerRingStart] : outerRingStart);
		else
			outerV2 = (borderVertices ? borderVertices[outerOffset] : outerOffset);
		
		//cout << "outerOffset " << outerOffset << " innerOffset " << innerOffset;
		//cout << "outerV0 " << outerV0 << " outerV1 " << outerV1 << " inner " << (oldVertexCount + innerOffset);
		addTriangle(outerV0, outerV1, oldVertexCount + innerOffset);
		addTriangle(outerV1, outerV2, oldVertexCount + innerOffset);
	}

	// one more outer ring edge triangle due to odd border edge number?
	if (odd)
	{	
		//cout << "Adding outer triangle due to odd edge count.\n";
		if (borderVertices)
			addTriangle(borderVertices[borderEdgeCount - 1], borderVertices[0], getVertexCount() - 1);
		else
			addTriangle(oldVertexCount - 1, outerRingStart, getVertexCount() - 1);
	}

	// triangles with inner ring edges
	//cout << "Adding inner triangles of new ring.\n";
	//cout << "outerRingStart " << outerRingStart;
	for (uint32 outerOffset = outerRingStart + 2, innerOffset = 0; innerOffset < additionalVertexCount; outerOffset += 2)
	{
		const uint32 innerV0 = oldVertexCount + innerOffset;
		uint32 innerV1;
		uint32 outerV;

		++innerOffset;
		if (innerOffset == additionalVertexCount)
		{
			innerV1 = oldVertexCount;
			outerV = (borderVertices ? borderVertices[outerRingStart] : outerRingStart);
		}
		else
		{
			innerV1 = oldVertexCount + innerOffset;
			outerV = (borderVertices ? borderVertices[outerOffset] : outerOffset);
		}

		//cout << "inner triangle: " << innerV0 << " " << outerV << " " << innerV1 << endl;
		
		addTriangle(innerV0, outerV, innerV1);
	}
}

void FlexibleMesh::fillHole(const uint32 *borderVertices, const uint32 borderEdgeCount)
{
	// get indices for new triangles
	uint32 indices[5];
	if (borderVertices)
	{
		memcpy(indices, borderVertices, sizeof(uint32) * borderEdgeCount);
	}
	else
	{
		const uint32 start = getVertexCount() - borderEdgeCount;
		for (uint32 i = 0; i < borderEdgeCount; ++i)
			indices[i] = start + i;
	}
	
	// add up to 3 triangles to fill the hole without new vertices
	addTriangle(indices[0], indices[1], indices[2]);
	if (3 == borderEdgeCount)
		return;
	
	addTriangle(indices[2], indices[3], indices[0]);
	if (4 == borderEdgeCount)
		return;
	
	addTriangle(indices[3], indices[4], indices[0]);
	if (5 == borderEdgeCount)
		return;

	assert(false);
}

void FlexibleMesh::reserveForHoleFilling(const vector<vector<uint32>> &holeBorders)
{
	const uint32 holeCount = (uint32) holeBorders.size();
	uint32 newVertexCount = getVertexCount();
	uint32 newEdgeCount = getEdgeCount();
	uint32 newTriangleCount = getTriangleCount();

	// each hole is filled whereas smaller rings are inserted until a ring size of 5 is reached
	// each smaller ring has (previousRingSize / 2) new vertices
	for (uint32 holeIdx = 0; holeIdx < holeCount; ++holeIdx)
	{
		const vector<uint32> &border = holeBorders[holeIdx];
		const uint32 edgeCount = (uint32) border.size();

		for (uint32 ringSize = edgeCount; ringSize > 5; )
		{
			if (ringSize > 5)
			{
				const uint32 nextRingSize = ringSize / 2;
				newVertexCount += nextRingSize;
				newEdgeCount += nextRingSize * 4 + (ringSize & 1);
				newTriangleCount += ringSize + nextRingSize;

				ringSize = nextRingSize;
				continue;
			}

			if (ringSize < 3)
			{
				assert(false);
				throw Exception("Wrongly implemented hole filling.");
			}
			
			// new elements if only 5, 4 or 3 border edges are left
			// newVertexCount +=0;
			newEdgeCount += ringSize - 3;
			newTriangleCount += ringSize - 2;
			break;
		}
	}

	reserve(newVertexCount, newEdgeCount, newTriangleCount * 3);
}

Math::Vector3 FlexibleMesh::getCenterOfNeighbors(const uint32 vertexIdx) const
{	
	// neighborhood data
	const vector<uint32> &edgeIndices = mVerticesToEdges[vertexIdx];
	const uint32 neighborCount = (uint32) edgeIndices.size();
	
	// sum of neighbor positions
	Vector3 center;
	for (uint32 neighborIdx = 0; neighborIdx < neighborCount; ++neighborIdx)
	{
		const uint32 edgeIdx = edgeIndices[neighborIdx];
		const Edge &edge = mEdges[edgeIdx];
		const uint32 neighborvertexIdx = edge.getOtherVertex(vertexIdx);
		const Vector3 &p = mPositions[neighborvertexIdx];

		center += p;
	}

	// average
	center /= (Real) neighborCount;
	return center;
}

void FlexibleMesh::onNewElements(const uint32 oldVertexCount, const uint32 oldEdgeCount, const uint32 oldTriangleCount)
{
	// new element counts
	const uint32 vertexCount = getVertexCount();
	const uint32 edgeCount = getEdgeCount();
	const uint32 triangleCount = getTriangleCount();

	const uint32 observerCount = getObserverCount();
	for (uint32 observerIdx = 0; observerIdx < observerCount; ++observerIdx)
		mObservers[observerIdx]->onNewElements(oldVertexCount, vertexCount, oldEdgeCount, edgeCount, oldTriangleCount, triangleCount);
}

void FlexibleMesh::reserve(const uint32 newVertexCount, const uint32 newEdgeCount, const uint32 newIndexCount)
{
	// vertices
	mVerticesToEdges.reserve(newVertexCount);
	mColors.reserve(newVertexCount);
	mNormals.reserve(newVertexCount);
	mPositions.reserve(newVertexCount);
	mScales.reserve(newVertexCount);

	// edges
	mEdges.reserve(newEdgeCount);

	// triangles & indices
	mIndices.reserve(newIndexCount);
	
	// update observers
	const uint32 observerCount = getObserverCount();
	for (uint32 observerIdx = 0; observerIdx < observerCount; ++observerIdx)
		mObservers[observerIdx]->onReserveMemory(newVertexCount, newEdgeCount, newIndexCount);
}

void FlexibleMesh::allocateMemory(const uint32 vertexCount, const uint32 indexCount)
{
	resize(vertexCount, indexCount);
}

void FlexibleMesh::resize(const uint32 newVertexCount, const uint32 newIndexCount)
{
	// vertices
	mVerticesToEdges.resize(newVertexCount);
	mColors.resize(newVertexCount);
	mNormals.resize(newVertexCount);
	mPositions.resize(newVertexCount);
	mScales.resize(newVertexCount);

	// triangles
	mIndices.resize(newIndexCount);
}
void FlexibleMesh::doSelfCheck() const
{
	#ifdef _DEBUG
		// check vertex data
		const uint32 vertexCount = getVertexCount();

		for (uint32 vertexIdx = 0; vertexIdx < vertexCount; ++vertexIdx)
		{
			const Vector3 &c = mColors[vertexIdx];
			const Vector3 &n = mNormals[vertexIdx];
			const Vector3 &p = mPositions[vertexIdx];
			const Real &s = mScales[vertexIdx];

			assert(!p.hasNaNComponent());
			assert(!n.hasNaNComponent());
			assert(!c.hasNaNComponent());
			assert(!isNaN(s));
		}
	#endif // _DEBUG
}

void FlexibleMesh::checkEdges() const
{
	// counts & edges
	const uint32 vertexCount = getVertexCount();
	const uint32 edgeCount = getEdgeCount();
	const uint32 triangleCount = getTriangleCount();

	// check vertices & triangles of each edge
	for (uint32 i = 0; i < edgeCount; ++i)
	{
		const Edge &edge = mEdges[i];
		const uint32 *triangles = edge.getTriangleIndices();
		const uint32 *vertices = edge.getVertexIndices();

		bool error = false;
		for (uint32 sideIdx = 0; sideIdx < 2; ++sideIdx)
			if (triangles[sideIdx] >= triangleCount || vertices[sideIdx] >= vertexCount)
				error = true;

		if (!error)
			continue;

		// debug output
		string message = "checkEdges(): found a broken edge.\n";

		cerr << message;
		cerr << "Index: " << i << "\n";
		cerr << edge << endl;
		
		cout << message;
		cout << "Index: " << i << "\n";
		cout << edge << endl;
	}
}

//void FlexibleMesh::checkConnectivity(const string &info,
//	const uint32 *doomedVertices, const uint32 *doomedEdges, const uint32 *doomedTriangles) const
//{
//	// check edges
//	const uint32 edgeCount = getEdgeCount();
//	#pragma omp parallel for
//	for (int64 edgeIdx = 0; edgeIdx < edgeCount; ++edgeIdx)
//	{
//		if (doomedEdges && doomedEdges[edgeIdx])
//			continue;
//
//		const Edge &edge = mEdges[edgeIdx];
//		const uint32 *triangles = edge.getTriangleIndices();
//		const uint32 *vertices = edge.getVertexIndices();
//
//		// is there a problem?
//		bool error = false;
//		error |= Triangle::INVALID_INDEX == triangles[0] || Triangle::INVALID_INDEX == triangles[1] ||
//				 Vertex::INVALID_INDEX == vertices[0] || Vertex::INVALID_INDEX == vertices[1];
//		if (doomedVertices)
//			error |= (doomedVertices[vertices[0]] || doomedVertices[vertices[1]]);
//		if (doomedTriangles)
//			error |= (doomedTriangles[triangles[0]] || doomedTriangles[triangles[1]]);
//		if (!error)
//			continue;
//
//		#pragma omp critical (OMPCout)
//		{
//			cout << info << endl;
//			cout << "Invalid edge: " << edgeIdx;
//			cout << edge;
//			cout << endl;
//		}
//	}
//
//	// check vertices
//	if (doomedVertices && doomedEdges)
//	{
//		const uint32 vertexCount = getVertexCount();
//		#pragma omp parallel for
//		for (int64 vertexIdx = 0; vertexIdx < vertexCount; ++vertexIdx)
//		{
//			if (doomedVertices[vertexIdx])
//				continue;
//
//			const vector<uint32> &edges = mVerticesToEdges[vertexIdx];
//			const uint32 edgeCount = (uint32) edges.size();
//			for (uint32 localEdgeIdx = 0; localEdgeIdx < edgeCount; ++localEdgeIdx)
//			{
//				const uint32 globalEdgeIdx = edges[localEdgeIdx];
//				if (!doomedEdges[globalEdgeIdx])
//					continue;
//
//				const Edge &edge = mEdges[globalEdgeIdx];
//				#pragma omp critical (OMPCout)
//				{
//					cout << info << endl;
//					cout << "Invalid vertex to edge link: " << vertexIdx << "," << localEdgeIdx << "," << globalEdgeIdx << endl;
//					cout << edge;
//					cout << endl;
//				}
//			}
//		}
//	}
//
//	// check triangles
//	if (doomedVertices && doomedTriangles)
//	{
//		const uint32 triangleCount = getTriangleCount();
//		#pragma omp parallel for
//		for (int64 triangleIdx = 0; triangleIdx < triangleCount; ++triangleIdx)
//		{
//			if (doomedTriangles[triangleIdx])
//				continue;
//
//			const uint32 *triangle = getTriangle((uint32) triangleIdx);
//			const bool error = (Vertex::INVALID_INDEX == triangle[0] || Vertex::INVALID_INDEX == triangle[1] || Vertex::INVALID_INDEX == triangle[2]);
//			if (!error)
//				continue;
//			
//			#pragma omp critical (OMPCout)
//			{
//				cout << info << endl;
//				cout << "Invalid triangle: " << triangleIdx << ": " << triangle[0] << "," << triangle[1] << "," << triangle[2];
//				cout << endl;
//			}
//		}
//	}
//}

	//FlexibleMesh copy(*this);
	//copy.setColor(Vector3(1.0f, 0.0f, 0.0f), 2800);
	//copy.setColor(Vector3(1.0f, 0.0f, 0.0f), 2801);
	//copy.saveToFile("C:\\Dev\\Scenes\\ShellTower\\Results\\loaded", true, false);
	//checkConnectivity("findAdjacencies");