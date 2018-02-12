/*
 * Copyright (C) 2018 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */
#include "CollisionDetection/CollisionDetection.h"
#include "Evaluator.h"
#include "Math/MathHelper.h"
#include "Platform/Utilities/PlyFile.h"
#include "SurfaceReconstruction/Geometry/FlexibleMesh.h"
#include "SurfaceReconstruction/Geometry/StaticMesh.h"

using namespace CollisionDetection;
using namespace Math;
using namespace std;
using namespace Storage;
using namespace SurfaceReconstruction;
using namespace Utilities;

Evaluator::Evaluator(const string &groundTruthFile, const string &reconstructionFile)
{
	// todo magic constants 42
	const Vector3 AABB_SCALE_FACTORS(0.9f, 1.2f, 0.7f);
	const Real MAX_Z = 0.1f;

	// load meshes
	const StaticMesh *groundTruth = new StaticMesh(groundTruthFile);
	const StaticMesh *reconstruction = new StaticMesh(reconstructionFile);

	// only keep one side of the ground truth (2D evaluation)
	{
		const Vector3 *oldPositions = groundTruth->getPositions();
		const uint32 *oldIndices = groundTruth->getIndices();
		const uint32 oldIndexCount = groundTruth->getIndexCount();
		const uint32 oldTriangleCount = oldIndexCount / 3;
		const uint32 oldVertexCount = groundTruth->getVertexCount();

		// compute vertex & triangle offsets for filtering
		uint32 *vertexOffsets = new uint32[oldVertexCount + 1];

		vertexOffsets[0] = 0;
		for (uint32 vertexIdx = 0; vertexIdx < oldVertexCount; ++vertexIdx)
		{
			vertexOffsets[vertexIdx + 1] = vertexOffsets[vertexIdx];

			// delete current vertex?
			const Vector3 &p = oldPositions[vertexIdx];
			if (p.z > MAX_Z)
				++vertexOffsets[vertexIdx + 1];
		}
		
		// counts for filtered geometry
		const uint32 newVertexCount = oldVertexCount - vertexOffsets[oldVertexCount];

		// get filtered geometry
		// reserve memory for filtered ground truth geometry
		Vector3 *newPositions = new Vector3[newVertexCount];
		FlexibleMesh::filterData<Vector3>(newPositions, oldPositions, vertexOffsets, oldVertexCount);

		// 2D reconstruction vertex data
		mGroundTruthPositions.resize(newVertexCount);
		for (uint32 vertexIdx = 0; vertexIdx < newVertexCount; ++vertexIdx)
		{
			// 2D position
			const Vector3 &p = newPositions[vertexIdx];
			mGroundTruthPositions[vertexIdx] = Math::reduceVector(p, AXIS_Z);
		}

		// 2D reconstruction edges
		const uint32 *triangle = oldIndices;

		for (uint32 triangleIdx = 0; triangleIdx < oldTriangleCount; ++triangleIdx, triangle += 3)
		{
			// right triangle edge which does not change z and is on the right side
			for (uint32 edgeIdx = 0; edgeIdx < 3; ++edgeIdx)
			{
				const uint32 relativeV0 = edgeIdx;
				const uint32 relativeV1 = (edgeIdx + 1) % 3;
				const uint32 oldIdx0 = triangle[relativeV0];
				const uint32 oldIdx1 = triangle[relativeV1];

				const Vector3 &p0 = oldPositions[oldIdx0];
				const Vector3 &p1 = oldPositions[oldIdx1];
				if (p0.z > MAX_Z || p1.z > MAX_Z)
					continue;
				
				// add edge
				mGroundTruthIndices.push_back(oldIdx0 - vertexOffsets[oldIdx0]);
				mGroundTruthIndices.push_back(oldIdx1 - vertexOffsets[oldIdx1]);

				// add edge normal
				Vector3 triangleNormal;
				Math::computeTriangleNormal(triangleNormal, oldPositions[triangle[0]], oldPositions[triangle[1]], oldPositions[triangle[2]]);

				Vector2 edgeNormal = Math::reduceVector(triangleNormal, AXIS_Z);
				edgeNormal.normalize();
				mGroundTruthEdgeNormals.push_back(edgeNormal);
				break;
			}
		}

		// free memory
		delete [] newPositions;
		newPositions = NULL;

		delete [] vertexOffsets;
		vertexOffsets = NULL;
	}

	// AABB
	groundTruth->computeAABB(mAABB[0], mAABB[1]);
	const Vector3 size = mAABB[1] - mAABB[0];
	for (Axis axis = AXIS_X; axis <= AXIS_Z; axis = (Axis) (axis + 1))
	{
		const Real change = size[axis] * (AABB_SCALE_FACTORS[axis] - 1.0f);
		mAABB[0][axis] -= 0.5f * change;
		mAABB[1][axis] += 0.5f * change;
	}

	// filter reconstruction geometry outside mAABB
	{
		// mesh data which is filtered
		const Vector3 *oldPositions = reconstruction->getPositions();
		const Vector3 *oldNormals = reconstruction->getNormals();
		const uint32 *oldIndices = reconstruction->getIndices();
		const uint32 oldIndexCount = reconstruction->getIndexCount();
		const uint32 oldTriangleCount = oldIndexCount / 3;
		const uint32 oldVertexCount = reconstruction->getVertexCount();

		// compute vertex & triangle offsets for filtering
		uint32 *vertexOffsets = new uint32[oldVertexCount + 1];
		uint32 *triangleOffsets = new uint32[oldTriangleCount + 1];

		// only keep points within mAABB
		vertexOffsets[0] = 0;
		for (uint32 vertexIdx = 0; vertexIdx < oldVertexCount; ++vertexIdx)
		{
			vertexOffsets[vertexIdx + 1] = vertexOffsets[vertexIdx];

			// delete current vertex?
			const Vector3 &p = oldPositions[vertexIdx];
			if (!CollisionDetection::isPointInAABB(p, mAABB))
				vertexOffsets[vertexIdx + 1] += 1;
		}
		FlexibleMesh::computeTriangleOffsets(triangleOffsets, vertexOffsets, oldIndices, oldIndexCount, NULL);

		// counts for filtered geometry
		const uint32 newVertexCount = oldVertexCount - vertexOffsets[oldVertexCount];
		const uint32 newTriangleCount = oldTriangleCount - triangleOffsets[oldTriangleCount];
		const uint32 newIndexCount = 3 * newTriangleCount;

		// reserve memory for filtered reconstruction geometry
		Vector3 *newNormals = new Vector3[newVertexCount];
		Vector3 *newPositions = new Vector3[newVertexCount];
		uint32 *newIndices = new uint32[newIndexCount];

		FlexibleMesh::filterData<Vector3>(newNormals, oldNormals, vertexOffsets, oldVertexCount);
		FlexibleMesh::filterData<Vector3>(newPositions, oldPositions, vertexOffsets, oldVertexCount);
		FlexibleMesh::filterTriangles(newIndices, oldIndices, triangleOffsets, oldIndexCount, vertexOffsets);

		// 2D reconstruction vertex data
		mNormals.resize(newVertexCount);
		mPositions.resize(newVertexCount);
		for (uint32 vertexIdx = 0; vertexIdx < newVertexCount; ++vertexIdx)
		{
			const Vector3 &n = newNormals[vertexIdx];
			const Vector3 &p = newPositions[vertexIdx];

			// 2D normal & position
			mNormals[vertexIdx] = Math::reduceVector(n, AXIS_Z);
			mNormals[vertexIdx].normalize();
			mPositions[vertexIdx] = Math::reduceVector(p, AXIS_Z);
		}

		// 2D reconstruction edges
		const uint32 *triangle = newIndices;
		mIndices.reserve(newTriangleCount * 3);

		for (uint32 triangleIdx = 0; triangleIdx < newTriangleCount; ++triangleIdx, triangle += 3)
		{
			for (uint32 edgeIdx = 0; edgeIdx < 3; ++edgeIdx)
			{
				const uint32 relativeV0 = edgeIdx;
				const uint32 relativeV1 = (edgeIdx + 1) % 3;

				mIndices.push_back(triangle[relativeV0]);
				mIndices.push_back(triangle[relativeV1]);
			}
		}

		// free memory
		delete [] newNormals;
		newNormals = NULL;

		delete [] newPositions;
		newPositions = NULL;

		delete [] newIndices;
		newIndices = NULL;

		delete [] triangleOffsets;
		triangleOffsets = NULL;

		delete [] vertexOffsets;
		vertexOffsets = NULL;
	}

	// free meshes
	delete groundTruth;
	groundTruth = NULL;

	delete reconstruction;
	reconstruction = NULL;

	computeErrors();
}


void Evaluator::computeErrors()
{
	// memory
	const int64 vertexCount = mPositions.size();
	mClosestPoints.resize(vertexCount);
	mDirectedErrors.resize(vertexCount);
	mSignedErrors.resize(vertexCount);

	// compute the error values for all vertices
	#pragma omp parallel for
	for (int64 vertexIdx = 0; vertexIdx < vertexCount; ++vertexIdx)
	{
		const Vector2 &p = mPositions[vertexIdx];
		const Vector2 &n = mNormals[vertexIdx];

		// distance & closest point
		Real distance = REAL_MAX;
		uint32 cloesestEdgeIdx = -1;
		const Vector2 c = computeClosestPointToGroundTruth(cloesestEdgeIdx, distance, p, n);
		
		// positive or negative error?
		bool negative = false;
		const Vector2 directedError = p - c;
		if (-1 != cloesestEdgeIdx)
			negative = (0 > directedError.dotProduct(mGroundTruthEdgeNormals[cloesestEdgeIdx]));

		mClosestPoints[vertexIdx] = c;
		mDirectedErrors[vertexIdx] = directedError;
		mSignedErrors[vertexIdx] = (negative ? -distance : distance);
	}
}

Vector2 Evaluator::computeClosestPointToGroundTruth(uint32 &closestEdgeIdx, Real &distance, const Vector2 &p, const Vector2 &n)
{
	Vector2 closestPoint(REAL_MAX, REAL_MAX);
	Real minDistance = REAL_MAX;

	// test (p, n) against all ground truth edges
	const uint32 edgeCount = (uint32) mGroundTruthEdgeNormals.size();
	for (uint32 edgeIdx = 0, edgeVertexIdx = 0; edgeIdx < edgeCount; ++edgeIdx, edgeVertexIdx += 2)
	{
		// only consider surface which can possibly be the source of p regarding normals
		const Vector2 &edgeNormal = mGroundTruthEdgeNormals[edgeIdx];
		const Real dotProduct = edgeNormal.dotProduct(n);
		if (Math::EPSILON > dotProduct)
			continue;

		// compute distance to edge edgeIdx
		const uint32 e0Idx = mGroundTruthIndices[edgeVertexIdx];
		const uint32 e1Idx = mGroundTruthIndices[edgeVertexIdx + 1];
		const Vector2 &e0 = mGroundTruthPositions[e0Idx];
		const Vector2 &e1 = mGroundTruthPositions[e1Idx];
		Vector2 testPoint;

		const Real distance = CollisionDetection::getDistanceToLineSegment(&testPoint, p, e0, e1);
		if (distance >= minDistance)
			continue;

		minDistance = distance;
		closestPoint = testPoint;
		closestEdgeIdx = edgeIdx;
	}

	distance = minDistance;
	return closestPoint;
}

void Evaluator::saveResults(const string &targetFolder) const
{
	saveErrors(targetFolder);

	const string groundTruthFileName = targetFolder + "GroundTruth2D.ply";
	save2DMesh(groundTruthFileName, mGroundTruthPositions.data(), mGroundTruthIndices.data(), mGroundTruthPositions.size(), mGroundTruthIndices.size(), true);

	const string reconstructionFileName = targetFolder + "Reconstruction2D.ply";
	save2DMesh(reconstructionFileName, mPositions.data(), mIndices.data(), mPositions.size(), mIndices.size(), false);
}

void Evaluator::saveErrors(const string &targetFolder) const
{
	const Encoding encoding = ENCODING_ASCII;
	const int64 vertexCount = mPositions.size();
	const string fileName = targetFolder + "Errors.txt";

	// write out error values
	File errorFile(fileName, File::CREATE_WRITING, false);

	// header
	// vertex count
	const string vertexCountText = "Vertex count:\n";
	errorFile.write(vertexCountText.c_str(), 1, vertexCountText.size());
	errorFile.writeInt32((int32) vertexCount, encoding);
	errorFile.write("\n\n", 1, 2);

	// table header
	const string tableHeader = "Signed error\tDirected error x\tDirected error y\tClosest point x\tClosest point y\tReconstructed point x\tReconstructed point y\n";
	errorFile.write(tableHeader.c_str(), 1, tableHeader.size());

	// each vertex error
	for (int64 i = 0; i < vertexCount; ++i)
	{
		// absolute error
		errorFile.writeReal(mSignedErrors[i], encoding);
		errorFile.write("\t", 1, 1);

		// directed error
		errorFile.writeReal(mDirectedErrors[i].x, encoding);
		errorFile.write("\t", 1, 1);
		errorFile.writeReal(mDirectedErrors[i].y, encoding);
		errorFile.write("\t", 1, 1);

		// closest point
		errorFile.writeReal(mClosestPoints[i].x, encoding);
		errorFile.write("\t", 1, 1);
		errorFile.writeReal(mClosestPoints[i].y, encoding);
		errorFile.write("\t", 1, 1);

		// reconstructed point
		errorFile.writeReal(mPositions[i].x, encoding);
		errorFile.write("\t", 1, 1);
		errorFile.writeReal(mPositions[i].y, encoding);

		errorFile.write("\n", 1, 1);
	}

	errorFile.write("\n\nThe End\n", 1, 1);
}

void Evaluator::save2DMesh(const Path &fileName, const Vector2 *oldPositions, const uint32 *oldIndices,
	const uint64 oldVertexCount, const uint64 oldIndexCount, const bool keepEdges) const
{
	// new vertexCount
	const uint64 edgeCount = oldIndexCount / 2;
	const uint64 additionalCount = (keepEdges ? edgeCount : 0);
	const uint64 newVertexCount = oldVertexCount + additionalCount;

	// 3D positions
	Vector3 *positions = new Vector3[newVertexCount];
	for (uint64 i = 0; i < oldVertexCount; ++i)
		positions[i].set(oldPositions[i].x, oldPositions[i].y, 0.0f);

	// new indices?
	const uint32 *indices = NULL;
	const uint64 newIndexCount = (keepEdges ? oldIndexCount + additionalCount : 0);
	vector<uint32> newIndices;

	if (keepEdges)
	{
		// add vertices to create triangles
		for (uint64 edgeIdx = 0, nextOldIdx = 0, nextVertex = oldVertexCount;
			edgeIdx < edgeCount;
			++edgeIdx, nextOldIdx += 2)
		{
			// get vertex indices
			const uint32 oldIdx0 = oldIndices[nextOldIdx];
			const uint32 oldIdx1 = oldIndices[nextOldIdx + 1];

			// new vertex for triangles to have degenerated triangles / 2D edges in the final ply
			const Vector2 &neighbor0 = oldPositions[oldIdx0];
			const Vector2 &neighbor1 = oldPositions[oldIdx1];

			const Vector2 center = (neighbor0 + neighbor1) * 0.5f;
			positions[nextVertex].set(center.x, center.y, 0.0f);
			//const Vector2 target = center + mGroundTruthEdgeNormals[edgeIdx];
			//const Real length = (neighbor0 - neighbor1).getLength();
			//positions[nextVertex].set(target.x, target.y, 0.0f);

			// add edge / triangle with indexed new vertex
			newIndices.push_back(oldIdx0);
			newIndices.push_back((unsigned int) nextVertex++);
			newIndices.push_back(oldIdx1);
		}
		
		indices = newIndices.data();
	}

	// save data to ply file
	PlyFile file(fileName, File::CREATE_WRITING, true);
	file.saveTriangleMesh(ENCODING_ASCII, true,
		(uint32) newVertexCount, (uint32) newIndexCount,
		NULL, NULL, positions, NULL, NULL, NULL, 0, indices);

	if (keepEdges)
	{
		// free memory
		delete [] positions;
		positions = NULL;
	}
}
