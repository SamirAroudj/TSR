/*
 * Copyright (C) 2018 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */

#include "SurfaceReconstruction/Geometry/StaticMesh.h"

using namespace Math;
using namespace std;
using namespace Storage;
using namespace SurfaceReconstruction;

// Mesh constants


StaticMesh::StaticMesh(const Path &fileName) : 
	StaticMesh()
{
	Mesh::loadFromFile(fileName);
}

StaticMesh::StaticMesh(const StaticMesh &other) :
	StaticMesh(other.mColors, other.mNormals, other.mPositions, other.mScales, other.mIndices,
		other.mVertexCount, other.mIndexCount)
{

}

StaticMesh::StaticMesh(const vector<Vector3> &colorBuffer, const vector<Vector3> &normalBuffer, const vector<Vector3> &positionBuffer,
	const vector<Real> &scaleBuffer, const vector<uint32> &indexBuffer) :
		StaticMesh(colorBuffer.data(), normalBuffer.data(), positionBuffer.data(), scaleBuffer.data(), indexBuffer.data(),
			(uint32) positionBuffer.size(), (uint32) indexBuffer.size())
{
}

StaticMesh::StaticMesh(const Vector3 *colorBuffer, const Vector3 *normalBuffer, const Vector3 *positionBuffer, const Real *scaleBuffer,
	const uint32 *indexBuffer, const uint32 vertexCount, const uint32 indexCount) :
	StaticMesh(vertexCount, indexCount)
{
	if (0 == vertexCount || 0 == indexCount)
		return;

	const size_t vectorBytes = sizeof(Vector3) * mVertexCount;
	const size_t realBytes = sizeof(Real) * mVertexCount;
	const size_t indexBufferBytes = sizeof(uint32) * mIndexCount;
	
	memcpy(mColors, colorBuffer, vectorBytes);
	memcpy(mNormals, normalBuffer, vectorBytes);
	memcpy(mPositions, positionBuffer, vectorBytes);
	memcpy(mScales, scaleBuffer, realBytes);
	memcpy(mIndices, indexBuffer, indexBufferBytes);
}

StaticMesh::StaticMesh(const uint32 vertexCount, const uint32 indexCount) :
	StaticMesh()
{
	if (0 == vertexCount || 0 == indexCount)	
		return;
	
	mVertexCount = vertexCount;
	mIndexCount = indexCount;
	allocateMemory(mVertexCount, mIndexCount);
}

StaticMesh::StaticMesh() :
	mColors(NULL), mNormals(NULL), mPositions(NULL), mScales(NULL), mIndices(NULL),
	mIndexCount(0), mVertexCount(0)
{

}

//StaticMesh::StaticMesh(const Polybezier<Vector2> *surfaces, const uint32 surfaceCount, const Real height, bool vertexPadding) :
//	mColors(NULL), mNormals(NULL), mPositions(NULL), mScales(NULL), mIndices(NULL),
//	mIndexCount(0), mVertexCount(0)
//{
//	// polybezier -> mesh by considering the polybezier to be in the x-z plane & extruding it along the y-axis by height to create a 3D surface
//	const Vector3 scaleBottom(0.95f, 1.0f, 0.8f); // todo magic numbers
//	const Vector3 scaleTop(0.95f, 1.0f, 0.9f);	
//	//const Vector3 scaleBottom(1.0f, 1.0f, 1.0f); // todo magic numbers
//	//const Vector3 scaleTop(1.0f, 1.0f, 1.0f);
//	const uint32 TRIANGLE_INDICES[12] = { 2, 1, 0, 2, 3, 1, 4, 3, 2, 4, 5, 3 };
//
//	// get vertex & index count
//	for (uint32 surfaceIdx = 0; surfaceIdx < surfaceCount; ++surfaceIdx)
//	{
//		const Polybezier<Vector2> &s = surfaces[surfaceIdx];
//
//		mVertexCount += s.getControlPolygonSize();
//		mIndexCount	 += s.getControlPolygonSize() - 1;
//	}
//	mVertexCount -= 1;
//	mVertexCount *= 12; // polybezier is extruded once along y-axis resulting in 4 triangles with 3 individual vertices each
//	mIndexCount	 *= 12; // two triangles for each polybezier control polygon line
//
//	if (0 == mVertexCount)
//		return;
//
//	mVertexCapacity = mVertexCount;
//	if (vertexPadding)
//		++mVertexCapacity;
//	reserve();
//
//	// create the mesh by extrusion of the polybezier along the y-axis
//	uint32 nextVertex	= 0;
//	uint32 nextIndex	= 0;
//
//	// fill vertex & index buffer
//	for (uint32 surfaceIdx = 0; surfaceIdx < surfaceCount; ++surfaceIdx)
//	{
//		const Polybezier<Vector2> &surface	= surfaces[surfaceIdx];
//		const uint32 pointCount	= surface.getControlPolygonSize();
//		const Real halfHeight	= height * 0.5f;
//
//		// single polybezier
//		for (uint32 pointIdx = 1, previousPointIdx = 0;
//			pointIdx < pointCount; ++pointIdx)
//		{
//			// create a quad for each polybezier line
//			const Vector2 &p0 = surface.getControlPoint(previousPointIdx);
//			const Vector2 &p1 = surface.getControlPoint(pointIdx);
//
//			// 4 triangles per polybezier line
//			const Vector3 positions[6] =
//			{
//				// 3 points left, 3 points right
//				Vector3(p0.x * scaleBottom.x, -halfHeight * scaleBottom.y, p0.y * scaleBottom.z),
//				Vector3(p1.x * scaleBottom.x, -halfHeight * scaleBottom.y, p1.y * scaleBottom.z),
//				Vector3(p0.x, 0, p0.y),
//				Vector3(p1.x, 0, p1.y),
//				Vector3(p0.x * scaleTop.x, halfHeight * scaleTop.y, p0.y * scaleTop.z),
//				Vector3(p1.x * scaleTop.x, halfHeight * scaleTop.y, p1.y * scaleTop.z),
//			};
//
//			for (uint32 triangleIdx = 0; triangleIdx < 4; ++triangleIdx)
//			{
//				const uint32 *idx = TRIANGLE_INDICES + triangleIdx * 3;
//
//				// triangle normal
//				Vector3 normal;
//				const Vector3 &v0 = positions[*(idx + 0)];
//				const Vector3 &v1 = positions[*(idx + 1)];
//				const Vector3 &v2 = positions[*(idx + 2)];
//				Math::computeTriangleNormal(normal, v0, v1, v2);
//
//				// triangle color
//				const Color c = Color::createFromNormal(normal);
//
//				// add 3 vertices for this triangle
//				for (uint32 i = 0; i < 3; ++i, ++nextVertex, ++nextIndex)
//				{
//					mIndices[nextIndex] = nextVertex;
//
//					mPositions[nextVertex] = positions[*(idx + i)];
//					mNormals[nextVertex] = normal;
//					mColors[nextVertex].set(c.getComponents()[0], c.getComponents()[1], c.getComponents()[2]);
//				}
//			}
//
//			previousPointIdx = pointIdx;
//		}
//	}
//
//	assert(nextIndex == mIndexCount);
//	assert(nextVertex == mVertexCount);
//
//	// compute normals & colors from normals
//	computeNormals();
//	for (uint32 vertexIdx = 0; vertexIdx < mVertexCount; ++vertexIdx)
//	{
//		const Color c = Color::createFromNormal(mNormals[vertexIdx]);
//		mColors[vertexIdx].set(c.getRed(), c.getGreen(), c.getBlue());
//	}
//
//	// no scale values available
//	memset(mScales, 0, sizeof(Real) * mVertexCount);
//}

StaticMesh::~StaticMesh()
{
	clear();
}


void StaticMesh::clear()
{
	delete [] mColors;
	mColors = NULL;

	delete [] mNormals;
	mNormals = NULL;

	delete [] mPositions;
	mPositions = NULL;

	delete [] mScales;
	mScales = NULL;

	delete [] mIndices;
	mIndices = NULL;

	mVertexCount = 0;
	mIndexCount	= 0;
}

void StaticMesh::allocateMemory(const uint32 vertexCount, const uint32 indexCount)
{
	clear();

	mVertexCount = vertexCount;
	mIndexCount = indexCount;

	mColors = new Vector3[mVertexCount];
	mNormals = new Vector3[mVertexCount];
	mPositions = new Vector3[mVertexCount];
	mScales = new Real[mVertexCount];

	if (mIndexCount > 0)
		mIndices = new uint32[mIndexCount];
	else
		mIndices = NULL;
}

void StaticMesh::setIndices(const uint32 *indices, const uint32 indexCount)
{
	mIndexCount = indexCount;

	delete [] mIndices;
	mIndices = new uint32[mIndexCount];
	memcpy(mIndices, indices, sizeof(uint32) * mIndexCount);
}
