/*
 * Copyright (C) 2018 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */

#include <cassert>
#include "Platform/FailureHandling/Exception.h"
#include "SurfaceReconstruction/Geometry/Edge.h"

using namespace FailureHandling;
using namespace std;
using namespace SurfaceReconstruction;

const uint32 Edge::INVALID_IDX = (uint32) -1;

void Edge::addTriangle(const uint32 triangleIdx)
{
	// allowed: up to 1 triangle assigned before addTriangle
	if (Triangle::INVALID_IDX != mTriangles[1])
	{
		cout << "Exception!\n";
		cout << "Cannot add triangle:\n";
		cout << "mVertices = (" << mVertices[0] << ", " << mVertices[1] << ")\n";
		cout << "mTriangles = (" << mTriangles[0] << ", " << mTriangles[1] << ")\n";
		cout << "Member call: addTriangle(" << triangleIdx << ");\n" << endl;

		assert(false);
		throw Exception("Edge::addTriangle: This edge object has already been assigned two triangles. Only up to two triangles per edge are supported!");
	}

	const uint32 t0 = (mTriangles[0] < triangleIdx ? mTriangles[0] : triangleIdx);
	const uint32 t1 = (mTriangles[0] < triangleIdx ? triangleIdx : mTriangles[0]);

	mTriangles[0] = t0;
	mTriangles[1] = t1;
}

void Edge::replaceTriangle(const uint32 oldTriangleIdx, const uint32 newTriangleIdx)
{
	const uint32 keptIndex = (mTriangles[0] == oldTriangleIdx ? mTriangles[1] : mTriangles[0]);
	setTriangles(keptIndex, newTriangleIdx);
}

void Edge::replaceVertex(const uint32 oldVertexIdx, const uint32 newVertexIdx)
{
	const uint32 keptIndex = (mVertices[0] == oldVertexIdx ? mVertices[1] : mVertices[0]);
	setVertices(keptIndex, newVertexIdx);
}

ostream &SurfaceReconstruction::operator <<(ostream &os, const Edge &edge)
{
	os << "Edge vertices = { " << edge.getVertexIndices()[0] << ", " << edge.getVertexIndices()[1] << "}\n";
	os << "Edge triangles = { " << edge.getTriangleIndices()[0] << ", " << edge.getTriangleIndices()[1] << "}\n";

	return os;
}