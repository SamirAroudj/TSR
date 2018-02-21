/*
 * Copyright (C) 2018 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */
#include <iostream>
#include "Platform/FailureHandling/Exception.h"
#include "SurfaceReconstruction/Geometry/Triangle.h"
#include "SurfaceReconstruction/Geometry/Vertex.h"

using namespace FailureHandling;
using namespace std;
using namespace SurfaceReconstruction;

const uint32 Triangle::INVALID_INDEX = (uint32) -1;

bool Triangle::areAdjacent(const uint32 triangleVertices0[3], const uint32 triangleVertices1[3])
{
	// count shared vertices
	uint32 sharedVertices = 0;

	for (uint32 i = 0; i < 3; ++i)
	{
		const uint32 index0 = triangleVertices0[i];

		for (uint32 j = 0; j < 3; ++j)
		{
			const uint32 index1 = triangleVertices1[j];

			if (index0 == index1)
			{
				++sharedVertices;
				break;
			}
		}
	}

	return (2 == sharedVertices);
}

uint32 Triangle::getEdgeIdx(const uint32 indices[3], const uint32 index0, const uint32 index1)
{
	// edge0?
	if (indices[0] == index0 && indices[1] == index1)
		return 0;
	if (indices[0] == index1 && indices[1] == index0)
		return 0;

	// edge1?
	if (indices[1] == index0 && indices[2] == index1)
		return 1;
	if (indices[1] == index1 && indices[2] == index0)
		return 1;

	// edge2?
	if (indices[2] == index0 && indices[0] == index1)
		return 2;
	if (indices[2] == index1 && indices[0] == index0)
		return 2;

	// the line segment between vertices index0 and index1 is not part of the triangle defined by the vertices indices
	return -1;
} 

uint32 Triangle::getOtherVertex(const uint32 indices[3], const uint32 index0, const uint32 index1)
{
	// find & return the index in indices which is not in { index0, index1 }

	// which ones are equal?
	bool equal[2][3] = { { false, false, false }, { false, false, false } };
	for (uint32 i = 0; i < 3; ++i)
	{
		if (index0 == indices[i])
			equal[0][i] = true;
		if (index1 == indices[i])
			equal[1][i] = true;
	}

	// sanity check
	for (uint32 i = 0; i < 2; ++i)
	{
		if (equal[i][0] || equal[i][1] || equal[i][2])
			continue;
		
		cerr << "Invalid Triangle::getOtherVertex()\n";
		cerr << "Triangle " << indices[0] << ", "  << indices[1] << ", " << indices[2] << "\n";
		cerr << "Candidates " << index0 << ", " << index1 << "\n";
		cerr << endl;

		return Vertex::INVALID_INDEX;
	}

	// return the index which is opposite to index 0 and index 1
	for (uint32 i = 0; i < 3; ++i)
		if (!equal[0][i] && !equal[1][i])
			return indices[i];

	return Vertex::INVALID_INDEX;
}

void Triangle::getVerticesInWindingOrder(uint32 ordered[2], const uint32 triangle[3], const uint32 unordered[2])
{
	// test both vertex order possibilities to fit to the triangle winding direction
	for (uint32 directionIdx = 0; directionIdx < 2; ++directionIdx)
	{
		ordered[0] = unordered[directionIdx];
		ordered[1] = unordered[!directionIdx];

		for (uint32 cornerIdx = 0; cornerIdx < 3; ++cornerIdx)
			if (ordered[0] == triangle[cornerIdx] && ordered[1] == triangle[(cornerIdx + 1) % 3] )
				return;
	}

	// not possible -> output error message & throw exception
	const string message("Invalid call of Triangle::getVerticesInWindingOrder.\n");
	cerr << message;
	cerr << "Entered query vertices: " << unordered[0] << ", " << unordered[1] << " are not part of the triangle ";
	cerr << triangle[0] << ", " << triangle[1] << ", " << triangle[2] << "!" << endl;

	throw Exception(message);
}