/*
 * Copyright (C) 2017 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */
#ifndef _TRIANGLE_H_
#define _TRIANGLE_H_

#include "SurfaceReconstruction/Geometry/Vertex.h"
#include "Platform/DataTypes.h"

// todo comments

namespace SurfaceReconstruction
{
	class Triangle
	{
	public:
		static uint32 getEdgeIdx(const uint32 indices[3], const uint32 index0, const uint32 index1);
		
		inline static uint32 getLocalVertexIdx(const uint32 indices[3], const uint32 globalVertexIdx);

		static uint32 getOtherVertex(const uint32 indices[3], const uint32 index0, const uint32 index1);

		static void getVerticesInWindingOrder(uint32 orderedVertices[2], const uint32 triangle[3], const uint32 vertices[2]);

		/** todo 
		@return Returns true if both triangles share a single edge. */
		static bool areAdjacent(const uint32 triangleVertices0[3], const uint32 triangleVertices1[3]);
		
		inline static bool isInvalidIndex(const uint32 triangleIdx);

	public:
		static const uint32 INVALID_IDX;
	}; 

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///   inline & template function definitions   /////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	inline uint32 Triangle::getLocalVertexIdx(const uint32 indices[3], const uint32 globalVertexIdx)
	{
		// where does the triangle consisting of indices have globalVertexIdx?
		for (uint32 cornerIdx = 0; cornerIdx < 3; ++cornerIdx)
			if (indices[cornerIdx] == globalVertexIdx)
				return cornerIdx;
		return Vertex::INVALID_IDX;
	}

	inline bool Triangle::isInvalidIndex(const uint32 triangleIdx)
	{
		return (INVALID_IDX == triangleIdx);
	}
}

#endif // _TRIANGLE_H_
