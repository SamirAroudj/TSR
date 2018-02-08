/*
 * Copyright (C) 2018 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */
#ifndef _EDGE_H_
#define _EDGE_H_

#include <cassert>
#include <iostream>
#include "SurfaceReconstruction/Geometry/Triangle.h"
#include "Platform/DataTypes.h"

// todo comments

namespace SurfaceReconstruction
{
	class Edge
	{
	public:
		static inline bool isInvalidIndex(const uint32 edgeIdx);

	public:
		inline Edge();
		inline Edge(const uint32 vertex0, const uint32 vertex1, const uint32 triangleIdx);
		inline Edge(const uint32 vertex0, const uint32 vertex1, const uint32 triangle0, const uint32 triangle1);

		void addTriangle(const uint32 triangleIdx);
		
		inline const uint32 getOtherTriangle(const uint32 triangleIdx) const;
		inline const uint32 getOtherVertex(const uint32 vertexIdx) const;
		inline const uint32 *getTriangleIndices() const;
		inline const uint32 *getVertexIndices() const;

		inline bool hasTwoNeighbors() const;

		inline bool isAdjacentToVertex(const uint32 vertexIdx) const;

		void replaceTriangle(const uint32 oldTriangleIdx, const uint32 newTriangleIdx);
		void replaceVertex(const uint32 oldVertexIdx, const uint32 newVertexIdx);

		inline void setTriangles(const uint32 triangle0, const uint32 triangle1);
		inline void setVertices(const uint32 vertex0, const uint32 vertex1);

	public:
		static const uint32 INVALID_IDX;

	private:
		uint32 mTriangles[2];
		uint32 mVertices[2];
	}; 

	std::ostream &operator <<(std::ostream &os, const Edge &edge);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///   inline function definitions   ////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	inline Edge::Edge()
	{
		mTriangles[0] = Triangle::INVALID_IDX;
		mTriangles[1] = Triangle::INVALID_IDX;

		mVertices[0] = (uint32) -1;
		mVertices[1] = (uint32) -1;
	}

	inline Edge::Edge(const uint32 vertex0, const uint32 vertex1, const uint32 triangleIdx) :
		Edge(vertex0, vertex1, triangleIdx, Triangle::INVALID_IDX)
	{
	
	}

	inline Edge::Edge(const uint32 vertex0, const uint32 vertex1, const uint32 triangle0, const uint32 triangle1) 
	{
		setTriangles(triangle0, triangle1);
		setVertices(vertex0, vertex1);
	}

	inline const uint32 Edge::getOtherTriangle(const uint32 triangleIdx) const
	{
		assert(mTriangles[0] == triangleIdx || mTriangles[1] == triangleIdx);
		return (mTriangles[0] == triangleIdx ? mTriangles[1] : mTriangles[0]);
	}
	
	inline const uint32 Edge::getOtherVertex(const uint32 vertexIdx) const
	{
		assert(mVertices[0] == vertexIdx || mVertices[1] == vertexIdx);
		return (mVertices[0] == vertexIdx ? mVertices[1] : mVertices[0]);
	}

	inline const uint32 *Edge::getTriangleIndices() const
	{
		return mTriangles;
	}

	inline const uint32 *Edge::getVertexIndices() const
	{
		return mVertices;
	}

	inline bool Edge::hasTwoNeighbors() const
	{
		return (mTriangles[0] != Triangle::INVALID_IDX) && (mTriangles[1] != Triangle::INVALID_IDX);
	}

	inline bool Edge::isAdjacentToVertex(const uint32 vertexIdx) const
	{
		return (vertexIdx == mVertices[0] || vertexIdx == mVertices[1]);
	}
	
	inline bool Edge::isInvalidIndex(const uint32 edgeIdx)
	{
		return (INVALID_IDX == edgeIdx);
	}

	inline void Edge::setTriangles(const uint32 triangle0, const uint32 triangle1)
	{
		mTriangles[0] = (triangle0 < triangle1 ? triangle0 : triangle1);
		mTriangles[1] = (triangle0 < triangle1 ? triangle1 : triangle0);
	}

	inline void Edge::setVertices(const uint32 vertex0, const uint32 vertex1)
	{
		mVertices[0] = (vertex0 < vertex1 ? vertex0 : vertex1);
		mVertices[1] = (vertex0 < vertex1 ? vertex1 : vertex0);
	}
}

#endif // _EDGE_H_