/*
 * Copyright (C) 2018 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */

#ifndef _ISLES_REMOVER_H_
#define _ISLES_REMOVER_H_

// todo comments

#include <atomic>
#include <cassert>
#include <vector>
#include "SurfaceReconstruction/Geometry/IVertexChecker.h"

namespace SurfaceReconstruction
{
	class Edge;
	class FlexibleMesh;

	class IslesEraser : public IVertexChecker
	{
	public:
		IslesEraser(const FlexibleMesh &originalMesh, const uint8 *vertexFlags = NULL, const uint8 considerFlags = 0); 
		virtual ~IslesEraser();

		void clear();

		bool computeOffsets(const uint32 minIsleSize);
		
		const std::vector<std::vector<uint32>> &computeRingBordersOfDoomedIsles();

		inline const std::map<uint32, uint32> &getIsleSizes() const;

		const uint32 getNewIndexCount() const;
		const uint32 getNewVertexCount() const;
		inline const uint32 *getEdgeOffsets() const;
		inline const uint32 *getTriangleOffsets() const;
		inline const uint32 *getVertexOffsets() const;

		inline const std::atomic<uint32> *getVerticesToIsles() const;

		bool hasFoundTooSmallIsle(const uint32 minIsleSize);

		virtual bool isBad(const uint32 vertexIdx) const;

	private:
		IslesEraser();
		inline IslesEraser(const IslesEraser &other);
		inline IslesEraser &operator =(const IslesEraser &rhs);

		
		void addIsleBorderEdge(const uint32 edgeIdx0, const uint32 edgeIdx1, const uint32 isleID, const std::vector<uint32> &mapping);

		void computeIsleSizes();
		void computeVerticesToIsles();

		void findBorderRings();

		void findIsles();

		inline bool ignore(const uint32 vertexIdx) const;

		void gatherIsleBorders(const std::vector<uint32> &mapping);

		void mergeIsles();
		bool connectNeighbors();
		void processDoomedTriangleForBorder(const std::vector<uint32> &mapping, const uint32 triangleIdx);
		void propagateLowestIsleIDs();
		void removeTwoTimesPassedEdges(const uint32 borderIdx);
		uint32 setInParallel(const uint32 vertexIdx, const uint32 bestVertexToIsle);
		void sortBorderRingEdges(std::vector<uint32> &ring, std::vector<uint32> &tempRing);

	public:
		static const uint32 INVALID_TRIANGLE_ISLAND;

	private:
		static const uint32 TRI = 3; /// Each face is a triangle and consist of 3 = three ~ tre ~ tri ~ TRI vertices.

	private:
		std::vector<std::vector<uint32>> mIslesBorders;
		std::map<uint32, uint32> mIsleSizes;

		std::atomic<uint32> *mVerticesToIsles;
		uint32 *mEdgeOffsets;
		uint32 *mTriangleOffsets;
		uint32 *mVertexOffsets;
		uint32 mMinIsleSize;

		// old data which is filtered
		const FlexibleMesh &mMesh;
		const uint8 *mVertexFlags;
		const uint8 mConsiderFlags;
	};

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///   inline function definitions   ////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	inline IslesEraser::IslesEraser(const IslesEraser &other) : 
		IslesEraser(other.mMesh, other.mVertexFlags)
	{
		assert(false);
	}

	inline const std::map<uint32, uint32> &IslesEraser::getIsleSizes() const
	{
		return mIsleSizes;	
	}

	inline const uint32 *IslesEraser::getEdgeOffsets() const
	{
		return mEdgeOffsets;
	}

	inline const uint32 *IslesEraser::getTriangleOffsets() const
	{
		return mTriangleOffsets;
	}

	inline const uint32 *IslesEraser::getVertexOffsets() const
	{
		return mVertexOffsets;
	}

	inline const std::atomic<uint32> *IslesEraser::getVerticesToIsles() const
	{
		return mVerticesToIsles;
	}

	inline bool IslesEraser::ignore(const uint32 vertexIdx) const
	{
		if (!mVertexFlags)
			return false;

		return (mConsiderFlags != (mConsiderFlags & mVertexFlags[vertexIdx]));
	}

	inline IslesEraser &IslesEraser::operator =(const IslesEraser &rhs)
	{
		clear();

		assert(false);
		return *this;
	}
}

#endif // _MESH_ISLES_REMOVER_H_
