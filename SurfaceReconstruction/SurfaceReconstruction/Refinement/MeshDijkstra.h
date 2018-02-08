/*
 * Copyright (C) 2018 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */

#ifndef _MESH_DIJKSTRA_H_
#define _MESH_DIJKSTRA_H_

#include <algorithm>
#include <map>
#include <vector>
#include "SurfaceReconstruction/Geometry/Surfel.h"
#include "SurfaceReconstruction/Refinement/MeshDijkstraParameters.h"
#include "SurfaceReconstruction/Refinement/RangedVertexIdx.h"
// todo comments

namespace SurfaceReconstruction
{
	class FlexibleMesh;

	class MeshDijkstra
	{
	public:
		struct Comparer
		{
		public:
			inline Comparer(const MeshDijkstra &dijkstra);
			inline bool operator ()(const uint32 leftLocalIdx, const uint32 rightLocalIdx) const;

		public:
			const MeshDijkstra &mDijkstra;
		};

	public:
		MeshDijkstra();
		inline void findVertices(const FlexibleMesh *mesh, const Math::Vector3 *triangleNormals,
			const uint32 *vertexNeighbors, const uint32 *vertexNeighborsOffsets,
			const Surfel &startSurfel, const uint32 *startTriangle,
			const Real maxCosts, const MeshDijkstraParameters &params);
		void findVertices(const FlexibleMesh *mesh, const Math::Vector3 *triangleNormals,
			const uint32 *vertexNeighbors, const uint32 *vertexNeighborsOffsets,
			const Math::Vector3 &referenceNormal, const Math::Vector3 &referencePosition,
			const Math::Vector3 *startNormals, const uint32 *startVertices, const uint32 startVertexCount,
			const Real maxCosts, const Real maxAngleDifference, const Real angularCostsFactor);

		inline const std::vector<uint32> &getOrder() const;
		inline std::vector<RangedVertexIdx> &getVertices();
		inline const std::vector<RangedVertexIdx> &getVertices() const;

		//uint32 getNextBest() const;

	private:
		inline void addToWorkingSet(const uint32 localVertexIdx);
		void buildStartSet(const Math::Vector3 *startNormals, const uint32 *startVertices, const uint32 startVertexCount,
			const Math::Vector3 &referenceNormal, const Math::Vector3 &referencePosition);
		void clear();

		bool getStepGeometry(Math::Vector3 &n, Math::Vector3 &p0, Math::Vector3 &p1,
			const Math::Vector3 *positions, const uint32 v0Idx, const uint32 v1Idx) const;

		void processNeighbor(const uint32 bestLocalIdx, const uint32 globalNeighborIdx,
			const Math::Vector3 &referenceNormal, const Math::Vector3 *positions);

	public:
		static const uint32 INVALID_NODE;

	private:
		// search data
		std::vector<uint32> mOrder;					/// local indices w.r.t. mVertices in ascending costs order
		std::vector<RangedVertexIdx> mVertices;		/// global vertex index and costs w.r.t. the start in random order
		std::map<uint32, uint32> mVisitedVertices;	/// first = key = global vertex index which must be unique, second = value = index w.r.t. mVertices
		std::vector<uint32> mWorkingSet;			/// indices w.r.t. mVertices, in ascending costs order, at border of search region -> first one = lowest costs = next best one

		// search config data
		Real mAngularCostsFactor;
		Real mMaxAngleDifference;
		Real mMaxCosts;

		// mesh data
		const FlexibleMesh *mMesh;
		const Math::Vector3 *mTriangleNormals;
		const uint32 *mVertexNeighborsOffsets;	/// mVertexNeighbors[i] starts at mVertexNeighborsOffsets[i] and ends at (exclusive) mVertexNeighborsOffsets[i + 1];
		const uint32 *mVertexNeighbors;			/// mVertexNeighbors[i] contains the global direct vertex neighbor indices of vertex i
	};

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///   inline function definitions   ////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	inline MeshDijkstra::Comparer::Comparer(const MeshDijkstra &dijkstra) :
		mDijkstra(dijkstra)
	{
		
	}

	inline bool MeshDijkstra::Comparer::operator()(const uint32 leftLocalIdx, const uint32 rightLocalIdx) const
	{
		return mDijkstra.mVertices.at(leftLocalIdx).getCosts() > mDijkstra.mVertices.at(rightLocalIdx).getCosts();
	}

	void MeshDijkstra::addToWorkingSet(const uint32 localVertexIdx)
	{
		const Real costs = mVertices.at(localVertexIdx).getCosts();
		if (costs < mMaxCosts && REAL_MAX != costs)
		{
			mWorkingSet.push_back(localVertexIdx);
			std::push_heap(mWorkingSet.begin(), mWorkingSet.end(),  MeshDijkstra::Comparer(*this));
		}
	}

	inline void MeshDijkstra::findVertices(const FlexibleMesh *mesh, const Math::Vector3 *triangleNormals,
		const uint32 *vertexNeighbors, const uint32 *vertexNeighborsOffsets,
		const Surfel &startSurfel, const uint32 *startTriangle,
		const Real maxCosts, const MeshDijkstraParameters &params)
	{
		const uint32 count = 3;
		const Math::Vector3 startNormals[count] = { startSurfel.mNormal, startSurfel.mNormal, startSurfel.mNormal };

		findVertices(mesh, triangleNormals,	vertexNeighbors, vertexNeighborsOffsets,
			startSurfel.mNormal, startSurfel.mPosition,
			startNormals, startTriangle, count,
			maxCosts, params.getMaxAngleDifference(), params.getAngularCostsFactor());
	}

	inline const std::vector<uint32> &MeshDijkstra::getOrder() const
	{
		return mOrder;
	}

	inline std::vector<RangedVertexIdx> &MeshDijkstra::getVertices()
	{
		return mVertices;
	}

	inline const std::vector<RangedVertexIdx> &MeshDijkstra::getVertices() const
	{
		return mVertices;
	}
}

#endif // _MESH_DIJKSTRA_H_
