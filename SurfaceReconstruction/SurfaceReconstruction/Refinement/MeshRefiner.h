/*
 * Copyright (C) 2018 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */
#ifndef _MESH_REFINER_H_
#define _MESH_REFINER_H_

// todo comments

#include <cassert>
#include <vector>
#include "Math/Vector2.h"
#include "Math/Vector3.h"
#include "Platform/Utilities/Size2.h"
#include "SurfaceReconstruction/Geometry/FlexibleMesh.h"
#include "SurfaceReconstruction/Geometry/RayTracer.h"
#include "SurfaceReconstruction/Scene/IReconstructorObserver.h"

namespace SurfaceReconstruction
{
	class Mesh;

	class MeshRefiner : public Patterns::Subject<IReconstructorObserver>, public IFlexibleMeshObserver
	{
	public:
		template <class T>
		static void normalize(T *weightedSums, const Real *weights, const int64 arraySize,
			const Real minimumWeight, const T &invalidWeightResult);

	public:
		inline const FlexibleMesh &getFlexibleMesh() const;

		inline const std::vector<Math::Vector3> &getVectorField() const;
		inline const std::vector<Real> &getWeightField() const;
		

		virtual void onEdgeMerging(const uint32 targetVertex, const uint32 edgeVertex0, const uint32 edgeVertex1);
		virtual void onEdgeSplitVertex(const uint32 newVertexIdx, const uint32 edgeVertex0, const uint32 edgeVertex1);

		virtual void onFilterData(
			const uint32 *vertexOffsets, const uint32 vertexCount,
			const uint32 *edgeOffsets, const uint32 edgeCount,
			const uint32 *triangleOffsets, const uint32 triangleCount);
		
		virtual void onNewElements(
			const uint32 firstNewVertex, const uint32 newVertexCount,
			const uint32 firstNewEdge, const uint32 newEdgeCount,
			const uint32 firstNewTriangle, const uint32 newTriangleCount);

		virtual void onReserveMemory(const uint32 vertexCapacity, const uint32 edgeCapacity, const uint32 indexCapacity);

	protected:
		MeshRefiner();
		MeshRefiner(const std::string &meshFileName);
		MeshRefiner(const FlexibleMesh &initialMesh);
		virtual ~MeshRefiner();

		void applyMovementField();

		virtual void clear();

		virtual void doSelfCheck();


		virtual void onNewStartMesh();
		
		void resize(const uint32 vertexCount);

		void updateObservers(const uint32 iteration, const std::string extraNameText,
			const IReconstructorObserver::ReconstructionType type) const;

		inline void zeroMovementAndWeightField(std::vector<Real> &weightField);
		inline void zeroMovementField();
		inline void zeroWeightField(std::vector<Real> &weightField);

	private:
		MeshRefiner(const MeshRefiner &copy);
		inline MeshRefiner &operator = (const MeshRefiner &rhs);

	protected:
		// Ray tracer
		RayTracer mRayTracer;

		// Basic mesh data
		FlexibleMesh mMesh;

		// vertices
		std::vector<Math::Vector3> mVectorField;			/// Stores for each vertex where to move it in order to improve mesh quality
		std::vector<Real> mWeightField;
	};

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///   inline function definitions   ////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	template <class T>
	void MeshRefiner::normalize(T *weightedSums, const Real *weightField, const int64 count,
		const Real minimumWeight, const T &invalidWeightResult)
	{
		// normalize weighted sums via weight field
		#pragma omp parallel for
		for (int64 i = 0; i < count; ++i)
		{
			const Real &weight = weightField[i];
			if (weight >= minimumWeight && !Math::isNaN(weight))
				weightedSums[i] /= weight;
			else
				weightedSums[i] = invalidWeightResult;
		}
	}

	inline const FlexibleMesh &MeshRefiner::getFlexibleMesh() const
	{
		return mMesh;		
	}

	inline const std::vector<Math::Vector3> &MeshRefiner::getVectorField() const
	{
		return mVectorField;
	}

	inline const std::vector<Real> &MeshRefiner::getWeightField() const
	{
		return mWeightField;
	}

	inline void MeshRefiner::zeroMovementAndWeightField(std::vector<Real> &weightField)
	{
		zeroMovementField();
		zeroWeightField(weightField);
	}
	
	inline void MeshRefiner::zeroMovementField()
	{
		const int64 vertexCount = mVectorField.size();
	
		// clear movement field
		#pragma omp parallel for
		for (int64 vertexIdx = 0; vertexIdx < vertexCount; ++vertexIdx)
			mVectorField[vertexIdx].set(0.0f, 0.0f, 0.0f);
	}

	inline void MeshRefiner::zeroWeightField(std::vector<Real> &weightField)
	{
		const int64 vertexCount = weightField.size();

		// clear weight field
		#pragma omp parallel for
		for (int64 vertexIdx = 0; vertexIdx < vertexCount; ++vertexIdx)
			weightField[vertexIdx] = 0.0f;
	}	

	inline MeshRefiner &MeshRefiner::operator = (const MeshRefiner &rhs)
	{
		assert(false);
		return *this;
	}
}

#endif // _SURFACE_MESH_REFINER_H_
