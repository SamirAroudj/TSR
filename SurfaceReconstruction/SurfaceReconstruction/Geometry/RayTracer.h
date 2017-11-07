/*
 * Copyright (C) 2017 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */
#ifndef _RAY_TRACER_H_
#define _RAY_TRACER_H_

#include <embree2/rtcore.h>
#include <embree2/rtcore_ray.h>
#include <vector>
#include "Graphics/PinholeCamera.h"
#include "Math/Vector3.h"
#include "Patterns/Subject.h"
#include "SurfaceReconstruction/Geometry/Surfel.h"
#include "SurfaceReconstruction/Scene/Samples.h"
#include "Utilities/Size2.h"

// todo comments

namespace SurfaceReconstruction
{
	// forward declarations
	class Mesh;
	class View;

	class RayTracer
	{
	public:
		static inline Math::Vector2 getRelativeSamplingOffset(const uint32 localSamplingCoords[2], const Utilities::Size2<uint32> &raysPerViewSamplePair);

	public:
		RayTracer();
		~RayTracer();
		
		void clear();

		/** Releases any previous scene data and creates a new static scene from the entered mesh.
			Scene creation also includes building an acceleration structure fitting to the entered static mesh.
		@mesh Defines the scene and is forwarded to the internally used ray tracer.
			The mesh must stay valid and static as long as the scene is traced.*/
		void createStaticScene(const Mesh &mesh, const bool coherentSceneTracing = true);
		void createStaticScene(const std::vector<Math::Vector3> &positions, const std::vector<uint32> &indices, const bool coherentSceneTracing = true);
		/** todo 
		positionsSizeInBytes Defines the size of the array positions in bytes and is used to check whether the last vector is readable by SSE instructions.
			The last vector object must be readable using SSE instructions. Thus padding the array so that the last vector is within 16 bytes readable memory is required.  */
		void createStaticScene(const Math::Vector3 *positions, const uint32 vertexCount,
			const uint32 *indices, const uint32 indexCount, const bool coherentSceneTracing = true);
		
		void freeScene();
		
		void renderFromView(GeometryMap *geometryMap, const Utilities::ImgSize &size, const Graphics::PinholeCamera &camera,
			const Math::Matrix3x3 &HPSToNNRayDirWS, const bool backfaceCulling);

		//void filterForBackFaceCulling(int *valid, RTCRayN *ray, const RTCHitN *potentionHit, const size_t N, const bool forOcclusionTest) const;
		
		bool findIntersection(Surfel &surfel, const Math::Vector3 &rayDirWS, const Math::Vector3 &rayStartWS, const bool backFaceCulling = true);
		void findIntersectionsForViewSamplePairs(
			const bool backFaceCulling,	const uint32 startPairIdx, const uint32 endPairIdx, const uint32 rayBatchSize,
			const Utilities::Size2<uint32> &raysPerViewSamplePair, const bool orientLikeView);
		void findIntersectionsAlongMeshNormals(const Math::Vector3 *normals, const Real *searchLengths, 
			const Real searchLengthScaleFactor, const bool backFaceCulling);

		//void findCounterSideIntersections(Real *distances, 
		//	const Math::Vector3 *rayOrigins, const Math::Vector3 *rayDirections, const Real *scales, const uint32 rayCount);

		void findOcclusions(const Math::Vector3 *positions, const uint32 positionCount, const Math::Vector3 &rayOrigin, const bool backFaceCulling);

		/** Computes for each valid Surfel object in geometryMap whether it is visible from camera's point of view.
		todo */
		void findOcclusions(const GeometryMap &geometryMap, const Utilities::ImgSize &size, const Graphics::PinholeCamera &camera, const bool backFaceCulling);
		
		inline bool getBackfaceCulling() const;
		
		/** todo 
		@return Returns the hit triangle index.*/
		inline Math::Vector3 getHit(Math::Vector3 *hitNormal, const uint32 rayIdx) const;
		inline Math::Vector3 getHit(Math::Vector3 &hitNormal, Real baryCoords[2], const uint32 rayIdx) const;
		inline uint32 getHit(Math::Vector3 &hitNormal, Math::Vector3 &hitPosition, const uint32 rayIdx) const;
		inline uint32 getHitTriangleIndex(const uint32 rayIdx) const;
		inline bool getHitValidity(const uint32 rayIdx) const;
		inline void getSurfel(Surfel &surfel, const uint32 rayIdx) const;
		Math::Vector3 getRayDirection(const uint32 rayIdx) const;

		RTCRay &initializeRay(const uint32 rayIdx, const uint32 mask);
		
		bool isOccludedRay(const uint32 rayIdx) const;
		bool isOccludedRay(const uint32 x, const uint32 y, const Utilities::ImgSize &size) const;
		
		inline void setBackFaceCulling(const bool backFaceCulling);

		void setMaximumRayCount(const uint32 maxRayCount);
		
	protected:
		void getHitData(Math::Vector3 *hitNormal, Math::Vector3 &hitPosition, Real &baryCoordsV0, Real &baryCoordsV1, const RTCRay &ray) const;

	private:
		//static void filterFunctionForIntersections(int *valid, void *userData, const RTCIntersectContext *context, RTCRayN *rays, const RTCHitN *potentialHits, const size_t N);
		//static void filterFunctionForOcclusions(int *valid, void *userData, const RTCIntersectContext *context, RTCRayN *rays, const RTCHitN *potentialHits, const size_t N);

		static void globalInitialization();

	private:
		static const int VALID_RAY_PACKET;
		static const int INVALID_RAY_PACKET;

		static bool msGlobalInitialization;

	private:
		RTCDevice mDevice;
		RTCScene mScene;
		RTCSceneFlags mSceneCoherency;
		RTCIntersectContext mIntersectContext;
		RTCRay *mRays;

		// pointers to the shared buffers of the current static scene
		const Math::Vector3 *mMeshPositions;
		const uint32 *mMeshIndices;

		uint32 mMeshIdx;
		uint32 mMeshVertexCount;
		uint32 mRaysSize;
		bool mBackFaceCulling;
	};

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///   inline function definitions   ////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	
	inline Math::Vector2 RayTracer::getRelativeSamplingOffset(const uint32 localSamplingCoords[2], const Utilities::Size2<uint32> &raysPerViewSamplePair)
	{
		// rectangular pattern
		return Math::Vector2
		(
			((Real) localSamplingCoords[0] / (Real) (raysPerViewSamplePair[0] - 1)) - 0.5f,
			((Real) localSamplingCoords[1] / (Real) (raysPerViewSamplePair[1] - 1)) - 0.5f
		);
	}

	inline bool RayTracer::getBackfaceCulling() const
	{
			return mBackFaceCulling;
	}
		
	inline uint32 RayTracer::getHit(Math::Vector3 &hitNormal, Math::Vector3 &hitPosition, const uint32 rayIdx) const
	{
		hitPosition = getHit(&hitNormal, rayIdx);
		return getHitTriangleIndex(rayIdx);
	}

	inline Math::Vector3 RayTracer::getHit(Math::Vector3 *hitNormal, const uint32 rayIdx) const
	{
		Math::Vector3 hitPosition;
		Real baryCoords[2];

		getHitData(hitNormal, hitPosition, baryCoords[0], baryCoords[1], mRays[rayIdx]);
		return hitPosition;
	}
		
	inline Math::Vector3 RayTracer::getHit(Math::Vector3 &hitNormal, Real baryCoords[2], const uint32 rayIdx) const
	{
		Math::Vector3 hitPosition;
		getHitData(&hitNormal, hitPosition, baryCoords[0], baryCoords[1], mRays[rayIdx]);
		return hitPosition;
	}
	
	inline void RayTracer::getSurfel(Surfel &surfel, const uint32 rayIdx) const
	{
		getHitData(&surfel.mNormal, surfel.mPosition, surfel.mBaryCoords[0], surfel.mBaryCoords[1], mRays[rayIdx]);
		surfel.mTriangleIdx = mRays[rayIdx].primID;
	}

	inline uint32 RayTracer::getHitTriangleIndex(const uint32 rayIdx) const
	{
		return mRays[rayIdx].primID;
	}

	inline bool RayTracer::getHitValidity(const uint32 rayIdx) const
	{
		return mRays[rayIdx].geomID != RTC_INVALID_GEOMETRY_ID;
	}

	inline void RayTracer::setBackFaceCulling(const bool backFaceCulling)
	{
		mBackFaceCulling = backFaceCulling;
	}
}

#endif // _RAY_TRACER_H_
 
