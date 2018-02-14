/*
 * Copyright (C) 2018 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */

#include <pmmintrin.h>
#include <xmmintrin.h>
#include "Math/MathHelper.h"
#include "Platform/FailureHandling/Exception.h"
#include "SurfaceReconstruction/Geometry/Mesh.h"
#include "SurfaceReconstruction/Geometry/RayTracer.h" 
#include "SurfaceReconstruction/Geometry/Triangle.h"
#include "SurfaceReconstruction/Scene/Camera/Cameras.h"
#include "SurfaceReconstruction/Scene/Scene.h"

using namespace FailureHandling;
using namespace Graphics;
using namespace Math;
using namespace std;
using namespace SurfaceReconstruction;
using namespace Utilities;

const int RayTracer::VALID_RAY_PACKET = -1;
const int RayTracer::INVALID_RAY_PACKET = 0;

bool RayTracer::msGlobalInitialization = false;

RayTracer::RayTracer() :
	mDevice(NULL), mScene(NULL), mRays(NULL),
	mMeshPositions(NULL), mMeshIndices(NULL),
	mMeshIdx((uint32) -1), mRaysSize(0),
	mBackFaceCulling(false)
{
	// basic initialization?
	if (!msGlobalInitialization)
		RayTracer::globalInitialization();
	
	// create device
	const char *config = "";//"start_threads=1,set_affinity=1";
	mDevice = rtcNewDevice(config);
	if (!mDevice)
		throw Exception("Could not create Embree ray tracing device.");
}

void RayTracer::globalInitialization()
{
	// see https://embree.github.io/api.html
	_MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON);
	_MM_SET_DENORMALS_ZERO_MODE(_MM_DENORMALS_ZERO_ON);
	
	msGlobalInitialization = true;
}

RayTracer::~RayTracer()
{
	clear();
	
	// free device
	rtcDeleteDevice(mDevice);
	mDevice = NULL;
}

void RayTracer::clear()
{
	freeScene();
	
	// free rays
	mRaysSize = 0;
	delete [] mRays;
	mRays = NULL;

	mMeshPositions = NULL;
	mMeshIndices = NULL;
	mMeshIdx = (uint32) -1;
	mMeshVertexCount = 0;
	mRaysSize = 0;
}

void RayTracer::createStaticScene(const Mesh &mesh, const bool coherentSceneTracing)
{
	// get mesh data
	const uint32 indexCount = mesh.getIndexCount();
	const uint32 vertexCount = mesh.getVertexCount();
	const Vector3 *positions = mesh.getPositions();
	const uint32 *indices = mesh.getIndices();

	createStaticScene(positions, vertexCount, indices, indexCount, coherentSceneTracing);
}

void RayTracer::createStaticScene(const vector<Vector3> &positions, const vector<uint32> &indices, const bool coherentSceneTracing)
{
	createStaticScene(positions.data(), (uint32) positions.size(),
		indices.data(), (uint32) indices.size(), coherentSceneTracing);
}

void RayTracer::createStaticScene(const Vector3 *positions, const uint32 vertexCount,
	const uint32 *indices, const uint32 indexCount, const bool coherentSceneTracing)
{
	cout << "Creating scene for ray tracing." << endl;

	// free previous scene
	freeScene();

	// update vertexCount
	if (0 == vertexCount)
		return;
	mMeshVertexCount = vertexCount;

	// create scene and add a triangle mesh to represent mesh
	const uint32 triangleCount = indexCount / 3; 
	mSceneCoherency = (coherentSceneTracing ? RTC_SCENE_COHERENT : RTC_SCENE_INCOHERENT);
	const RTCSceneFlags sceneFlags = RTC_SCENE_STATIC | RTC_SCENE_HIGH_QUALITY | RTC_SCENE_ROBUST | mSceneCoherency;
	const RTCAlgorithmFlags sceneAlgorithmFlags = RTC_INTERSECT_STREAM;

	mScene = rtcDeviceNewScene(mDevice, sceneFlags, sceneAlgorithmFlags);
	mMeshIdx = rtcNewTriangleMesh(mScene, RTC_GEOMETRY_STATIC, triangleCount, vertexCount);
	mMeshPositions = positions;
	mMeshIndices = indices;
	
	// share / reuse the index & vertex buffers for ray tracing
	rtcSetBuffer(mScene, mMeshIdx, RTC_INDEX_BUFFER, mMeshIndices, 0, 3 * sizeof(uint32));
	
	// set vertices (default layout == 16 bytes aligned Vector4 positions (4 floats))
	float *vertexBuffer = (float *) rtcMapBuffer(mScene, mMeshIdx, RTC_VERTEX_BUFFER);
	{
		const Real *source = mMeshPositions[0].getData();
			
		#pragma omp parallel for
		for (int64 vertexIdx = 0; vertexIdx < vertexCount;	++vertexIdx)
		{
			float *targetPosition = vertexBuffer + 4 * vertexIdx;
			const Real *sourcePosition = source + 3 * vertexIdx;

			// reformat vertex vertexIdx
			targetPosition[0] = (float) sourcePosition[0];
			targetPosition[1] = (float) sourcePosition[1];
			targetPosition[2] = (float) -sourcePosition[2]; // conversion: left-handed to right-handed system
			targetPosition[3] = 1.0f;
		}
	}
	rtcUnmapBuffer(mScene, mMeshIdx, RTC_VERTEX_BUFFER);
		
//	// forward mesh to ray tracer
//	{
//		// set indices
//		uint32 *indexBuffer = (uint32 *) rtcMapBuffer(mScene, mMeshIdx, RTC_INDEX_BUFFER);
//		uint32 *targetTriangle = indexBuffer;
//		const uint32 *sourceTriangle = indices;
		
//		for (uint32 triangleIdx = 0; triangleIdx < triangleCount;
//			++triangleIdx, targetTriangle += 3, sourceTriangle += 3 )
//		{
//			targetTriangle[0] = sourceTriangle[0];
//			targetTriangle[1] = sourceTriangle[1];
//			targetTriangle[2] = sourceTriangle[2];
//			targetTriangle[3] = -1;
//		}
//		rtcUnmapBuffer(mScene, mMeshIdx, RTC_INDEX_BUFFER);
//	}

	rtcSetUserData(mScene, mMeshIdx, this);
	//rtcSetIntersectionFilterFunctionN(mScene, mMeshIdx, filterFunctionForIntersections);
	//rtcSetOcclusionFilterFunctionN(mScene, mMeshIdx, filterFunctionForOcclusions);
	
	rtcCommit(mScene);

	// set up context for rtcIntersect1M/rtcOccluded1M
	mIntersectContext.flags = (RTC_SCENE_COHERENT == mSceneCoherency ? RTC_INTERSECT_COHERENT : RTC_INTERSECT_INCOHERENT);
	mIntersectContext.userRayExt = NULL;
}

void RayTracer::freeScene()
{
	mMeshPositions = NULL;
	mMeshIndices = NULL;

	if (mMeshIdx != (uint32) -1)
	{
		// not on static scenes //rtcDeleteGeometry(mScene, mMeshIdx);
		mMeshIdx = -1;
	}
	
	rtcDeleteScene(mScene);
	mScene = NULL;
}

void RayTracer::renderFromView(GeometryMap *geometryMap, const ImgSize &size, const PinholeCamera &camera, const Matrix3x3 &HPSToNNRayDirWS, const bool backFaceCulling)
{
	// camera data for ray creations
	const Vector4 &cameraOrigin = camera.getPosition();
	
	// configure ray tracer
	const uint32 rayCount = size.getElementCount();
	setMaximumRayCount(rayCount);
	setBackFaceCulling(backFaceCulling);

	// gather rays
	#pragma omp parallel for
	for (int64 rayIdx = 0; rayIdx < rayCount; ++rayIdx)
	{
		// set ray data
		RTCRay &ray = initializeRay((uint32) rayIdx, (uint32) rayIdx);
			
		// ray origin
		ray.org[0] = (float) cameraOrigin.x;
		ray.org[1] = (float) cameraOrigin.y;
		ray.org[2] = (float) -cameraOrigin.z; // conversion: left-handed to right-handed system
		
		// ray direction
		const uint32 x = (uint32) (rayIdx % size[0]);
		const uint32 y = (uint32) (rayIdx / size[0]);
		Vector3 rayDir = Cameras::getRay(x, y, HPSToNNRayDirWS);
		ray.dir[0] = (float) rayDir.x;
		ray.dir[1] = (float) rayDir.y;
		ray.dir[2] = (float) -rayDir.z; // conversion: left-handed to right-handed system
			
		// ray working area
		ray.tnear = 0.0f;
		ray.tfar = FLT_MAX;
	}
	
	// clear geometryMap?
	if (geometryMap)
	{
		#pragma omp parallel for
		for (int64 rayIdx = 0; rayIdx < rayCount; ++rayIdx)
		{
			// init surfel
			Surfel &surfel = (*geometryMap)[rayIdx];
	
			surfel.mNormal.set(REAL_MAX, REAL_MAX, REAL_MAX);
			surfel.mPosition.set(REAL_MAX, REAL_MAX, REAL_MAX);
			surfel.mBaryCoords[0] = REAL_MAX;
			surfel.mBaryCoords[1] = REAL_MAX;
			surfel.mTriangleIdx = Triangle::INVALID_IDX;
		}
	}
	
	// trace
	rtcIntersect1M(mScene, &mIntersectContext, mRays, rayCount, sizeof(RTCRay));
	
	// fill geometry buffer?
	if (!geometryMap)
		return;

	// get ray tracing results
	#pragma omp parallel for
	for (int64 i = 0; i < rayCount; ++i)
	{
		// valid hit?
		const uint32 rayIdx = (uint32) i;
		const RTCRay &ray = mRays[rayIdx];
		if (ray.geomID == RTC_INVALID_GEOMETRY_ID)
			continue;
			
		// set projected position, normal at that point & triangle idx
		Surfel &surfel = (*geometryMap)[rayIdx];
		getSurfel(surfel, rayIdx);
	}
}

void RayTracer::findOcclusions(const Vector3 *positions, const uint32 positionCount, const Vector3 &rayOrigin, const bool backFaceCulling)
{
	// configure ray tracer
	setMaximumRayCount(positionCount);

	// gather rays
	const int64 rayCount = positionCount;

	// trace views from rayOrigin / camera to positions
	#pragma omp parallel for
	for (int64 i = 0; i < rayCount; ++i)
	{
		// initial invalid ray results
		const uint32 rayIdx = (uint32) i;
		RTCRay &ray = initializeRay(rayIdx, rayIdx);

		// ray direction
		const Vector3 &position = positions[rayIdx];
		const Vector3 toPosition(position.x - rayOrigin.x, position.y - rayOrigin.y, position.z - rayOrigin.z);
		
		// set ray origin and direction
		ray.org[0] = (float) rayOrigin.x;
		ray.org[1] = (float) rayOrigin.y;
		ray.org[2] = (float) -rayOrigin.z; // conversion: left-handed to right-handed system

		ray.dir[0] = (float) toPosition.x;
		ray.dir[1] = (float) toPosition.y;
		ray.dir[2] = (float) -toPosition.z; // conversion: left-handed to right-handed system
			
		// working area
		ray.tnear = 0.0f; // valid rays are traced to the camera at maximum to test for occlusions between the surfel and the camera		
		ray.tfar = (float) (1.0f - Math::EPSILON);
	}
	
	rtcOccluded1M(mScene, &mIntersectContext, mRays, rayCount, sizeof(RTCRay));
}

void RayTracer::findOcclusions(const GeometryMap &geometryMap, const ImgSize &size, const PinholeCamera &camera, const bool backFaceCulling)
{
	// camera data for ray creations
	const Vector4 &camWS = camera.getPosition();
	
	// configure ray tracer
	const uint32 rayCount = size.getElementCount();
	setMaximumRayCount(rayCount);
	setBackFaceCulling(backFaceCulling);

	// gather rays
	#pragma omp parallel for
	for (int64 rayIdx = 0; rayIdx < rayCount; ++rayIdx)
	{
		// valid surfel?
		const Surfel &surfel = geometryMap[rayIdx];
		const Vector3 toCamera(camWS.x - surfel.mPosition.x, camWS.y - surfel.mPosition.y, camWS.z - surfel.mPosition.z);
		const bool invalidRay = Triangle::isInvalidIndex(surfel.mTriangleIdx);
		
		// ray origin & direction
		RTCRay &ray = initializeRay((uint32) rayIdx, (uint32) rayIdx);
			
		ray.org[0] = (float)  surfel.mPosition.x;
		ray.org[1] = (float)  surfel.mPosition.y;
		ray.org[2] = (float) -surfel.mPosition.z; // conversion: left-handed to right-handed system

		ray.dir[0] = (float)  toCamera.x;
		ray.dir[1] = (float)  toCamera.y;
		ray.dir[2] = (float) -toCamera.z; // conversion: left-handed to right-handed system
			
		// working area
		if (!invalidRay)
			ray.tnear = 0.0f; // valid rays are traced to the camera at maximum to test for occlusions between the surfel and the camera		
		ray.tfar = 1.0f;
	}
	
	// trace
	rtcOccluded1M(mScene, &mIntersectContext, mRays, rayCount, sizeof(RTCRay));
}

bool RayTracer::findIntersection(Surfel &surfel, const Math::Vector3 &rayStartWS, const Math::Vector3 &rayDirWS, const bool backFaceCulling)
{
	setMaximumRayCount(1);
	setBackFaceCulling(backFaceCulling);

	// set up ray
	RTCRay &ray = initializeRay(0, 0);
	ray.tnear = 0.0f;
	ray.tfar = FLT_MAX;
	ray.org[0] = (float) rayStartWS.x;
	ray.org[1] = (float) rayStartWS.y;
	ray.org[2] = (float) -rayStartWS.z; // necessary due to different conventions
	ray.dir[0] = (float) rayDirWS.x;
	ray.dir[1] = (float) rayDirWS.y;
	ray.dir[2] = (float) -rayDirWS.z; // necessary due to different conventions

	// trace ray
	rtcIntersect1M(mScene, &mIntersectContext, mRays, 1, sizeof(RTCRay));
	
	// return hit data
	const bool hitSomething = getHitValidity(0);
	if (!hitSomething)
		return false;

	getSurfel(surfel, 0);
	return hitSomething;
}

void RayTracer::findIntersectionsForViewSamplePairs(
	const bool backFaceCulling, const uint32 startPairIdx, const uint32 endPairIdx, const uint32 rayBatchSize,
	const Utilities::Size2<uint32> &raysPerViewSamplePair, const bool orientLikeViews)
{
	cout << "findIntersectionsForViewSamplePairs" << endl;

	// get scene data
	const Scene &scene = Scene::getSingleton();
	const Samples &samples = scene.getSamples();
	const Cameras &cameras = *scene.getCameras();

	// configure ray tracer
	const uint32 pairCount = endPairIdx - startPairIdx;
	const uint32 raysPerPair = raysPerViewSamplePair.getElementCount();
	const uint32 rayCount = pairCount * raysPerPair;

	setMaximumRayCount(rayCount);
	setBackFaceCulling(backFaceCulling);

	// for each ray: from camera to sample
	#pragma omp parallel for
	for (int64 i = 0; i < rayCount; ++i)
	{
		// get indices
		const uint32 rayIdx = (uint32) i;
		const uint32 localPairIdx = rayIdx / raysPerPair;
		const uint32 globalPairIdx = localPairIdx + startPairIdx;
		const uint32 sampleIdx = samples.getSampleIdx(globalPairIdx);
		const uint32 cameraIdx = samples.getViewIdx(globalPairIdx);
		
		// valid ray?
		RTCRay &ray = initializeRay(rayIdx, rayIdx);
		if (!cameras.isValid(cameraIdx))
		{
			ray.geomID = RTC_INVALID_GEOMETRY_ID;
			ray.tnear = FLT_MAX;
			ray.tfar = 0.0f;
			continue;
		}

		// ray start position = camera center
		const View &view = *(views[cameraIdx]);
		const Vector3 startPosWS = view.getPositionWS();

		ray.org[0] = (float)  startPosWS.x;
		ray.org[1] = (float)  startPosWS.y;
		ray.org[2] = (float) -startPosWS.z; // conversion: left-handed to right-handed system

		// build coordinate system for supersampling pattern of current view sample pair
		// sample coordinate system orientation: first basis vector is parallel to viewing direction
		const Vector3 &samplePosWS = samples.getPositionWS(sampleIdx);
		const Real sampleScale = samples.getScale(sampleIdx);
		Vector3 t0, t1;

		if (orientLikeViews)
		{
			// oriented like the sensor which captured the sample
			const Quaternion &cameraOrientation = view.getCamera().getOrientation();		
			cameraOrientation.rotateVector(t0, Vector3(1.0f, 0.0f, 0.0f)); //cameraOrientation.rotateVector(t1, Vector3(0.0f, 1.0f, 0.0f));
			cameraOrientation.rotateVector(t1, Vector3(0.0f, 1.0f, 0.0f));
		}
		else
		{
			// calculating basis orthogonal to ray from camera to sample
			const Vector3 toSamplePosWS = samplePosWS - startPosWS;
			t0 = toSamplePosWS.getOrthogonalVector(true);
			t1 = toSamplePosWS.crossProduct(t0);
			t1.normalize();
		}

		// get remaining data for sampling pattern
		const uint32 localSamplingIdx = (rayIdx % raysPerPair);
		const uint32 localSamplingCoords[2] = { localSamplingIdx % raysPerViewSamplePair[0], localSamplingIdx / raysPerViewSamplePair[1] };

		// ray direction = towards sampling point of sampling pattern of sample patch
		Vector2 offset = getRelativeSamplingOffset(localSamplingCoords, raysPerViewSamplePair);
		offset *= sampleScale;

		const Vector3 targetWS = samplePosWS + t0 * offset.x + t1 * offset.y;
		const Vector3 toTargetWS = targetWS - startPosWS;
		ray.dir[0] = (float)  toTargetWS.x;
		ray.dir[1] = (float)  toTargetWS.y;
		ray.dir[2] = (float) -toTargetWS.z; // conversion: left-handed to right-handed system

		// ray working area
		ray.tnear = 0.0f;
		ray.tfar = FLT_MAX;
	}
	
	cout << "rtcIntersect1M" << endl;
	const uint32 rayBatchCount = (rayCount + rayBatchSize - 1) / (rayBatchSize);
	#pragma omp parallel for schedule(static)
	for (int64 batchIdx = 0; batchIdx < rayBatchCount; ++batchIdx)
	{
		// what rays
		const uint32 processedCount = (uint32) (rayBatchSize * batchIdx);
		const uint32 currentBatchSize = (batchIdx < rayBatchCount - 1 ? rayBatchSize : rayCount - processedCount);
		RTCRay *rayBatch = mRays + processedCount;

		// trace
		rtcIntersect1M(mScene, &mIntersectContext, rayBatch, currentBatchSize, sizeof(RTCRay));
	}
}

void RayTracer::findIntersectionsAlongMeshNormals(const Vector3 *meshNormals, const Real *searchLengths,
	const Real searchLengthScaleFactor, const bool backFaceCulling)
{
	// configure ray tracer
	const uint32 rayCount = mMeshVertexCount;
	setMaximumRayCount(rayCount);
	setBackFaceCulling(backFaceCulling);
	
	// a ray from each vertex along its normal direction
	#pragma omp parallel for
	for (int64 i = 0; i < rayCount; ++i)
	{
		// ray idx, origin & direction
		const uint32 rayIdx = (uint32) i;
		const Vector3 &origin = mMeshPositions[rayIdx];
		const Vector3 &normal = meshNormals[rayIdx];
		const Real &searchLength = searchLengths[rayIdx];
		
		// ray origin & direction
		RTCRay &ray = initializeRay(rayIdx, rayIdx);

		ray.org[0] = (float)  origin.x;
		ray.org[1] = (float)  origin.y;
		ray.org[2] = (float) -origin.z; // conversion: left-handed to right-handed system

		ray.dir[0] = (float)  normal.x;
		ray.dir[1] = (float)  normal.y;
		ray.dir[2] = (float) -normal.z; // conversion: left-handed to right-handed system

		// ray working area
		ray.tnear = 0.0f;
		ray.tfar = (float) (searchLength * searchLengthScaleFactor);
	}
	
	// trace the rays
	rtcIntersect1M(mScene, &mIntersectContext, mRays, rayCount, sizeof(RTCRay));
}

void RayTracer::getHitData(Vector3 *hitNormal, Vector3 &hitPosition, Real &baryCoordsV0, Real &baryCoordsV1, const RTCRay &ray) const
{		
	// get hit triangle
	const uint32 *indices = mMeshIndices + ray.primID * 3;
	const Vector3 triangle[3] = { mMeshPositions[indices[0]], mMeshPositions[indices[1]], mMeshPositions[indices[2]] };
			
	// hit position & barycentric coordinates
	const Real rayW = (1.0f - ray.u - ray.v);
	hitPosition = triangle[0] * rayW + triangle[1] * ray.u + triangle[2] * ray.v;
	baryCoordsV0 = rayW;
	baryCoordsV1 = ray.u;

	// hit normal?
	if (!hitNormal)
		return;
	Math::computeTriangleNormal(*hitNormal, triangle[0], triangle[1], triangle[2]);
}

Math::Vector3 RayTracer::getRayDirection(const uint32 rayIdx) const
{
	// direction data
	const float *dir = mRays[rayIdx].dir;

	// normalized direction considering different conventions (embree <-> current project regarding z)
	Vector3 rayDir(dir[0], dir[1], -dir[2]);
	rayDir.normalize();

	return rayDir;
}

RTCRay &RayTracer::initializeRay(const uint32 rayIdx, const uint32 mask)
{
	RTCRay &ray = mRays[rayIdx];

	ray.instID = RTC_INVALID_GEOMETRY_ID;
	ray.geomID = RTC_INVALID_GEOMETRY_ID;
	ray.primID = RTC_INVALID_GEOMETRY_ID;

	ray.mask = mask;
	ray.time = 0.0f;

	ray.tnear = FLT_MAX;
	ray.tfar = 0.0f;

	return ray;
}

//void RayTracer::filterFunctionForIntersections(int *valid, void *userData, const RTCIntersectContext *context, RTCRayN *rays, const RTCHitN *potentialHits, const size_t N)
//{
//	// filter due to back face culling?
//	const RayTracer &rayTracer = *((RayTracer *) userData);
//	rayTracer.filterForBackFaceCulling(valid, rays, potentialHits, N, false);
//}
//
//void RayTracer::filterFunctionForOcclusions(int *valid, void *userData, const RTCIntersectContext *context, RTCRayN *rays, const RTCHitN *potentialHits, const size_t N)
//{
//	// filter due to back face culling?
//	const RayTracer &rayTracer = *((RayTracer *) userData);
//	rayTracer.filterForBackFaceCulling(valid, rays, potentialHits, N, true);
//}
//
//void RayTracer::filterForBackFaceCulling(int *valid, RTCRayN *rays, const RTCHitN *potentialHits, const size_t N, const bool forOcclusionTest) const
//{
//	// test each valid ray in the packet
//	bool anyHit = false;
//	for (size_t i = 0; i < N; ++i)
//	{
//		// valid ray?
//		const Real tFar = RTCRayN_tfar(rays, N, i);
//		const Real t = RTCHitN_t(potentialHits, N, i);
//		if (t > tFar)
//			continue;
//
//		// get triangle normal & ray direction
//		const Vector3 normal(RTCHitN_Ng_x(potentialHits, N, i),
//							 RTCHitN_Ng_y(potentialHits, N, i),
//							 RTCHitN_Ng_z(potentialHits, N, i));
//
//		// ignore if back face culling && back side hit
//		if (mBackFaceCulling)
//		{
//			// get ray direction
//			const Vector3 rayDir(RTCRayN_dir_x(rays, N, i),
//								 RTCRayN_dir_y(rays, N, i),
//								 RTCRayN_dir_z(rays, N, i));
//			// back side hit?
//			if (rayDir.dotProduct(normal) > 0)
//				continue;
//		}
//		
//		// keep the ray packet
//		anyHit = true;
//
//		// set hit data
//		// only geomID = 0 for occlusion tests
//		if (forOcclusionTest)
//		{
//			RTCRayN_geomID(rays, N, i) = 0;
//			continue;
//		}
//		
//		// inform observers
//		const uint32 primitiveIdx = RTCHitN_primID(potentialHits, N, i);
//		const Real u = RTCHitN_u(potentialHits, N, i);
//		const Real v = RTCHitN_v(potentialHits, N, i);
//
//		// complete ray hit data of intersection test
//		RTCRayN_primID(rays, N, i) = primitiveIdx;
//		RTCRayN_instID(rays, N, i) = RTCHitN_instID(potentialHits, N, i);
//		RTCRayN_geomID(rays, N, i) = RTCHitN_geomID(potentialHits, N, i);
//
//		RTCRayN_Ng_x(rays, N, i) = (float) normal.x;
//		RTCRayN_Ng_y(rays, N, i) = (float) normal.y;
//		RTCRayN_Ng_z(rays, N, i) = (float) normal.z;
//		
//		RTCRayN_u(rays, N, i) = (float) u;
//		RTCRayN_v(rays, N, i) = (float) v;
//
//		RTCRayN_tfar(rays, N, i) = (float) t;
//	}
//	
//	*valid |= (anyHit ? VALID_RAY_PACKET : INVALID_RAY_PACKET);
//}

bool RayTracer::isOccludedRay(const uint32 rayIdx) const
{
	const RTCRay &ray = mRays[rayIdx];
	return (ray.geomID == 0);
}

bool RayTracer::isOccludedRay(const uint32 x, const uint32 y, const ImgSize &size) const
{
	const RTCRay &ray = mRays[y * size[0] + x];
	return (ray.geomID == 0);
}

void RayTracer::setMaximumRayCount(const uint32 maxRayCount)
{
	// already enough memory?
	if (0 != maxRayCount && mRaysSize >= maxRayCount)
		return;
	
	// free previous memory
	delete [] mRays;
	mRays = NULL;
	
	mRaysSize = maxRayCount;
	if (0 == maxRayCount)
		return;
	else
		mRays = new RTCRay[mRaysSize];
}
