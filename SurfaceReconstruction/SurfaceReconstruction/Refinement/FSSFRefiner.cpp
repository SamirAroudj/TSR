/*
 * Copyright (C) 2017 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */

#include <iostream>
#include <omp.h>
#include "CollisionDetection/CollisionDetection.h"
#include "Platform/Platform.h"
#include "SurfaceReconstruction/Geometry/FlexibleMesh.h"
#include "SurfaceReconstruction/Geometry/IslesEraser.h"
#include "SurfaceReconstruction/Geometry/StaticMesh.h"
#include "SurfaceReconstruction/Refinement/FSSFParameters.h"
#include "SurfaceReconstruction/Refinement/FSSFRefiner.h"
#include "SurfaceReconstruction/Refinement/MeshDijkstra.h"
#include "SurfaceReconstruction/Scene/Scene.h"
#include "SurfaceReconstruction/Scene/Tree/LeavesIterator.h"
#include "SurfaceReconstruction/Scene/Tree/Tree.h"
#include "SurfaceReconstruction/Scene/Tree/TriangleNodesChecker.h"
#include "SurfaceReconstruction/Scene/View.h"
#include "SurfaceReconstruction/SurfaceExtraction/Occupancy.h"
#include "Utilities/HelperFunctions.h"
#include "Utilities/RandomManager.h"

using namespace CollisionDetection;
using namespace Math;
using namespace Platform;
using namespace std;
using namespace Storage;
using namespace SurfaceReconstruction;
using namespace Utilities;

FSSFRefiner::FSSFRefiner(const FlexibleMesh &initialMesh) :
	FSSFRefiner()
{
	cout << "FSSF refinement.\n";
	cout << "Vertices: " << initialMesh.getVertexCount() << ", edges: " << initialMesh.getEdgeCount() << ", triangles: " << initialMesh.getTriangleCount() << endl;

	
	clear();
	mMesh = initialMesh;
	onNewStartMesh();
}

FSSFRefiner::FSSFRefiner(const string &meshFileName) :
	FSSFRefiner()
{
	loadFromFile(meshFileName);
}

FSSFRefiner::FSSFRefiner() : 
	mDijkstras(NULL), mLocalConfidences(NULL)//, mLocalEdgeWeights(NULL)
{
	// objects for parallel dijkstra searches	
	const uint32 maxNumThreads = omp_get_max_threads();
	mDijkstras = new MeshDijkstra[maxNumThreads];
	//mLocalEdgeWeights = new vector<Real>[maxNumThreads];

	// for median surfel confidence searches within projected sample areas
	const size_t patternSize = mParams.mRaysPerViewSamplePair.getElementCount();
	mLocalConfidences = new vector<Real>[maxNumThreads];

	for (uint32 threadIdx = 0; threadIdx < maxNumThreads; ++threadIdx)
		mLocalConfidences[threadIdx].resize(patternSize, 0.0f);
}

void FSSFRefiner::refine()
{
	cout << "Starting mesh refinement by means of input samples." << endl;

	// check mesh
	const uint32 vertexCount = mMesh.getVertexCount();
	const uint32 indexCount = mMesh.getIndexCount();
	if (0 == vertexCount || 0 == indexCount)
	{
		cout << "FSSFRefiner::refine: Nothing refined as the mesh is empty. (No vertices or triangles)." << endl;
		return;
	}

	// prepare mesh
	for (uint32 iteration = 0; iteration < mParams.mIterationCountInitialSmoothing; ++iteration)
		mMesh.umbrellaSmooth(mVectorField.data(), mWeightField.data(), mParams.mUmbrellaSmoothingLambdaHigh);

	bool converged = false;
	for (uint32 iteration = 0; !converged; ++iteration)
		converged = doRefinementStep(iteration);

	cout << "Finished mesh refinement via samples." << endl;
}

bool FSSFRefiner::doRefinementStep(const uint32 iteration)
{
	cout << "Starting sample-based refinement step. Iteration: " << iteration << "." << endl;
	
	// remove triangles with too small side lengths or altitudes
	simplifyMesh(iteration);
	mMesh.checkEdges();

	// floating scale quantities, such as colors, errors etc. via kernel-based interpolations
	kernelInterpolation();
	mMesh.checkEdges();

	// correct the mesh w.r.t. the samples (move surface towards the samples using the floating scale vertex movement field)
	mMesh.computeVertexScales(mMesh.getScales(), mVertexNeighbors.data(), mVertexNeighborsOffsets.data(), mMesh.getPositions(), mMesh.getVertexCount());
	moveVertices();
	updateObservers(iteration, "FSSFMoved", IReconstructorObserver::RECONSTRUCTION_VIA_SAMPLES);
	
	// regularization & resolution stuff
	enforceRegularGeometry(iteration);

	// error stats
	const bool stop = computeErrorStatistics(iteration);

	// save intermediate result
	saveResult(iteration);
	if (stop)
		return true;

	subdivideMesh();
	mMesh.checkEdges();

	cout << "Finished sample-based refinement step." << endl;
	return false;
}

void FSSFRefiner::kernelInterpolation()
{
	// prepare computation of floating scale quantities from mesh & ray hits
	mMesh.computeNormalsOfTriangles(mTriangleNormals.data());
	mMesh.getVertexNeighbors(mVertexNeighbors, mVertexNeighborsOffsets);

	zeroFloatingScaleQuantities();
	mRayTracer.createStaticScene(mMesh.getPositions(), mMesh.getVertexCount(), mMesh.getIndices(), mMesh.getIndexCount(), true);

	// for all ray hits: apply surface kernel & sample downweighting functions
	cout << "Summing of weighted quantities via surface kernels." << endl;
	const Scene &scene = Scene::getSingleton();
	const Samples &samples = scene.getSamples();
	const uint32 pairCount = samples.getMaxViewConeCount();
	
	// ray trace scene from sensors to samples in batches
	for (uint32 startPairIdx = 0; startPairIdx < pairCount; startPairIdx += EMBREE_PAIR_BATCH_SIZE)
	{
		// find intersections
		uint32 batchSize = EMBREE_PAIR_BATCH_SIZE;
		if (batchSize + startPairIdx > pairCount)
			batchSize = pairCount - startPairIdx;
		mRayTracer.findIntersectionsForViewSamplePairs(true, startPairIdx, startPairIdx + batchSize,
			EMBREE_RAY_BATCH_SIZE, mParams.mRaysPerViewSamplePair);

		// process ray tracing results
		cout << "Processing projected samples." << endl;
		#pragma omp parallel for schedule(dynamic, OMP_PAIR_BATCH_SIZE)
		for (int64 i = 0; i < batchSize; ++i)
		{
			const uint32 localPairIdx = (uint32) i;
			const uint32 globalPairIdx = localPairIdx + startPairIdx;
			const uint32 sampleIdx = samples.getSampleIdx(globalPairIdx);
			ProjectedSample projectedSample;
			
			// get matching confidence for linked view sample pair and current surface estimate
			getProjectedSample(projectedSample, localPairIdx, sampleIdx);
			if (Triangle::INVALID_IDX == projectedSample.mSurfel.mTriangleIdx || projectedSample.mConfidence <= EPSILON)
				continue;
		
			// apply local refinement starting from hit surfel
			processProjectedSample(projectedSample, sampleIdx);
		}
	}

	// normalize floating scale quantities / weighted sums (scales, colors & corrections)
	normalize();
	markUnreliableVerticesViaSupport();
}

void FSSFRefiner::getProjectedSample(ProjectedSample &projectedSample,
	const uint32 localPairIdx, const uint32 sampleIdx) const
{
	// compute weighted mean matching confidence from all ray hits / sampled surfels within projeced sample area
	const Samples &samples = Scene::getSingleton().getSamples();
	const uint32 patternSize = mParams.mRaysPerViewSamplePair.getElementCount();
	const uint32 startRayIdx = localPairIdx * patternSize;
	vector<Real> &localConfidences = mLocalConfidences[omp_get_thread_num()];
	uint32 localSamplingCoords[2];
	uint32 localRayIdx = 0;

	projectedSample.mSurfel.mTriangleIdx = Triangle::INVALID_IDX;
	projectedSample.mConfidence = 0.0f;
	for (localSamplingCoords[1] = 0; localSamplingCoords[1] < mParams.mRaysPerViewSamplePair[1]; ++localSamplingCoords[1])
	{
		for (localSamplingCoords[0] = 0; localSamplingCoords[0] < mParams.mRaysPerViewSamplePair[0]; ++localSamplingCoords[0], ++localRayIdx)
		{
			const uint32 rayIdx = startRayIdx + localRayIdx;
			const Vector2 offset = RayTracer::getRelativeSamplingOffset(localSamplingCoords, mParams.mRaysPerViewSamplePair);

			// sample confidence & (ray hit, sample) mismatch confidence -> kernel weight factor
			localConfidences[localRayIdx] = 0.0f;
			if (!mRayTracer.getHitValidity(rayIdx))
				continue;
		
			// surfel
			Surfel currentSurfelWS;
			mRayTracer.getSurfel(currentSurfelWS, rayIdx);

			// matching confidence
			const Real projectionConfidence = getProjectionConfidence(currentSurfelWS, sampleIdx);
			localConfidences[localRayIdx] = projectionConfidence;
			
			// output surfel if in center && not a low confidence
			if (offset.getLengthSquared() >= EPSILON * EPSILON)
				continue;
			if (projectionConfidence <= EPSILON)
				return;

			projectedSample.mSurfel = currentSurfelWS;
			projectedSample.mViewingDir = mRayTracer.getRayDirection(rayIdx);
		}
	}

	// get & check median value
	vector<Real>::iterator middle = localConfidences.begin() + (localConfidences.size() / 2);
	std::nth_element(localConfidences.begin(), middle, localConfidences.end());

	// goodish projected sample
	const Real medianValue = *middle;
	if (medianValue > EPSILON)
		projectedSample.mConfidence = medianValue * samples.getConfidence(sampleIdx);
}

Real FSSFRefiner::getProjectionConfidence(const Surfel &surfelWS, const uint32 sampleIdx) const
{
	const Samples &samples = Scene::getSingleton().getSamples();
	const Vector3 &sampleNormalWS = samples.getNormalWS(sampleIdx);
	const Vector3 &samplePosWS = samples.getPositionWS(sampleIdx);
	const Real &sampleScale = samples.getScale(sampleIdx);
	const Real angularSupport = mParams.mSupportSampleMaxAngleDifference;
	const Real distanceSupport = mParams.mSupportSampleDistanceBandwidth * sampleScale;

	// probability according to angular distance
	const Real dotProduct = sampleNormalWS.dotProduct(surfelWS.mNormal);
	const Real angleDifference = acosr(clamp(dotProduct, 1.0f, -1.0f));
	if (angleDifference >= angularSupport)
		return 0.0f;

	// probability according to distance between sample and surface (along ray)
	const Real sampleDistance = fabsr((surfelWS.mPosition - samplePosWS).dotProduct(sampleNormalWS));
	if (sampleDistance >= distanceSupport)
		return 0.0f;
	
	const Vector2 stdDeviation(angularSupport / 3.0f, distanceSupport / 3.0f);
	const Real xSq_xVariance = (angleDifference * angleDifference) / (stdDeviation.x * stdDeviation.x);
	const Real ySq_yVariance = (sampleDistance * sampleDistance) / (stdDeviation.y * stdDeviation.y);
	const Real projectionConfidence = expr(-0.5f * (xSq_xVariance + ySq_yVariance));
	
	return projectionConfidence;
}

void FSSFRefiner::processProjectedSample(const ProjectedSample &projectedSample, const uint32 sampleIdx)
{
	// normalized data-driven surface kernel: surface positions get Dijkstra costs-based weights
	// dijkstra search data
	const Samples &samples = Scene::getSingleton().getSamples();
	const Surfel &surfelWS = projectedSample.mSurfel;
	const uint32 *hitTriangle = mMesh.getTriangle(surfelWS.mTriangleIdx);
	const Vector3 hitNormals[3] = { surfelWS.mNormal, surfelWS.mNormal, surfelWS.mNormal };
	const Real surfaceSupportRange = samples.getScale(sampleIdx) * mDijkstraParams.getBandwidthFactor();
	
	// find vertices & edges within the support range of the projected sample
	MeshDijkstra &dijkstra = mDijkstras[omp_get_thread_num()];
	dijkstra.findVertices(&mMesh, mTriangleNormals.data(),
		mVertexNeighbors.data(), mVertexNeighborsOffsets.data(),
		surfelWS.mNormal, surfelWS.mPosition,
		hitNormals, hitTriangle, 3,
		surfaceSupportRange, mDijkstraParams.getMaxAngleDifference(), mDijkstraParams.getAngularCostsFactor());
	
	// compute weights for surface posisionts
	//vector<Real> &edgeWeights = mLocalEdgeWeights[threadIdx];
	const Real sumOfSurfaceWeights = computeSurfaceWeights(dijkstra,/* edgeWeights,*/ surfaceSupportRange);
	if (sumOfSurfaceWeights <= EPSILON)
		return;

	addFloatingScaleQuantities(dijkstra, /*edgeWeights,*/ projectedSample, sampleIdx);
}

Real FSSFRefiner::computeSurfaceWeights(MeshDijkstra &dijkstra, /*vector<Real> &edgeWeights,*/ const Real surfaceSupportRange) const
{
	// get vertices whithin surface kernel range and their Dijkstra/edge-based predecessors
	vector<RangedVertexIdx> &rangedVertices = dijkstra.getVertices();
	const vector<uint32> &order = dijkstra.getOrder();
	const uint32 rangedVertexCount = (uint32) order.size();
	
	// compute non-normalized surface weights for vertices & edges and return their sum
	Real sumOfSurfaceWeights = 0.0f;

	// for all vertices
	for (uint32 localVertexIdx = 0; localVertexIdx < rangedVertexCount; ++localVertexIdx)
	{
		const uint32 nextBestIdx = order[localVertexIdx];
		RangedVertexIdx &rangedIdx = rangedVertices[nextBestIdx];
		const Real costs = rangedIdx.getCosts();

		// weight for the vertex depending on its distance costs
		const Real weight = getKernel2DPoly3Spiky(costs, surfaceSupportRange);
		rangedIdx.setCosts(weight);
		sumOfSurfaceWeights += weight;
	}

	//if (!mFineEstimation)
	//	return sumOfSurfaceWeights;

	//// for all edges
	//edgeWeights.resize(rangedVertexCount);
	//for (uint32 localVertexIdx = 0; localVertexIdx < rangedVertexCount; ++localVertexIdx)
	//{
	//	// valid edge?
	//	const uint32 nextBestIdx = order[localVertexIdx];
	//	const uint32 localPredecessorIdx = rangedVertices[nextBestIdx].getLocalPredecessorIdx();
	//	if (MeshDijkstra::INVALID_NODE == localPredecessorIdx)
	//		continue;

	//	// edge costs
	//	const Real costsV0 = rangedVertices[nextBestIdx].getCosts();
	//	const Real costsV1 = rangedVertices[localPredecessorIdx].getCosts();
	//	const Real costs = (costsV0 + costsV1) * 0.5f;

	//	// weight for the vertex depending on its distance costs
	//	//const Real weight = getGaussian2DForSquaredDistance(edgeCosts * edgeCosts, sampleSurfaceSupportStdDeviation);
	//	const Real weight = getKernel2DPoly3Spiky(costs, surfaceSupportRange);
	//	edgeWeights[localVertexIdx] = weight;
	//	sumOfSurfaceWeights += weight;
	//}

	return sumOfSurfaceWeights;
}

void FSSFRefiner::addFloatingScaleQuantities(const MeshDijkstra &dijkstra, //const vector<Real> &edgeWeights,
	const ProjectedSample &projectedSample, const uint32 sampleIdx)
{
	// sample & mesh data
	const Samples &samples = Scene::getSingleton().getSamples();
	const Vector3 *meshPositions = mMesh.getPositions();
	const Vector3 correctionDir = projectedSample.mSurfel.mNormal;
	const Vector3 &sampleColor = samples.getColor(sampleIdx);
	const Vector3 &sampleNormalWS = samples.getNormalWS(sampleIdx);
	const Vector3 &samplePosWS = samples.getPositionWS(sampleIdx);
	const Real sampleScale = samples.getScale(sampleIdx);

	// dijkstra data
	const vector<RangedVertexIdx> &rangedVertices = dijkstra.getVertices();
	const vector<uint32> &order = dijkstra.getOrder();
	const uint32 rangedVertexCount = (uint32) order.size();
	
	// add weights & weighted quantities to all vertices & edges in range
	for (uint32 localVertexIdx = 0; localVertexIdx < rangedVertexCount; ++localVertexIdx)
	{
		// get vertex data
		const uint32 nextBestIdx = order[localVertexIdx];
		const RangedVertexIdx &rangedVertex = rangedVertices[nextBestIdx];
		const uint32 vertexIdx = rangedVertex.getGlobalVertexIdx();
		const Real weight = rangedVertex.getCosts() * projectedSample.mConfidence;

		// add influence of ray to sample on vertex
		addFloatingScaleQuantities(&mMesh.getColor(vertexIdx), mVectorField[vertexIdx],	mSurfaceErrors[vertexIdx], mWeightField[vertexIdx],
			correctionDir, meshPositions[vertexIdx],
			&sampleColor, sampleNormalWS, samplePosWS, weight);

		//// also compute errors at edges?
		//if (!mFineEstimation)
		//	continue;

		//// valid edge?
		//const uint32 localPredecessorIdx = rangedVertex.getLocalPredecessorIdx();
		//if (MeshDijkstra::INVALID_NODE == localPredecessorIdx)
		//	continue;

		//// get edge data
		//const uint32 predecessorIdx = rangedVertices[localPredecessorIdx].getGlobalVertexIdx();
		//const uint32 edgeIdx = mMesh.getEdgeIndex(vertexIdx, predecessorIdx);
		//const Vector3 edgeCenterWS = (meshPositions[vertexIdx] + meshPositions[predecessorIdx]) * 0.5f;
		////Vector3 edgeNormalWS = meshNormals[vertexIdx] + meshNormals[predecessorIdx];
		////edgeNormalWS.normalize();

		//// add influence of ray to sample on vertex
		//addFloatingScaleQuantities(NULL, mEdgeVectorField[edgeIdx], mEdgeWeights[edgeIdx],
		//	correctionDirWS, edgeNormalWS, edgeCenterWS,
		//	NULL, sampleNormalWS, samplePosWS, edgeWeights[localVertexIdx] * confidence);
	}
}

void FSSFRefiner::addFloatingScaleQuantities(Vector3 *targetColor, Vector3 &targetSumOfCorrections,
	Real &targetSumOfSurfaceErrors, Real &targetSumOfWeights,
	const Vector3 &correctionDirWS, const Vector3 &surfacePosWS,
	const Vector3 *sampleColor, const Vector3 &sampleNormalWS, const Vector3 &samplePosWS,
	const Real weight)
{
	// compute final hit weight (= influence strength)
	if (weight <= EPSILON)
		return;

	// compute capped weighted movement towards sample
	// todo important todo 42 what is better surfac or hit normal?
	Vector3 correction;
	bool reasonable = getSurfaceErrorCorrection(correction,
		correctionDirWS, surfacePosWS, sampleNormalWS, samplePosWS);
	if (!reasonable)
		return;
	
	// update global vertex weight
	#pragma omp atomic
	targetSumOfWeights += weight;

	// update global vertex movement
	const Vector3 weightedCorrection = correction * weight;
	#pragma omp atomic
	targetSumOfCorrections.x += weightedCorrection.x;
	#pragma omp atomic
	targetSumOfCorrections.y += weightedCorrection.y;
	#pragma omp atomic
	targetSumOfCorrections.z += weightedCorrection.z;		

	// update surface error
	//const Real lengthSq = correction.getLengthSquared();
	//const Real loss = logr(1.0f + lengthSq);
	//const Real loss = correction.getLength();
	const Real loss = logr(1.0f + correction.getLength());
	#pragma omp atomic
	targetSumOfSurfaceErrors += loss * weight;

	// update global color
	if (!targetColor || !sampleColor)
		return;

	// compute weighted color
	const Vector3 weightedColor = *sampleColor * weight;

	// add color influence
	#pragma omp atomic
	targetColor->x += weightedColor.x;
	#pragma omp atomic
	targetColor->y += weightedColor.y;
	#pragma omp atomic
	targetColor->z += weightedColor.z;	
}

bool FSSFRefiner::getSurfaceErrorCorrection(Vector3 &correction, 
	const Vector3 &correctionDirection, const Vector3 &surfacePositionWS,
	const Vector3 &sampleNormalWS, const Vector3 &samplePosWS) const
{
	const Vector3 diff = surfacePositionWS - samplePosWS;
	const Real distanceToSamplePlane = diff.dotProduct(sampleNormalWS);

	// make sure that the point is on the sample plane if we do not move it along the sample normal but surface normal
	const Real cosAngle = correctionDirection.dotProduct(sampleNormalWS); 
	if (cosAngle < cosr(mDijkstraParams.getMaxAngleDifference()))
		return false;

	correction = correctionDirection * (-distanceToSamplePlane / cosAngle);
	return true;
}

void FSSFRefiner::zeroFloatingScaleQuantities()
{
	cout << "Initial floating scale quantities." << endl;

	// base & mesh data
	zeroMovementAndWeightField(mWeightField);
	mMesh.zeroColors();
	mMesh.zeroScales();

	// zero surface errors
	const uint32 vertexCount = mMesh.getVertexCount();
	#pragma omp parallel for
	for (int64 vertexIdx = 0; vertexIdx < vertexCount; ++vertexIdx)
		mSurfaceErrors[vertexIdx] = 0.0f;
	
	//if (!mFineEstimation)
	//	return;

	//// zero edge data
	//const int64 edgeCount = mMesh.getEdgeCount();

	//// zero edge correction vectors
	//#pragma omp parallel for
	//for (int64 edgeIdx = 0; edgeIdx < edgeCount; ++edgeIdx)
	//	mEdgeVectorField[edgeIdx].set(0.0f, 0.0f, 0.0f);
	//
	//// zero edge scales
	//#pragma omp parallel for
	//for (int64 edgeIdx = 0; edgeIdx < edgeCount; ++edgeIdx)
	//	mEdgeScales[edgeIdx] = 0.0f;

	//// zero edge weights
	//#pragma omp parallel for
	//for (int64 edgeIdx = 0; edgeIdx < edgeCount; ++edgeIdx)
	//	mEdgeWeights[edgeIdx] = 0.0f;
}

void FSSFRefiner::normalize()
{
	cout << "Normalizing weighted sums." << endl;

	const Vector3 WEAK_SUPPORT_COLOR(0.757255f, 0.150784f, 0.520784f);
	const Vector3 ZERO(0.0f, 0.0f, 0.0f);
	
	// vertex data
	const int64 vertexCount = mMesh.getVertexCount();
	MeshRefiner::normalize<Vector3>(mMesh.getColors(), mWeightField.data(), vertexCount, mParams.mSupportWeakThreshold, WEAK_SUPPORT_COLOR);
	MeshRefiner::normalize<Vector3>(mVectorField.data(), mWeightField.data(), vertexCount, mParams.mSupportWeakThreshold, ZERO);
	MeshRefiner::normalize<Real>(mSurfaceErrors.data(), mWeightField.data(), vertexCount, mParams.mSupportWeakThreshold, REAL_MAX);

	//if (!mFineEstimation)
	//	return;

	//// edge data
	//const int64 edgeCount = mMesh.getEdgeCount();
	//MeshRefiner::normalize<Vector3>(mEdgeVectorField.data(), mEdgeWeights.data(), edgeCount, mSupportWeakThreshold, ZERO);
	//MeshRefiner::normalize<Real>(mEdgeScales.data(), mEdgeWeights.data(), edgeCount, mSupportWeakThreshold, REAL_MAX);
}

void FSSFRefiner::markUnreliableVerticesViaSupport()
{
	cout << "Finding unreliable vertices." << endl;

	// get scene, occupancy, tree, nodes
	const Scene &scene = Scene::getSingleton();
	const Occupancy &occupancy = *scene.getOccupancy();
	const Tree &tree = *scene.getTree();
	const Nodes &nodes = tree.getNodes();
	const Scope rootScope(tree.getRootScope());

	// get mesh data
	const uint32 vertexCount = mMesh.getVertexCount();

	// unreliable if no surface support
	#pragma omp parallel for
	for (int64 i = 0; i < vertexCount; ++i)
	{
		// unreliable if unsupported
		const uint32 vertexIdx = (uint32) i;
		const Real weight = mWeightField[vertexIdx];
		if (weight < mParams.mSupportWeakThreshold || isNaN(weight))
		{
			mVertexStates[vertexIdx] |= UNSUPPORTED;
			continue;
		}

		mVertexStates[vertexIdx] &= ~UNSUPPORTED;
	}
}

void FSSFRefiner::moveVertices()
{	
	const uint32 vertexCount = mMesh.getVertexCount();

	#pragma omp parallel for
	for (int64 i = 0; i < vertexCount; ++i)
	{
		// get & check vertex
		const uint32 vertexIdx = (uint32) i;
		if (isBad(vertexIdx))
			continue;

		// position is worse than oldPosition? -> move back
		Real &newError = mSurfaceErrors[vertexIdx];
		Real &lowestError = mBestSurfaceErrors[vertexIdx];
		if (fabsr(newError) >= mParams.mSurfaceErrorRelativeThreshold * fabsr(lowestError))
		{
			newError = lowestError;
			mMesh.setPosition(mBestPositions[vertexIdx], vertexIdx);
			continue;
		}

		// position & its error is okayish -> continue moving
		Vector3 &position = mMesh.getPosition(vertexIdx);

		// new best position with lowest error?
		if (newError < lowestError)
		{
			lowestError = newError;
			mBestPositions[vertexIdx] = position;
		}

		// move towards a better reconstruction! (hopefully)
		position += mVectorField[vertexIdx];
	}
}

void FSSFRefiner::enforceRegularGeometry(const uint32 iteration)
{
	cout << "Enforcing regular triangles." << endl;
	
	if (iteration > 0)
	{
		// move spiky geometry back
		for (uint32 smoothingIt = 0; true; ++smoothingIt)
		{
			findSpikyGeometry(iteration, smoothingIt);
			if (!moveSpikyGeometryBack())
				break;
		}
	}

	// smooth remaining spiky geometry
	for (uint32 smoothingIt = 0; true; ++smoothingIt)
	{
		findSpikyGeometry(iteration, smoothingIt);
		if (!smoothUntilConvergence(SPIKY))
			break;
	}

	// deletion of unsupported small parts
	enforceRegularGeometryForOutliers(iteration);
	
	mMesh.umbrellaSmooth(mVectorField.data(), mWeightField.data(), mParams.mUmbrellaSmoothingLambdaLow);
	updateObservers(iteration, "FSSFMoreRegular", IReconstructorObserver::RECONSTRUCTION_VIA_SAMPLES);
}

void FSSFRefiner::findSpikyGeometry(const uint32 FSSFIteration, const uint32 smoothingIt)
{
	cout << "Smoothing spiky geometry.\n";
	
	// find spiky geometry
	// angles
	computeVertexAngles();
	averageVertexAngles();

	// spikyness according to increased triangle sizes or sharp angles
	const uint32 vertexCount = mMesh.getVertexCount();

	#pragma omp parallel for
	for (int64 i = 0; i < vertexCount; ++i)
	{
		// initial state
		const uint32 vertexIdx = (uint32) i;
		mVertexStates[vertexIdx] &= ~SPIKY;

		// not spiky due to angles and triangle size?
		if (mAverageAngles[0][vertexIdx] < mParams.mSpikyGeometryAngleThreshold)
		{
			const Real oldScale = mMesh.getScale(vertexIdx);
			const Real threshold = mParams.mSpikyGeometryRelativeScaleThreshold * oldScale;
			const Real newScale = mMesh.computeVertexScale(mVertexNeighbors.data(), mVertexNeighborsOffsets.data(), mMesh.getPositions(), vertexIdx);
			if (newScale < threshold)
				continue;
		}

		// spiky
		mVertexStates[vertexIdx] |= SPIKY;
	}
}

	//FlexibleMesh copy(mMesh);
		//// spiky due to angles?
		//if (mAverageAngles[0][vertexIdx] >= mParams.mSpikyGeometryAngleThreshold)
		//{
		//	copy.setColor(Vector3(0.0f, 0.0f, 1.0f), vertexIdx);
		//}
		//else // spiky due to scale?
		//{
		//	const Real oldScale = mMesh.getScale(vertexIdx);
		//	const Real threshold = mParams.mSpikyGeometryRelativeScaleThreshold * oldScale;
		//	const Real newScale = mMesh.computeVertexScale(mVertexNeighbors.data(), mVertexNeighborsOffsets.data(), mMesh.getPositions(), vertexIdx);
		//	if (newScale < threshold)
		//		continue;
		//	
		//	copy.setColor(Vector3(1.0f, 0.0f, 0.0f), vertexIdx);
		//}

	//string text = "ColoredSpikyGeometry";
	//text += (char) ('A' + smoothingIt);
	//saveMesh(copy, FSSFIteration, text.data());

void FSSFRefiner::computeVertexAngles()
{
	// proper normals
	mMesh.computeNormalsWeightedByAngles();
	mMesh.computeNormalsOfTriangles(mTriangleNormals.data());
	
	// get mesh data
	const Edge *edges = mMesh.getEdges();
	const vector<uint32> *verticesToEdges = mMesh.getVerticesToEdges();
	const uint32 vertexCount = mMesh.getVertexCount();

	// angles w.r.t. neighbors
	#pragma omp parallel for
	for (int64 i = 0; i < vertexCount; ++i)
	{
		// get & check normal
		const uint32 vertexIdx = (uint32) i;
		const Vector3 &normal = mMesh.getNormal(vertexIdx);
		if (normal.hasInfiniteComponent() || normal.hasNaNComponent())
		{
			mAverageAngles[0][vertexIdx] = 42000.0f;
			mVertexStates[vertexIdx] |= SPIKY;
			continue;
		}
		
		// for each vertex: sum of angles w.r.t. direct neighbor triangles
		const vector<uint32> &vertexToEdges = verticesToEdges[vertexIdx];
		const uint32 edgeCount = (uint32) vertexToEdges.size();
		Real sum = 0.0f;

		for (uint32 localEdgeIdx = 0; localEdgeIdx < edgeCount; ++localEdgeIdx)
		{
			const uint32 globalEdgeIdx = vertexToEdges[localEdgeIdx];
			const Edge &edge = edges[globalEdgeIdx];
			const uint32 otherVertexIdx = edge.getOtherVertex(vertexIdx);

			Vector3 triangleNormals[2];
			mMesh.getAdjacentTriangleNormals(triangleNormals[0], triangleNormals[1],
				edge.getVertexIndices()[0], edge.getVertexIndices()[1]);

			for (uint32 sideIdx = 0; sideIdx < 2; ++sideIdx)
			{
				const Real cosAngle = clamp(normal.dotProduct(triangleNormals[sideIdx]), 1.0f, -1.0f);
				const Real angle = acosr(cosAngle);

				sum += angle;
			}
		}

		sum /= 2 * edgeCount;
		mAverageAngles[0][vertexIdx] = sum;
	}
}

void FSSFRefiner::averageVertexAngles()
{
	// for each vertex: local sum of sum of angles w.r.t. direct neighbors
	const uint32 vertexCount = mMesh.getVertexCount();
	const uint32 stepCount = 2; // todo important magic number 42

	for (uint32 step = 0; step < stepCount; ++step)
	{
		#pragma omp parallel for
		for (int64 i = 0; i < vertexCount; ++i)
		{
			const uint32 vertexIdx = (uint32) i;
			const uint32 startNeighborIdx = mVertexNeighborsOffsets[vertexIdx];
			const uint32 endNeighborIdx = mVertexNeighborsOffsets[vertexIdx + 1];
			Real sum = mAverageAngles[0][vertexIdx];

			for (uint32 localNeighborIdx = startNeighborIdx; localNeighborIdx < endNeighborIdx; ++localNeighborIdx)
			{
				const uint32 globalNeighborIdx = mVertexNeighbors[localNeighborIdx];
				sum += mAverageAngles[0][globalNeighborIdx];
			}

			sum /= endNeighborIdx - startNeighborIdx;
			mAverageAngles[1][vertexIdx] = sum;
		}

		mAverageAngles[0].swap(mAverageAngles[1]);
	}
	
	//saveAngleColoredMesh(iteration);
}

bool FSSFRefiner::moveSpikyGeometryBack()
{
	const uint32 vertexCount = mMesh.getVertexCount();
	bool anyRevert = false;

	// move spiky vertices back to old positions
	#pragma omp parallel for
	for (int64 i = 0; i < vertexCount; ++i)
	{
		// spiky?
		const uint32 vertexIdx = (uint32) i;
		if (0 == (SPIKY & mVertexStates[vertexIdx]))
			continue;

		// not already reverted?
		const Real error = mSurfaceErrors[vertexIdx];
		const Real bestError = mBestSurfaceErrors[vertexIdx];
		if (EPSILON > fabsr(error - bestError))
			continue;

		// revert
		anyRevert = true;
		mMesh.setPosition(mBestPositions[vertexIdx], vertexIdx);
		mSurfaceErrors[vertexIdx] = bestError;
	}

	return anyRevert;
}

void FSSFRefiner::enforceRegularGeometryForOutliers(const uint32 iteration)
{
	// any outliers?
	cout << "Finding outlier vertices." << endl;
	findOutliersUsingReliability();

	// outlier isles which will be replaced by flat geometry?
	IslesEraser outlierFinder(mMesh, mVertexStates.data(), OUTLIER);
	if (!outlierFinder.hasFoundTooSmallIsle(mParams.mOutlierIsleMinKeepingSize))
		return;

	// nice isles for easy deletion & easy hole filling
	maskLargeOutlierIsles(outlierFinder);
	createWellFormedOutlierIsles();
	saveOutlierColoredMesh(iteration);

	// offsets for removal of small isles of outliers
	IslesEraser outlierEraser(mMesh, mVertexStates.data(), OUTLIER);
	if (!outlierEraser.computeOffsets(mParams.mOutlierIsleMinKeepingSize))
		return;

	// replace bad geometry by simple & flat geometry
	const vector<vector<uint32>> &holeBorders = outlierEraser.computeRingBordersOfDoomedIsles();
	mMesh.deleteGeometry(outlierEraser.getVertexOffsets(), outlierEraser.getEdgeOffsets(), outlierEraser.getTriangleOffsets());
	fillHoles(holeBorders);
	mMesh.checkEdges();

	// remove small, isolated geometry & smooth spiky stuff
	mMesh.deleteIsolatedGeometry(Scene::getSingleton().getMinIsleSize());
	mMesh.checkEdges();
}

void FSSFRefiner::findOutliersUsingReliability()
{
	cout << "Finding unsupported vertices." << endl;
	const uint32 vertexCount = mMesh.getVertexCount();

	// init: all inliers
	#pragma omp parallel for
	for (int64 i = 0; i < vertexCount; ++i)
		mVertexStates[i] &= ~(OUTLIER | BORDER_INLIER);
	
	// outliers = unreliable ones & direct neighbors
	#pragma omp parallel for
	for (int64 i = 0; i < vertexCount; ++i)
	{
		// get data of vertex vertexIdx
		const uint32 vertexIdx = (uint32) i;
		if (0 == (UNSUPPORTED & mVertexStates[vertexIdx]))
			continue;

		// outliers = { vertexIdx, direct neighbors (vertexIdx) }
		const uint32 startNeighborIdx = mVertexNeighborsOffsets[vertexIdx];
		const uint32 endNeighborIdx = mVertexNeighborsOffsets[vertexIdx + 1];
		for (uint32 localNeighborIdx = startNeighborIdx; localNeighborIdx < endNeighborIdx; ++localNeighborIdx)
		{
			const uint32 globalNeighborIdx = mVertexNeighbors[localNeighborIdx];
			mVertexStates[globalNeighborIdx] |= OUTLIER;
		}
		mVertexStates[vertexIdx] |= OUTLIER;
	}
}

void FSSFRefiner::maskLargeOutlierIsles(const IslesEraser &outlierFinder)
{
	// mark outliers of too large isles as inliers
	cout << "Masking large outlier isles." << endl;
	const uint32 vertexCount = mMesh.getVertexCount();

	#pragma omp parallel for
	for (int64 i = 0; i < vertexCount; ++i)
	{
		// outlier of an isle which is not removed?
		const uint32 vertexIdx = (uint32) i;
		uint8 &state = mVertexStates[vertexIdx];
		if (0 == (OUTLIER & state))
			continue;

		// mark as inlier as removing too large isles is a bad idea since the holes cannot be filled reasonably
		if (!outlierFinder.isBad(vertexIdx))
			state &= ~OUTLIER;
	}
}

void FSSFRefiner::createWellFormedOutlierIsles()
{	
	cout << "Creating well formed isles." << endl;
	markBorderInliers();

	while (true)
	{
		mTempVertices.clear();
		findIsolatedInliers();
		addToOutliers(mTempVertices);

		mTempVertices.clear();
		if (!findInvalidBorderEdges())
			break;
		addToOutliers(mTempVertices);
	}
}

void FSSFRefiner::markBorderInliers()
{
	// find border inliers = inliers which are direkt neighbors of outliers
	cout << "Finding vertices at borders to bad geometry." << endl;
	const uint32 vertexCount = mMesh.getVertexCount();

	#pragma omp parallel for
	for (int64 i = 0; i < vertexCount; ++i)
	{
		const uint32 vertexIdx = (uint32) i;
		if (OUTLIER & mVertexStates[vertexIdx])
			continue;

		if (!hasNeighbor(OUTLIER, vertexIdx))
			continue;

		mVertexStates[vertexIdx] |= BORDER_INLIER;
	}
}

bool FSSFRefiner::findIsolatedInliers()
{
	const uint32 vertexCount = mMesh.getVertexCount();

	// mark isolated vertices with no inlier neighbor as outliers
	#pragma omp parallel for
	for (int64 i = 0; i < vertexCount; ++i)
	{
		const uint32 vertexIdx = (uint32) i;
		if (OUTLIER & mVertexStates[vertexIdx])
			continue;

		const uint32 noPureInlierCount = getNeighborCount(OUTLIER | BORDER_INLIER, vertexIdx);
		const uint32 neighborCount = (mVertexNeighborsOffsets[vertexIdx + 1] - mVertexNeighborsOffsets[vertexIdx]);
		if (noPureInlierCount == neighborCount)
		{
			#pragma omp critical (OMPPushBackNewOutlier)
			mTempVertices.push_back(vertexIdx);
		}
	}

	return !mTempVertices.empty();
}

bool FSSFRefiner::findInvalidBorderEdges()
{
	const uint32 edgeCount = mMesh.getEdgeCount();
	const Edge *edges = mMesh.getEdges();

	// each border edge must have exactly one opposite outlier vertex and one opposite inlier vertex
	// otherwise it is not a proper border
	#pragma omp parallel for
	for (int64 i = 0; i < edgeCount; ++i)
	{
		// border edge?
		const Edge &edge = edges[i];
		const uint32 *edgeV = edge.getVertexIndices();
		const uint32 edgeAndState = (mVertexStates[edgeV[0]] & mVertexStates[edgeV[1]]);
		if (0 == (BORDER_INLIER & edgeAndState))
			continue;

		// get vertex states of vertices opposite to the edge -> valid border edge?
		const uint32 *triangleIndices = edge.getTriangleIndices();
		const uint32 *triangles[2] = { mMesh.getTriangle(triangleIndices[0]), mMesh.getTriangle(triangleIndices[1]) };
		const uint32 oppositeVertices[2] = { Triangle::getOtherVertex(triangles[0], edgeV[0], edgeV[1]), Triangle::getOtherVertex(triangles[1], edgeV[0], edgeV[1]) };
		const uint32 states[2] = { mVertexStates[oppositeVertices[0]], mVertexStates[oppositeVertices[1]] };

		if ((OUTLIER == (OUTLIER & states[0])) && (0 == ((BORDER_INLIER | OUTLIER) & states[1])))
			continue;
		if ((OUTLIER == (OUTLIER & states[1])) && (0 == ((BORDER_INLIER | OUTLIER) & states[0])))
			continue;

		#pragma omp critical (OMPPushBackNewOutlier)
		mTempVertices.push_back(edgeV[0]);
		#pragma omp critical (OMPPushBackNewOutlier)
		mTempVertices.push_back(edgeV[1]);
	}

	return !mTempVertices.empty();
}

void FSSFRefiner::addToOutliers(const vector<uint32> &newOutliers)
{
	const uint32 newOutlierCount = (uint32) newOutliers.size();
	for (uint32 localIdx = 0; localIdx < newOutlierCount; ++localIdx)
	{
		// mark as outlier
		const uint32 vertexIdx = newOutliers[localIdx];
		uint8 &state = mVertexStates[vertexIdx];
		state |= OUTLIER;
		state &= ~BORDER_INLIER;

		// mark neighbor inliers as border inliers
		const uint32 start = mVertexNeighborsOffsets[vertexIdx];
		const uint32 end = mVertexNeighborsOffsets[vertexIdx + 1];
		for (uint32 localNeighborIdx = start; localNeighborIdx < end; ++localNeighborIdx)
		{
			const uint32 vertexNeighborIdx = mVertexNeighbors[localNeighborIdx];
			uint8 &neighborState = mVertexStates[vertexNeighborIdx];
			if (OUTLIER & neighborState)
				continue;
			neighborState |= BORDER_INLIER;
		}
	}
}

bool FSSFRefiner::hasNeighbor(const uint8 flag, const uint32 vertexIdx) const
{
	// any direct neighbor of type type?
	const uint32 startNeighborIdx = mVertexNeighborsOffsets[vertexIdx];
	const uint32 endNeighborIdx = mVertexNeighborsOffsets[vertexIdx + 1];

	for (uint32 localNeighborIdx = startNeighborIdx; localNeighborIdx < endNeighborIdx; ++localNeighborIdx)
	{
		const uint32 globalNeighborIdx = mVertexNeighbors[localNeighborIdx];
		if (flag & mVertexStates[globalNeighborIdx])
			return true;
	}

	return false;
}

uint32 FSSFRefiner::getVertexWithMostNeighbors(const uint32 triangle[3], const uint8 flag) const
{
	uint32 counts[3];
	for (uint32 cornerIdx = 0; cornerIdx < 3; ++cornerIdx)
		counts[cornerIdx] = getNeighborCount(flag, triangle[cornerIdx]);

	uint32 max = counts[0];
	uint32 maxCorner = 0;
	for (uint32 cornerIdx = 1; cornerIdx < 3; ++cornerIdx)
	{
		if (counts[cornerIdx] <= max)
			continue;
		max = counts[cornerIdx];
		maxCorner = cornerIdx;
	}

	return maxCorner;
}

uint32 FSSFRefiner::getNeighborCount(const uint8 flag, const uint32 vertexIdx) const
{
	// any direct neighbor of type type?
	const uint32 startNeighborIdx = mVertexNeighborsOffsets[vertexIdx];
	const uint32 endNeighborIdx = mVertexNeighborsOffsets[vertexIdx + 1];
	uint32 count = 0;

	for (uint32 localNeighborIdx = startNeighborIdx; localNeighborIdx < endNeighborIdx; ++localNeighborIdx)
	{
		const uint32 globalNeighborIdx = mVertexNeighbors[localNeighborIdx];
		if (flag & mVertexStates[globalNeighborIdx])
			++count;
	}

	return count;
}

void FSSFRefiner::fillHoles(const vector<vector<uint32>> &holeBorders)
{
	// replace found islands with simple geometry
	const uint32 firstHoleFillingVertex = mMesh.getVertexCount();
	mMesh.fillHoles(holeBorders);	
	mMesh.checkEdges();
	const uint32 newVertexCount = mMesh.getVertexCount();
	
	// grey hole filling vertices
	const Vector3 color(0.5f, 0.5f, 0.5f);
	#pragma omp parallel for
	for (int64 i = firstHoleFillingVertex; i < newVertexCount; ++i)
		mMesh.setColor(color, (uint32) i);

	// smoothing movement field for new geometry
	for (uint32 iteration =  0; iteration < 1; ++iteration)
	{
		#pragma omp parallel for
		for (int64 i = firstHoleFillingVertex; i < newVertexCount; ++i)
			mVectorField[i] = mMesh.computeUmbrellaSmoothingMovement((uint32) i, mParams.mUmbrellaSmoothingLambdaHigh);

		#pragma omp parallel for
		for (int64 i = firstHoleFillingVertex; i < newVertexCount; ++i)
			mMesh.getPosition((uint32) i) += mVectorField[i];
	}
}

void FSSFRefiner::simplifyMesh(const uint32 iteration)
{
	cout << "Simplifying by collapsing too small triangle edges or altitudes." << endl;
	mEdgeMergeCandidates.clear();
	mLeftEdgeMergeCandidates.clear();
	mLeftTriangleMergeCandidates.clear();
	
	while (true)
	{
		const uint32 oldVertexCount = mMesh.getVertexCount();

		// are there edges which should be removed / merged to a single vertex?
		findMergingEdges();
		if (mEdgeMergeCandidates.empty())
			break;

		// merge them
		mMesh.mergeEdges(mLeftEdgeMergeCandidates, mEdgeMergeCandidates);
		if (mLeftEdgeMergeCandidates.empty() || mMesh.getVertexCount() == oldVertexCount) // wasn't possible to merge the edges?
			break;
	}

	//updateObservers(iteration, "FSSFSimplified", IReconstructorObserver::RECONSTRUCTION_VIA_SAMPLES);
	mFirstHoleFillingTriangle = mMesh.getTriangleCount();
}

void FSSFRefiner::findMergingEdges()
{
	// get tree & mesh data
	const Vector3 *positions = mMesh.getPositions();
	uint32 triangleCount;
	uint32 *restrictedTriangleSet = getEdgeMergeTriangleSearchSet(triangleCount);

	// for each triangle: find intersecting nodes & mark the triangle if it's too large w.r.t. its intersection tree nodes
	mEdgeMergeCandidates.clear();

	#pragma omp parallel for
	for (int64 i = 0; i < triangleCount; ++i)
	{
		// get triangle vertices & side lengths
		const uint32 triangleIdx = (restrictedTriangleSet ? restrictedTriangleSet[i] : (uint32) i);
		const uint32 *tri = mMesh.getTriangle(triangleIdx);
		const Vector3 corners[3] = { positions[tri[0]], positions[tri[1]], positions[tri[2]] };
		const Real sideLengths2[3] =
		{ 
			(corners[0] - corners[1]).getLengthSquared(),
			(corners[1] - corners[2]).getLengthSquared(),
			(corners[2] - corners[0]).getLengthSquared()
		};

		// find minimum node length { triangle intersection nodes }, find shortest and longest triangle side
		const Real minNodeLength = getMinIntersectionTreeNodeLength(corners);
		Real minTriLength2 = sideLengths2[0];
		Real maxTriLength2 = sideLengths2[0];
		uint32 shortestSide = 0;
		uint32 longestSide = 0;
		for (uint32 sideIdx = 1; sideIdx < 3; ++sideIdx)
		{
			if (sideLengths2[sideIdx] < minTriLength2)
			{
				minTriLength2 = sideLengths2[sideIdx];
				shortestSide = sideIdx;
			}
			if (sideLengths2[sideIdx] > maxTriLength2)
			{
				maxTriLength2 = sideLengths2[sideIdx];
				longestSide = sideIdx;
			}
		}

		// compute length of smallest altitude and check it against shortest triangle side
		const Real altitudes[3] =
		{
			getDistanceToLine(corners[0], corners[1], corners[2]),
			getDistanceToLine(corners[1], corners[2], corners[0]),
			getDistanceToLine(corners[2], corners[0], corners[1])
		};
		const Real smallestAltitude = minimum(altitudes[0], minimum(altitudes[1], altitudes[2]));
		const Real smallestAltitudeLength2 = smallestAltitude * smallestAltitude;
		if (smallestAltitudeLength2 < minTriLength2)
			minTriLength2 = smallestAltitudeLength2;

		// smallest triangle part is large enough?
		const Real minThreshold = mParams.mEdgeMergeRelativeThreshold * minNodeLength; 
		if (minTriLength2 >= minThreshold * minThreshold)
			continue;
		
		// too small
		const uint32 edgeIdx = mMesh.getEdgeIndex(tri[shortestSide], tri[(shortestSide + 1) % 3]);
		#pragma omp critical (OMPPushBackMergingEdge)
			mEdgeMergeCandidates.push_back(edgeIdx);
	}

	Utilities::removeDuplicates(mEdgeMergeCandidates);
}

uint32 *FSSFRefiner::getEdgeMergeTriangleSearchSet(uint32 &triangleCount)
{
	// search within complete triangle set?
	const uint32 leftEdgeCandidateCount = (uint32) mLeftEdgeMergeCandidates.size();
	if (leftEdgeCandidateCount == 0)
	{
		triangleCount = mMesh.getTriangleCount();
		return NULL;
	}
	
	// is there a restricted search set of possible edge merge candidates from a previous merge iteration?
	mLeftTriangleMergeCandidates.clear();
	mLeftTriangleMergeCandidates.resize(2 * leftEdgeCandidateCount);

	#pragma omp parallel for
	for (int64 edgeIdx = 0; edgeIdx < leftEdgeCandidateCount; ++edgeIdx)
	{
		const uint32 globalEdgeIdx = mLeftEdgeMergeCandidates[edgeIdx];
		const Edge &edge = mMesh.getEdge(globalEdgeIdx);
		const uint32 *triangles = edge.getTriangleIndices();
		mLeftTriangleMergeCandidates[2 * edgeIdx + 0] = triangles[0];
		mLeftTriangleMergeCandidates[2 * edgeIdx + 1] = triangles[1];
	}
		
	mLeftEdgeMergeCandidates.clear();
	Utilities::removeDuplicates(mLeftTriangleMergeCandidates);
	triangleCount = (uint32) mLeftTriangleMergeCandidates.size();
	return mLeftTriangleMergeCandidates.data();
}

bool FSSFRefiner::computeErrorStatistics(const uint32 iteration)
{
	// mesh data
	const Edge *edges = mMesh.getEdges();
	const vector<uint32> *verticesToEdges = mMesh.getVerticesToEdges();
	const uint32 vertexCount = mMesh.getVertexCount();
	
	// average scales of vertex neighborhoods
	vector<Real> temp[2];
	temp[0].resize(vertexCount);
	temp[1].resize(vertexCount);
	mMesh.computeScalesViaEdgeDistances();
	memcpy(temp[0].data(), mMesh.getScales(), sizeof(Real) * vertexCount);

	uint32 sourceIdx = 0;
	for (uint32 smoothingIt = 0; smoothingIt < 3; ++smoothingIt, sourceIdx = !sourceIdx)
	{
		#pragma omp parallel for
		for (int64 i = 0; i < vertexCount; ++i)
		{
			const uint32 vertexIdx = (uint32) i;
			const vector<uint32> &localEdges = verticesToEdges[vertexIdx];
			const uint32 edgeCount = (uint32) localEdges.size();
			Real &avgScale = temp[!sourceIdx][vertexIdx];

			avgScale = temp[sourceIdx][vertexIdx];
			for (uint32 edgeIdx = 0; edgeIdx < edgeCount; ++edgeIdx)
			{
				const uint32 neighborIdx = edges[localEdges[edgeIdx]].getOtherVertex(vertexIdx);
				avgScale += temp[sourceIdx][neighborIdx];
			}

			avgScale /= (edgeCount + 1);
		}
	}

	// errors relative to vertex scales / mesh resolution
	mRelativeSurfaceErrors.resize(vertexCount);
	#pragma omp parallel for
	for (int64 i = 0; i < vertexCount; ++i)
	{
		const uint32 vertexIdx = (uint32) i;
		mRelativeSurfaceErrors[vertexIdx] = mSurfaceErrors[vertexIdx] / temp[sourceIdx][vertexIdx];
	}

	// error statistics
	const uint32 arrayCount = 1;
	const Real *errors[arrayCount] = { mSurfaceErrors.data() };
	const Real *weights[arrayCount] = { mWeightField.data() };
	const uint32 sizes[arrayCount] = { vertexCount };
	mStatistics.processIteration(errors, weights, sizes, arrayCount, mParams.mSupportWeakThreshold);

	// save statistics
	const Path &folder = Scene::getSingleton().getResultsFolder();
	const Path fileName = Path::appendChild(folder, "ErrorStatistics.txt");
	mStatistics.saveToFile(fileName);
	
	outputErrorColoredMesh(temp[0], mSurfaceErrors.data(), iteration, "absoluteErrors");
	outputErrorColoredMesh(temp[0], mRelativeSurfaceErrors.data(), iteration, "RelativeErrors");

	return mStatistics.hasConverged();
}

void FSSFRefiner::subdivideMesh()
{
	cout << "Subdividing mesh,\n";
	cout << "Vertices: " << mMesh.getVertexCount() << ", edges: " << mMesh.getEdgeCount() << ", triangles: " << mMesh.getTriangleCount() << endl;

	// are there triangles for subdivision?
	do
	{
		findSubdivisionEdges();
		mMesh.subdivideEdges(mSubdivisionEdges);
	} while (!mSubdivisionEdges.empty());
	
	// debug output
	const uint32 newVertexCount = mMesh.getVertexCount();
	const uint32 newEdgeCount = mMesh.getEdgeCount();
	const uint32 newTriangleCount = mMesh.getTriangleCount();

	cout << "Finished subdividing mesh via surface errors,\n";
	cout << "Vertices: " << newVertexCount << ", edges: " << newEdgeCount << ", triangles: " << newTriangleCount << endl;
	mFirstHoleFillingTriangle = newTriangleCount;
}

void FSSFRefiner::findSubdivisionEdges()
{
	cout << "Detecting subdivision edges.\n";

	mSubdivisionEdges.clear();
	findSubdivisionEdgesViaNodes();
	//findSubdivisionEdgesViaErrors();
	removeDuplicates(mSubdivisionEdges);
}

void FSSFRefiner::findSubdivisionEdgesViaNodes()
{
	// get tree & mesh data
	const uint32 triangleCount = mMesh.getTriangleCount();
	const Vector3 *positions = mMesh.getPositions();

	// for each triangle: find intersecting nodes & mark the triangle if it's too large w.r.t. its intersection tree nodes
	#pragma omp parallel for
	for (int64 triangleIdx = 0; triangleIdx < triangleCount; ++triangleIdx)
	{
		const uint32 *t = mMesh.getTriangle((uint32) triangleIdx);
		if (isNotForSubdivision(t, (uint32) triangleIdx))
			continue;

		// get triangle vertices & side lengths
		const Vector3 corners[3] = { positions[t[0]], positions[t[1]], positions[t[2]] };
		const Real sideLengths2[3] =
		{	
			(corners[0] - corners[1]).getLengthSquared(),
			(corners[1] - corners[2]).getLengthSquared(),
			(corners[2] - corners[0]).getLengthSquared()
		};

		// find longest side
		uint32 longestSide = 0;
		Real maxTriLength2 = sideLengths2[0];
		for (uint32 sideIdx = 1; sideIdx < 3; ++sideIdx)
		{
			if (sideLengths2[sideIdx] <= maxTriLength2)
				continue;

			longestSide = sideIdx;
			maxTriLength2 = sideLengths2[sideIdx];
		}
		
		// minimum node length { triangle intersection nodes }
		const Real minNodeLength = getMinIntersectionTreeNodeLength(corners);

		// longest triangle side > smallest side length of triangle intersection nodes?
		if (maxTriLength2 <= minNodeLength * minNodeLength)
			continue;

		const uint32 edgeIdx = mMesh.getEdgeIndex(t[longestSide], t[(longestSide + 1) % 3]);
		#pragma omp critical (OMPPushBackSubdivisionEdge)
			mSubdivisionEdges.push_back(edgeIdx);
	}
}

bool FSSFRefiner::isNotForSubdivision(const uint32 *triangle, const uint32 triangleIdx) const
{
	// not a hole filling triangle of the last iteration?
	if (triangleIdx >= mFirstHoleFillingTriangle)
			return true;

	// not supported by samples?
	if (isBad(triangle[0]) || isBad(triangle[1]) ||	isBad(triangle[2]))
		return true;

	// spiky geometry
	if (isSpiky(triangle[0]) || isSpiky(triangle[1]) || isSpiky(triangle[2]))
		return true;

	return false;
}

Real FSSFRefiner::getMinIntersectionTreeNodeLength(const Vector3 triangle[3]) const
{
	const Tree &tree = *Scene::getSingleton().getTree();
	const TriangleNodesChecker checker(triangle[0], triangle[1], triangle[2], (uint32) -1);
	Real minNodeLength = REAL_MAX;

	for (LeavesIterator it(tree, &checker); !it.isAtTheEnd(); ++it)
	{
		const Scope &scope = it.getScope();
		const Real size = scope.getSize();
		if (size < minNodeLength)
			minNodeLength = size;
	}

	return minNodeLength;
}

//void FSSFRefiner::findSubdivisionEdgesViaErrors()
//{
//
//	// get mesh data
//	const Edge *edges = mMesh.getEdges();
//	const Vector3 *positions = mMesh.getPositions();
//	const int64 edgeCount = mMesh.getEdgeCount();
//	
//	// find triangles to be subdivided to reduce the surface error at their edges via mEdgeVectorField and mEdgeWeights;
//	// add the 2 adjacent triangles for each edge with an error which is relatively high compared to the errors at the 2 adjacent vertices
//	#pragma omp parallel for
//	for (int64 i = 0; i < edgeCount; ++i)
//	{
//		const uint32 edgeIdx = (uint32) i;
//
//		// reliable edge?
//		const Edge &edge = edges[edgeIdx];
//		const Real weight = mEdgeWeights[edgeIdx];
//		if (weight <= mParams.mSupportWeakThreshold || isNaN(weight))
//			continue;
//		
//		// reliable vertices?
//		const uint32 v0 = edge.getVertexIndices()[0];
//		const uint32 v1 = edge.getVertexIndices()[1];
//		if (isBad(v0) || isBad(v1))
//			continue;
//
//		// scale-based subdivision limit already reached?
//		const Real edgeLength = (positions[v0] - positions[v1]).getLength();
//		const Real subdivisionEdgeLength = 0.5f * edgeLength;
//		const Real &edgeScale = mEdgeScales[edgeIdx];
//		const Real subdivisionMinimum = mSubdivisionRelativeScaleMinimum * edgeScale;
//		if (subdivisionEdgeLength <= subdivisionMinimum)
//			continue;
//
//		// get edge center surface movement
//		const Vector3 &correction = mEdgeVectorField[edgeIdx];
//		const Real normalizedAbsError = correction.getLength();
//		if (normalizedAbsError < EPSILON)
//			continue;
//
//		// get surface errors at edge vertices
//		const Real vertexError0 = mVectorField[v0].getLength();
//		const Real vertexError1 = mVectorField[v1].getLength();
//
//		// is the error high enough to subdivide the triangle?
//		if (normalizedAbsError < mSubdivisionRelativeErrorThreshold * vertexError0 &&
//			normalizedAbsError < mSubdivisionRelativeErrorThreshold * vertexError1)
//				continue;
//		
//		// mark the neighbor triangles for subdivison
//		#pragma omp critical (OMPPushBackSubdivisionEdge)
//			mSubdivisionEdges.push_back(edgeIdx);
//	}
//}

void FSSFRefiner::onReserveMemory(const uint32 vertexCapacity, const uint32 edgeCapacity, const uint32 indexCapacity)
{
	MeshRefiner::onReserveMemory(vertexCapacity, edgeCapacity, indexCapacity);	
	reserve(vertexCapacity, edgeCapacity, indexCapacity);
}

bool FSSFRefiner::smoothUntilConvergence(const uint8 requiredFlags)
{
	// find islands of vertices with requiredFlags & get counts
	IslesEraser isleManager(mMesh, mVertexStates.data(), requiredFlags);
	const atomic<uint32> *verticesToIsles = isleManager.getVerticesToIsles();
	const map<uint32, uint32> &sizes = isleManager.getIsleSizes();
	const uint32 threadCount = omp_get_max_threads();

	// gather isle IDs
	vector<uint32> isleIDs;
	isleIDs.reserve(sizes.size());
	for (map<uint32, uint32>::const_iterator it = sizes.begin(); it != sizes.end(); ++it)
		if (IslesEraser::INVALID_TRIANGLE_ISLAND != it->first)
			isleIDs.push_back(it->first);
	
	// smooth each isle
	vector<uint32> *isles = new vector<uint32>[threadCount];
	const uint32 isleCount = (uint32) isleIDs.size();
	const uint32 vertexCount = mMesh.getVertexCount();

	bool moved = false;
	#pragma omp parallel for
	for (int64 isleIdx = 0; isleIdx < isleCount; ++isleIdx)
	{
		// create isle
		vector<uint32> &isle = isles[omp_get_thread_num()];
		const uint32 isleID = isleIDs[isleIdx];
		const uint32 size = sizes.find(isleID)->second;

		isle.clear();
		isle.reserve(size);

		// fill isle
		for (uint32 vertexIdx = 0; vertexIdx < vertexCount; ++vertexIdx)
			if (isleID == verticesToIsles[vertexIdx])
				isle.push_back(vertexIdx);

		if (smoothIsleUntilConvergence(isle))
			moved = true;
	}

	delete [] isles;
	isles = NULL;
	return moved;
}

bool FSSFRefiner::smoothIsleUntilConvergence(const vector<uint32> &isle)
{
	const uint32 isleSize = (uint32) isle.size();
	bool moved = false;

	// smoothing movement field for vertices with requiredFlags
	for (uint32 localIdx = 0, convergedCount = 0; convergedCount < isleSize; ++localIdx)
	{
		if (localIdx >= isleSize)
			localIdx = 0;

		// smooth vertex until convergence
		// old position & target position 
		const uint32 vertexIdx = isle[localIdx];
		const Vector3 oldPos = mMesh.getPosition(vertexIdx);

		// target = center of neighbors
		//const Vector3 target = mMesh.getCenterOfNeighbors(vertexIdx);

		// target = weighted center of neighbors
		const Edge *edges = mMesh.getEdges();
		const vector<uint32> &edgeIndices = mMesh.getVerticesToEdges()[vertexIdx];
		const uint32 edgeCount = (uint32) edgeIndices.size();

		Real weight = (REAL_MAX == mSurfaceErrors[vertexIdx] ? EPSILON : 1.0f / (EPSILON + mSurfaceErrors[vertexIdx]));
		Vector3 target = oldPos * weight;
		Real sumOfWeights = weight;

		for (uint32 localEdgeIdx = 0; localEdgeIdx < edgeCount; ++localEdgeIdx)
		{
			const uint32 neighborVertexIdx = edges[edgeIndices[localEdgeIdx]].getOtherVertex(vertexIdx);
			weight = (REAL_MAX == mSurfaceErrors[vertexIdx] ? EPSILON : 1.0f / (EPSILON + mSurfaceErrors[vertexIdx]));
			target += mMesh.getPosition(neighborVertexIdx) * weight;
			sumOfWeights += weight;
		}
		target /= sumOfWeights;

		// update vertex position & convergence state
		const Vector3 movement = target - oldPos;
		if (movement.getLengthSquared() > EPSILON * EPSILON)
		{
			moved = true;
			convergedCount = 0;
			mMesh.setPosition(oldPos + movement, vertexIdx);
			continue;
		}
		
		++convergedCount;
	}

	return moved;
}

void FSSFRefiner::deleteHallucinatedGeometry()
{
	cout << "Deleting hallucinated geometry." << endl;
	mMesh.deleteUnsupportedGeometry(*this);
	//mMesh.deleteIsolatedGeometry(Scene::getSingleton().getMinIsleSize());
	cout << "Deleted hallucinated geometry." << endl;
	updateObservers(0, "FSSFTrimmed", IReconstructorObserver::RECONSTRUCTION_VIA_SAMPLES);
}

void FSSFRefiner::onFilterData(
	const uint32 *vertexOffsets, const uint32 vertexOffsetCount,
	const uint32 *edgeOffsets, const uint32 edgeOffsetCount,
	const uint32 *triangleOffsets, const uint32 triangleOffsetCount)
{
	MeshRefiner::onFilterData(vertexOffsets, vertexOffsetCount, edgeOffsets, edgeOffsetCount, triangleOffsets, triangleOffsetCount);
	
	FlexibleMesh::filterData<Vector3>(mBestPositions, vertexOffsets);
	FlexibleMesh::filterData<Real>(mBestSurfaceErrors, vertexOffsets);
	FlexibleMesh::filterData<Real>(mSurfaceErrors, vertexOffsets);
	FlexibleMesh::filterData<uint8>(mVertexStates, vertexOffsets);

	//FlexibleMesh::filterData<Vector3>(mEdgeVectorField, edgeOffsets);
	//FlexibleMesh::filterData<Real>(mEdgeScales, edgeOffsets);
	//FlexibleMesh::filterData<Real>(mEdgeWeights, edgeOffsets);
}

void FSSFRefiner::onEdgeMerging(const uint32 targetVertex, const uint32 edgeVertex0, const uint32 edgeVertex1)
{
	// inform base
	MeshRefiner::onEdgeMerging(targetVertex, edgeVertex0, edgeVertex1);

	// update vertexData
	createNewVertex(targetVertex, edgeVertex0, edgeVertex1, 0.5f);

	//// update edge data
	//for (uint32 i = 0; i < 2; ++i)
	//{
	//	const uint32 e0 = removedEdges0[i];
	//	const uint32 e1 = removedEdges1[i];
	//	const uint32 tE = targetEdges[i];

	//	mEdgeVectorField[tE] = (mEdgeVectorField[e0] + mEdgeVectorField[e1]) * 0.5f;
	//	mEdgeScales[tE] = (mEdgeScales[e0] + mEdgeScales[e1]) * 0.5f;
	//	mEdgeWeights[tE] = (mEdgeWeights[e0] + mEdgeWeights[e1]) * 0.5f;
	//}
}

void FSSFRefiner::onEdgeSplitVertex(const uint32 newVertexIdx, const uint32 edgeVertex0, const uint32 edgeVertex1)
{
	// inform base
	MeshRefiner::onEdgeSplitVertex(newVertexIdx, edgeVertex0, edgeVertex1);
	
	// update vertexData
	resize(mMesh.getVertexCount(), mMesh.getEdgeCount(), mMesh.getTriangleCount());
	createNewVertex(newVertexIdx, edgeVertex0, edgeVertex1, 0.5f);
}

void FSSFRefiner::createNewVertex(const uint32 targetVertex, const uint32 vertexIdx0, const uint32 vertexIdx1, const Real f0)
{
	const Real f1 = 1.0f - f0;

	// update vertex data
	mBestPositions[targetVertex] = mBestPositions[vertexIdx0] * f0 + mBestPositions[vertexIdx1] * f1;
	mBestSurfaceErrors[targetVertex] = REAL_MAX;
	mSurfaceErrors[targetVertex] = REAL_MAX;

	const uint8 orState = (mVertexStates[vertexIdx0] | mVertexStates[vertexIdx1]);
	mVertexStates[targetVertex] = ((UNSUPPORTED | SPIKY | OUTLIER) & orState);
}

void FSSFRefiner::onNewElements(
	const uint32 firstNewVertex, const uint32 newVertexCount,
	const uint32 firstNewEdge, const uint32 newEdgeCount,
	const uint32 firstNewTriangle, const uint32 newTriangleCount)
{
	MeshRefiner::onNewElements(firstNewVertex, newVertexCount, firstNewEdge, newEdgeCount, firstNewTriangle, newTriangleCount);
	resize(newVertexCount, newEdgeCount, newTriangleCount);
}

void FSSFRefiner::onNewStartMesh()
{
	MeshRefiner::onNewStartMesh();

	// get mesh data sizes
	const uint32 vertexCount = mMesh.getVertexCount();
	const uint32 edgeCount = mMesh.getEdgeCount();
	const uint32 indexCount = mMesh.getIndexCount();

	// resize & reserve memory for subdivisions
	resize(vertexCount, edgeCount, indexCount);
	mTempVertices.reserve(mMesh.getVertexCount());
	mMesh.reserve(vertexCount * MEMORY_ALLOCATION_FACTOR, edgeCount * MEMORY_ALLOCATION_FACTOR, indexCount * MEMORY_ALLOCATION_FACTOR);

	// initialize best positions
	const Vector3 *positions = mMesh.getPositions();
	#pragma omp parallel for
	for (int64 i = 0; i < vertexCount; ++i)
		mBestPositions[i] = positions[i];
}

void FSSFRefiner::reserve(const uint32 vertexCapacity, const uint32 edgeCapacity, const uint32 triangleCapacity)
{
	// vertices
	mBestPositions.reserve(vertexCapacity);
	mBestSurfaceErrors.reserve(vertexCapacity);
	mSurfaceErrors.reserve(vertexCapacity);
	mVertexStates.reserve(vertexCapacity);

	//// edges
	//mEdgeVectorField.reserve(edgeCapacity);
	//mEdgeScales.reserve(edgeCapacity);
	//mEdgeWeights.reserve(edgeCapacity);

	// triangles
	mTriangleNormals.reserve(triangleCapacity);
}

void FSSFRefiner::resize(const uint32 vertexCount, const uint32 edgeCount, const uint32 triangleCount)
{
	mAverageAngles[0].resize(vertexCount, 0.0f);
	mAverageAngles[1].resize(vertexCount, 0.0f);

	mBestPositions.resize(vertexCount);
	mBestSurfaceErrors.resize(vertexCount, REAL_MAX);
	mSurfaceErrors.resize(vertexCount, REAL_MAX);
	mVertexStates.resize(vertexCount, 0);
	
	//mEdgeVectorField.resize(count);
	//mEdgeScales.resize(count);
	//mEdgeWeights.resize(count);

	mTriangleNormals.resize(triangleCount);
}

void FSSFRefiner::deleteOutlierSamples()
{
	cout << "Finding and deleting outlier samples." << endl;

	// delete outliers detected via sample down weighting
	const Samples &samples = Scene::getSingleton().getSamples();
	const uint32 oldSampleCount = samples.getCount();

	// find inliers
	bool *inliers = new bool[oldSampleCount];
	findInlierSamples(inliers);

	// delete outliers
	Scene::getSingleton().eraseSamples(inliers, true);

	// free resources
	delete [] inliers;
	inliers = NULL;

	cout << "Finished deletion of outlier samples." << endl;
}

void FSSFRefiner::findInlierSamples(bool *inliers)
{
	cout << "Finding left outlier samples using estimated surface." << endl;
	
	// get scene data
	const Scene &scene = Scene::getSingleton();
	const Samples &samples = scene.getSamples();
	const uint32 sampleCount = samples.getCount();
	const uint32 pairCount = samples.getMaxViewConeCount();

	// create scene & set initial states
	mRayTracer.createStaticScene(mMesh.getPositions(), mMesh.getVertexCount(), mMesh.getIndices(), mMesh.getIndexCount(), true);
	memset(inliers, false, sizeof(bool) * sampleCount);

	// ray trace scene to find inliers
	for (uint32 startPairIdx = 0; startPairIdx < pairCount; startPairIdx += EMBREE_PAIR_BATCH_SIZE)
	{
		// find intersections
		uint32 batchSize = EMBREE_PAIR_BATCH_SIZE;
		if (batchSize + startPairIdx > pairCount)
			batchSize = pairCount - startPairIdx;
		mRayTracer.findIntersectionsForViewSamplePairs(true, startPairIdx, startPairIdx + batchSize, 
			EMBREE_RAY_BATCH_SIZE, mParams.mRaysPerViewSamplePair);
	
		// process intersections
		#pragma omp parallel for schedule(dynamic, OMP_PAIR_BATCH_SIZE)
		for (int64 i = 0; i < batchSize; ++i)
		{
			// get view sample pair data
			const uint32 localPairIdx = (uint32) i;
			const uint32 globalPairIdx = startPairIdx + localPairIdx;
			const uint32 sampleIdx = samples.getSampleIdx(globalPairIdx);
			ProjectedSample projectedSample;

			// get & check confidence of projected sample
			getProjectedSample(projectedSample, localPairIdx, globalPairIdx);
			if (projectedSample.mConfidence > EPSILON)
				inliers[sampleIdx] = true;
		}
	}
}

FSSFRefiner::~FSSFRefiner()
{
	clear();

	// free resources
	// for dijkstra searches
	delete [] mDijkstras;
	mDijkstras = NULL;
	 
	// for sample projections
	delete [] mLocalConfidences;
	mLocalConfidences = NULL;

	//delete [] mLocalEdgeWeights;
	//mLocalEdgeWeights = NULL;
}

void FSSFRefiner::clear()
{
	// clear merging & subdivision data
	mEdgeMergeCandidates.clear();
	mLeftEdgeMergeCandidates.clear();
	mSubdivisionEdges.clear();

	//// clear edges' data
	//mEdgeVectorField.clear();
	//mEdgeScales.clear();
	//mEdgeWeights.clear();

	// clear vertices' data
	mBestPositions.clear();
	mBestSurfaceErrors.clear();
	mSurfaceErrors.clear();
	mVertexStates.clear();

	// clear connectivity data
	mVertexNeighborsOffsets.clear();
	mVertexNeighbors.clear();

	// clear base object
	MeshRefiner::clear();
}

void FSSFRefiner::doSelfCheck()
{
	MeshRefiner::doSelfCheck();

	//#ifdef _DEBUG
	//	// check edge data
	//	const uint32 edgeCount = mMesh.getEdgeCount();
	//	for (uint32 edgeIdx = 0; edgeIdx < edgeCount; ++edgeIdx)
	//	{
	//		const Vector3 &c = mEdgeVectorField[edgeIdx];
	//		const Real &w = mEdgeWeights[edgeIdx];

	//		assert(!c.hasNaNComponent());
	//		assert(!isNaN(w));
	//	}
	//#endif // _DEBUG
}

void FSSFRefiner::loadFromFile(const std::string &meshFileName)
{
	clear();
	mMesh.loadFromFile(meshFileName);
	onNewStartMesh();
}

FSSFRefiner::FSSFRefiner(const FSSFRefiner &copy) :
	MeshRefiner(copy.getFlexibleMesh())
{
	assert(false);
}

void FSSFRefiner::outputErrorColoredMesh(vector<Real> &temp, const Real *errors, const uint32 iteration, const string &coreName) const
{
	cout << "Saving reconstruction with error-based colors." << endl;

	// create static mesh for saving
	const uint32 vertexCount = mMesh.getVertexCount();
	const uint32 indexCount = mMesh.getIndexCount();
	if (0 == indexCount || 0 == vertexCount)
		return;
	
	// compute & save scalar errors & find minimum & maximum
	temp.resize(mMesh.getVertexCount());
	#pragma omp parallel for
	for (int64 i = 0; i < vertexCount; ++i)
	{
		const uint32 vertexIdx = (uint32) i;

		if (!isBad(vertexIdx))
			temp[vertexIdx] = errors[vertexIdx];
		else
			temp[vertexIdx] = 0.0f;
	}

	// find nth element as maximum
	vector<Real>::iterator it = temp.begin() + (uint32) (temp.size() * 0.9f);
	std::nth_element(temp.begin(), it, temp.end());
	const Real maximum = *it;
	
	StaticMesh staticMesh(mMesh.getColors(), mMesh.getNormals(), mMesh.getPositions(), mMesh.getScales(), mMesh.getIndices(),
		vertexCount, indexCount);

	// compute error colors
	cout << "Computing error-based colors." << endl;
	const Real scaleFactor = 1.0f / maximum;
	for (uint32 vertexIdx = 0; vertexIdx < vertexCount; ++vertexIdx)
	{
		const Real error = errors[vertexIdx];
		const Real colorFactor = clamp(error * scaleFactor, 1.0f, 0.0f);

		Vector3 &color = staticMesh.getColor(vertexIdx);
		if (isBad(vertexIdx))
			color.set(1.0f, 0.5f, 0.5f);
		else
			color.set(colorFactor, 0.5f, 0.5f);
	}

	FSSFRefiner::saveMesh(staticMesh, iteration, coreName.data());
}

void FSSFRefiner::saveAngleColoredMesh(const uint32 iteration) const
{
	const uint32 vertexCount = mMesh.getVertexCount();
	FlexibleMesh copy(mMesh);

	#pragma omp parallel for
	for (int64 i = 0; i < vertexCount; ++i)
	{
		// already unreliable? -> skip
		const uint32 vertexIdx = (uint32) i;
		const Real average = clamp(mAverageAngles[0][vertexIdx], PI, 0.0f);

		Vector3 color;
		if (average > HALF_PI)
			color.x = (average - HALF_PI) / PI + 0.5f;
		else if (average > QUARTER_PI)
			color.y = (average - QUARTER_PI) / HALF_PI + 0.5f;
		else
			color.z = average / HALF_PI + 0.5f; 
		copy.setColor(color, vertexIdx);
		copy.setScale(average, vertexIdx);
	}

	FSSFRefiner::saveMesh(copy, iteration, "ColoredByAngles");
}

void FSSFRefiner::saveResult(const uint32 iteration)
{
	FlexibleMesh copy(mMesh);
	copy.deleteUnsupportedGeometry(*this);
	copy.computeNormalsWeightedByAngles();
	FSSFRefiner::saveMesh(copy, iteration, "Result");
}

void FSSFRefiner::saveOutlierColoredMesh(const uint32 iteration) const
{
	// mark OUTLIER, BORDER_INLIER and unreliable vertices
	FlexibleMesh copy(mMesh);
	Vector3 *colors = copy.getColors();
	for (uint32 i = 0; i < copy.getVertexCount(); ++i)
	{
		const uint8 state = mVertexStates[i];
		
		const Real red = (OUTLIER & state ? 1.0f : (BORDER_INLIER & state ? 0.5f : 0.0f));
		const Real green = (UNSUPPORTED & state ? 0.0f : (SPIKY & state ? 0.5f : 1.0f));
		const Real blue = 0.5f;
		colors[i].set(red, green, blue);
	}

	FSSFRefiner::saveMesh(copy, iteration, "OutliersAndUnreliableOnes");
}

void FSSFRefiner::saveMeshForDebugging(FlexibleMesh &mesh,
	const uint32 iteration, const vector<vector<uint32>> &holeBorders, const uint32 afterDeletionVertexCount)
{
	const uint32 holeCount = (uint32) holeBorders.size();
	const uint32 vertexCount = mesh.getVertexCount();

	// grey with red holes
	const Vector3 conflictEdgeColor(0.0f, 0.0f, 1.0f);
	const Vector3 conflictTriangleColor(1.0f, 1.0f, 1.0f);
	const Vector3 fillingColor(0.5f, 1.0f, 0.5f);
	const Vector3 borderColor(0.0f, 1.0f, 1.0f);
	const Vector3 defaultColor(0.5f, 0.5f, 0.5f);
	const Vector3 lastTriangleColor(1.0f, 0.0f, 0.0f);

	// initially default & filling vertices special color
	for (uint32 i = 0; i < vertexCount; ++i)
	{
		if (i >= afterDeletionVertexCount)
			mesh.setColor(fillingColor, i);
		else
			mesh.setColor(defaultColor, i);
	}

	// color hole borders
	for (uint32 holeIdx = 0; holeIdx < holeCount; ++holeIdx)
	{
		const vector<uint32> &hole = holeBorders[holeIdx];
		const uint32 holeSize = (uint32) hole.size();
		for (uint32 holeVertex = 0; holeVertex < holeSize; ++holeVertex)
		{
			const uint32 vertexIdx = hole[holeVertex];
			mesh.setColor(borderColor, vertexIdx);
		}
	}

	// color conflict elements
	const vector<EdgeConflict> &conflicts = mesh.getEdgeConflicts();
	const uint32 conflictCount = (uint32) conflicts.size();
	for (uint32 cIdx = 0; cIdx < conflictCount; ++cIdx)
	{
		const EdgeConflict &conflict = conflicts[cIdx];
		const uint32 edgeIdx = conflict.mEdgeIdx;
		const Edge &edge = mesh.getEdge(edgeIdx);
		const uint32 *v = edge.getVertexIndices();
		
		mesh.getColor(v[0]) = (mesh.getColor(v[0]) + conflictEdgeColor) * 0.5f;
		mesh.getColor(v[1]) = (mesh.getColor(v[1]) + conflictEdgeColor) * 0.5f;

		for (uint32 triIdx = 0; triIdx < conflict.mTriangles.size(); ++triIdx)
		{
			const uint32 globalIdx = conflict.mTriangles[triIdx];
			const uint32 *indices = mesh.getTriangle(globalIdx);
			
			for (uint32 j = 0; j < 3; ++j)
			{
				if (indices[j] == v[0] || indices[j] == v[1])
					continue;

				mesh.getColor(indices[j]) = (mesh.getColor(indices[j]) + conflictTriangleColor) * 0.5f;
			}
		}
	}

	const uint32 *lastTriangle = mesh.getTriangle(mesh.getTriangleCount() - 1);
	for (uint32 i = 0; i < 3; ++i)
		mesh.getColor(lastTriangle[i]) = (mesh.getColor(lastTriangle[i]) + lastTriangleColor) * 0.5f;

	FSSFRefiner::saveMesh(mesh, iteration,"ForDebugging");
}

void FSSFRefiner::saveMesh(const Mesh &mesh, const uint32 iteration, const char *text, const bool saveAsMesh)
{
	// file name
	const uint32 BUFFER_SIZE = 100;
	char buffer[BUFFER_SIZE];
	snprintf(buffer, BUFFER_SIZE, "%.4uFSSF", iteration);

	const Path &folder = Scene::getSingleton().getResultsFolder();
	const string temp(buffer);
	const Path fileName = Path::appendChild(folder, temp + text);

	// save it
	mesh.saveToFile(fileName, true, saveAsMesh);
}



	// instead of mMesh.fillHoles
	//// debugging stuff
	//saveMesh(mMesh, iteration, "AfterDel");
	//const uint32 firstHoleFillingVertex = mMesh.getVertexCount();
	//try
	//{
	//	mMesh.mHoleIdx = (uint32) -1;
	//	fillHoles(holeBorders);
	//}
	//catch(...)
	//{
	//	const vector<uint32> &hole = holeBorders[mMesh.mHoleIdx];
	//	cerr << "Broken hole." << endl;
	//	for (uint32 i = 0; i < hole.size(); ++i)
	//		cerr << hole[i] << " ";
	//	cerr << endl;

	//	FlexibleMesh copy(mMesh);
	//	saveMeshForDebugging(copy, 0, holeBorders, firstHoleFillingVertex);

	//	if ((uint32) -1 != mMesh.mHoleIdx)
	//	{
	//		const vector<uint32> &hole = holeBorders[mMesh.mHoleIdx];
	//		for (uint32 i = 0; i < hole.size(); ++i)
	//		{
	//			const uint32 v = hole[i];
	//			const Vector3 &p = copy.getPosition(v);
	//			cerr << "Prob vertex " << i << " " << p.x << " " << p.y << " " << p.z << "\n";
	//			copy.setColor(Vector3(1.0f, 0.2f, 0.3f), v);
	//		}
	//		cerr << flush;

	//		// add triangle for easy finding
	//		const uint32 vertexCount = copy.getVertexCount();
	//		copy.addVertex(Vector3(1.0f, 0.0f, 0.0f), Vector3(1.0f, 0.0f, 0.0f), Vector3(0.0f, 0.0f, 0.0f), 1.0f);
	//		copy.addVertex(Vector3(1.0f, 0.0f, 0.0f), Vector3(1.0f, 0.0f, 0.0f), copy.getPosition(hole[0]), 1.0f);
	//		copy.addVertex(Vector3(1.0f, 0.0f, 0.0f), Vector3(1.0f, 0.0f, 0.0f), copy.getPosition(hole[1]), 1.0f);
	//		copy.addTriangle(vertexCount, vertexCount + 1, vertexCount + 2);
	//		copy.addTriangle(vertexCount, vertexCount + 2, vertexCount + 1);

	//		//copy.setPosition(Vector3(0.0f, 0.0f, 1.0f), hole[0]);
	//		copy.saveToFile(Scene::getSingleton().getResultsFolder() + "ProblemHole", true, false);
	//	}

	//	throw uint32(-1);
	//}
