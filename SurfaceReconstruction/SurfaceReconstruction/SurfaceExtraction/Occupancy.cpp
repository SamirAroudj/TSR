/*
 * Copyright (C) 2017 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */

#include <cstring>
#include <fstream>
#include <omp.h>
#include "CollisionDetection/CollisionDetection.h"
#include "Math/MathCore.h"
#include "Math/Vector4.h"
#include "Platform/FailureHandling/FileCorruptionException.h"
#include "Platform/ParametersManager.h"
#include "Platform/Storage/File.h"
#include "SurfaceReconstruction/Scene/Scene.h"
#include "SurfaceReconstruction/Scene/Tree/Leaves.h"
#include "SurfaceReconstruction/Scene/Tree/LeavesIterator.h"
#include "SurfaceReconstruction/Scene/Tree/SphereNodesChecker.h"
#include "SurfaceReconstruction/Scene/Tree/Tree.h"
#include "SurfaceReconstruction/SurfaceExtraction/ViewConeNodesChecker.h"
#include "SurfaceReconstruction/Scene/Samples.h"
#include "SurfaceReconstruction/Scene/View.h"
#include "SurfaceReconstruction/SurfaceExtraction/DualMarchingCells.h"
#include "SurfaceReconstruction/SurfaceExtraction/LineChecker.h"
#include "SurfaceReconstruction/SurfaceExtraction/Occupancy.h"
#include "SurfaceReconstruction/SurfaceExtraction/SphereNodeStatesChecker.h"
#include "Utilities/HelperFunctions.h"

using namespace CollisionDetection;
using namespace FailureHandling;
using namespace Graphics;
using namespace Math;
using namespace Platform;
using namespace std;
using namespace Storage;
using namespace SurfaceReconstruction;
using namespace Utilities;

// Possible kernel functions for occupancy scalar field computation:
/** Enables switching between different functions to compute partial free space weights within view to sample cones
	whereas such a function is always defined as orthogonal to the central ray of the view cone it belongs to. */
//#define CONE_PCD_KERNEL_CONSTANT									// No tangential falloff (kernel = 1 / kernel area)
//#define CONE_PCD_KERNEL_POLY_6									// Use Muller's 2D Poly6 kernel
//#define CONE_PCD_KERNEL_CUBIC										// Cubic function
//#define CONE_PCD_KERNEL_POS_PARABLE								// Positive parable holding o(b)=0 & o'(b)=0.
//#define CONE_PCD_KERNEL_LINE										// Use a line holding o(b)=0.
#define CONE_PCD_KERNEL_NEG_PARABLE								// Use a negative parable holding o(b)=0 & o'(0)=0.
//#define CONE_PCD_KERNEL_EULER										// Use a negative exponential function with base e holding o(b)=0.
//#define CONE_PCD_KERNEL_4TH_ORDER									// Use a x^4-function holding o(-b)= 0 & o'(-b)=0 & o(b)=0 & o'(b)=0.


/** Enables switching between different functions to compute partial free space weights within vie to sample cones
	whereas such a function is always defined along central view cone ray direction. */
//#define CONE_RAY_KERNEL_LINE			// Use a line holding f(R)=0.
//#define CONE_RAY_KERNEL_POS_PARABLE		// Use a positive parable holding f(R)=0 & f'(R)=0.
#define CONE_RAY_KERNEL_NEG_PARABLE	// Use a negative parable holding f(R)=0 & f'(0)=0.
//#define CONE_RAY_KERNEL_EULER			// Use a negative exponential function with base e holding f(R)=0.
//#define CONE_RAY_KERNEL_CUBIC			// Use a cubic function holding f(R)=0, f'(R)=0 & f'(0)=0.

/** Enables switching between different functions to compute sampleness weights within sample spheres
	whereas such a function is always defined in the coordinate system of the sample. */
//#define SPHERICAL_KERNEL_CONSTANT	// k(x) = 1 / sphere volume for x \in support range
#define SPHERICAL_KERNEL_POLY_3	// Own 3D and degree 3 polynomial kernel
//#define SPHERICAL_KERNEL_POLY_6	// Use Mullers 3D Poly6 kernel
//#define SPHERICAL_KERNEL_SPIKY	// Use Mullers 3D spiky kernel

// constants
const uint32 Occupancy::FILE_VERSION = 0;
const uint32 Occupancy::MAX_DEPTH_DIFFERENCE = 3;

const uint32 Occupancy::OMP_PRIOR_LEAF_BATCH_SIZE = 0x1 << 9;
const uint32 Occupancy::OMP_SAMPLE_BATCH_SIZE = 0x1 << 7;
const uint32 Occupancy::OMP_VIEW_CONE_BATCH_SIZE = 0x1 << 7;

Occupancy::Occupancy(const Tree *tree) :
	Occupancy()
{
	cout << "Estimating space occupancy scalar field." << endl;
	if (!tree)
	{
		cout << "NOT! There is no scene tree to do that." << endl;
		return;
	}

	const uint32 leafCount = tree->getLeaves().getCount();	
	const uint32 nodeCount = tree->getNodes().getCount();
	
	// allocate memory
	// for cone lengths & kernel sums
	for (DataType type = EMPTINESS; type < DATA_TYPE_COUNT; type = (DataType) (type + 1))
	{
		mConeLengths[type] = new Real[leafCount];
		mKernelSums[type] = new Real[leafCount];
	}
	mPriorsForEmptiness = new Real[leafCount];

	// for node states
	mNodeStates = new uint32[nodeCount];
	computeOccupancy();
}

void Occupancy::computeOccupancy()
{
	// initialization
	const Tree &tree = *Scene::getSingleton().getTree();
	const uint32 leafCount = tree.getLeaves().getCount();
	const uint32 nodeCount = tree.getNodes().getCount();

	// init kernel sums & lengths & total count M (count M = sum of confidences)
	const size_t byteCount = leafCount * sizeof(Real);
	for (DataType type = EMPTINESS; type < DATA_TYPE_COUNT; type = (DataType) (type + 1))
	{
		memset(mConeLengths[type], 0, byteCount);
		memset(mKernelSums[type], 0, byteCount);
	}
	memset(mPriorsForEmptiness, 0, byteCount);
	mConeCount = 0.0f;
	
	// init nodeStates
	memset(mNodeStates, 0, sizeof(uint32) * nodeCount);

	cout << "Starting to compute global occupancy field via sampleness and emptiness PDFs." << endl;
	update(false, NULL, 0, 0);
}

void Occupancy::onSamplesChanged(const bool forRemoval, const uint32 *chosenSamples, const uint32 chosenSampleCount)
{
	cout << "Updating occupancy data to account for change of samples." << endl;
	
	update(forRemoval, chosenSamples, chosenSampleCount, 0);

	cout << "Finished updating free space due to change of samples. " << endl;
}

void Occupancy::updateKernels(const bool forRemoval, const DataType dataType, const uint32 *chosenSamples, const uint32 chosenSampleCount, const uint32 requiredFlags)
{
	cout << "Updating sums of kernel values due to " << (forRemoval ? "removed" : "added") << " samples." << endl;
	if (EMPTINESS == dataType)
		forwardAnyChildStateFlag(0, NODE_FLAG_SAMPLENESS);

	// get scene data
	const Scene &scene = Scene::getSingleton();
	const Samples &samples = scene.getSamples();
	const Tree &tree = *scene.getTree();

	// flag to be set for leaves which are influenced by the kernels
	NodeStateFlag leafFlag;
	if (EMPTINESS == dataType)
		leafFlag = NODE_FLAG_EMPTINESS;
	else if (SAMPLENESS == dataType)
		leafFlag = NODE_FLAG_SAMPLENESS;
	else
		throw Exception("Case is not implemented.");

	// init lengths & counts for all threads
	const uint32 threadCount = omp_get_max_threads();
	
	Real *counts = NULL;
	if (EMPTINESS == dataType)
	{	
		counts = new Real[threadCount];
		for (uint32 threadIdx = 0; threadIdx < threadCount; ++threadIdx)
			counts[threadIdx] = 0.0f;
	}

	// evaluate each view cone kernel for dataType PDF
	int64 viewConeCount = (chosenSamples ? samples.getViewsPerSample() * chosenSampleCount : samples.getMaxViewConeCount());
	#pragma omp parallel for schedule(dynamic, OMP_VIEW_CONE_BATCH_SIZE)
	for (int64 viewConeIdx = 0; viewConeIdx < viewConeCount; ++viewConeIdx)
	{
		Vector3 viewWS;
		uint32 sampleIdx;
		if (!getViewConeData(viewWS, sampleIdx, (uint32) viewConeIdx, chosenSamples))
			continue;

		// get sample data for cone creation
		const Vector3 normal = -samples.getNormalWS(sampleIdx);
		const Vector3 &sampleWS = samples.getPositionWS(sampleIdx);
		const Real kernelR = getBandwidthFactor() * samples.getScale(sampleIdx);
		const uint32 flags = (requiredFlags | (EMPTINESS == dataType ? NODE_FLAG_SAMPLENESS : 0));
		Real coneLength = REAL_MAX;

		Vector3 end = sampleWS;
		if (SAMPLENESS == dataType)
		{
			const Vector3 viewToSample = sampleWS - viewWS;
			const Real totalLength = viewToSample.getLength();
			const Vector3 viewDir = viewToSample / totalLength;

			coneLength = kernelR * mRelativeSampleConeLength;
			end += viewDir * (coneLength * 0.5f);
		}

		// create & check cone
		const ViewConeNodesChecker checker(viewWS, normal, end, kernelR, coneLength, getMaxDepth(sampleIdx), getNodeStates(), flags);
		if (checker.isDegenerated())
			continue;
		
		// for prior: update count M, weighted cone length
		const uint32 threadIdx = omp_get_thread_num();
		const Real confidence = samples.getConfidence(sampleIdx);
		const Real weightedConeLength = (forRemoval ? -1.0f : 1.0f) * confidence * checker.getLength();
		if (EMPTINESS == dataType)
		{
			if (forRemoval)
				counts[threadIdx] -= confidence;
			else
				counts[threadIdx] += confidence;
		}

		// update sums for all overlapping nodes
		for (LeavesIterator it(tree, &checker); !it.isAtTheEnd(); ++it)
		{
			// get scope data
			const Scope scope = it.getScope();
			const Vector3 leafCenter = scope.getCenterPosition();
			const uint32 leafIdx = it.getLeafIndex();

			// update values for the found leaf
			Real &targetSumOfKernels = mKernelSums[dataType][leafIdx];
			if (addKernel(targetSumOfKernels, leafCenter, checker, dataType, confidence, forRemoval))
			{
				// update sum of lengths
				Real &targetSumOfLengths = mConeLengths[dataType][leafIdx];
				#pragma omp atomic
					targetSumOfLengths += weightedConeLength;

				if (!forRemoval)
					mNodeStates[it.getNodeIndex()] |= leafFlag;
			}
		}
	}
	
	// update total count (sampleness cone count = emptiness cone count) 
	if (EMPTINESS == dataType)
		for (uint32 threadIdx = 0; threadIdx < threadCount; ++threadIdx)
			mConeCount += counts[threadIdx];
	
	// free resources
	delete [] counts;
	counts = NULL;

	//forwardAnyChildStateFlag(0, NODE_FLAG_EMPTINESS);
}

bool Occupancy::getViewConeData(Vector3 &viewPosWS, uint32 &sampleIdx,
	const uint32 viewConeIdx, const uint32 *chosenSamples)
{
	// get scene data
	const Scene &scene = Scene::getSingleton();
	const vector<View *> &views = scene.getViews();
	const Samples &samples = scene.getSamples();
	const uint32 viewsPerSample = samples.getViewsPerSample();

	// sample data
	uint32 viewIdx;
	if (chosenSamples)
	{
		sampleIdx = chosenSamples[viewConeIdx / viewsPerSample];
		viewIdx = samples.getViewIdx(viewConeIdx % viewsPerSample, sampleIdx);
	}
	else
	{
		sampleIdx = samples.getSampleIdx(viewConeIdx);
		viewIdx = samples.getViewIdx(viewConeIdx);
	}
	if (!Scene::getSingleton().isValidView(viewIdx))
		return false;
		
	// get sample confidence & view world space position
	viewPosWS = views[viewIdx]->getPositionWS();
	return true;
}

bool Occupancy::addKernel(Real &targetKernelSum,
	const Vector3 &evaluationPos, const ObliqueCircularCone &viewCone,
	const DataType dataType, const Real sampleConfidence, const bool negative) const
{
	// add kernel value to target evaluation position based on
	// 1. a function along view ray from view center to sample center
	// 2. a function within a projective circular sample disc (PCD) through leaf center

	// where does the plane <x - evaluationPos, sampleNormal> = 0 intersect the view line?
	const Real tCompleteCone = viewCone.getTCompleteCone(evaluationPos);
	const Real tCutCone = viewCone.getTCutCone(tCompleteCone);

	// is positionWS in front of or behind the view cone?
	if (tCutCone < 0.0f|| tCutCone >= 1.0f)
		return false;

	// => via z: get the projective circular disc (PCD) around the view ray which is parallel to the sample normal and goes through positionWS
	// (PCD = projection of the circular sample disc (CSD) (CSD: centered at sample, orthogonal to sample normal, size = maxEndConeRadius)
	const Vector3 &viewToEnd = viewCone.getApexToEnd();
	const Vector3 PCDCenter = viewCone.getApex() + viewToEnd * tCompleteCone;
	const Real PCDRadius = viewCone.getEndRadius() * tCompleteCone;

	// is positionWS within PCD and view cone?
	const Real radialDistanceSq = (evaluationPos - PCDCenter).getLengthSquared();
	const Real PCDRadiusSq = PCDRadius * PCDRadius;
	if (radialDistanceSq >= PCDRadiusSq)
		return false;

	// ray kernel falling off from cone start towards end center or
	// ray kernel centered at cut cone center / falling of from cut cone center towards cut cone end and towards cut cone start?
	Real Z;
	Real z;
	Real normFactor;
	if (EMPTINESS == dataType)
	{
		Z = viewCone.getLength();
		z = tCutCone * Z;
		normFactor = 1.0f;
	}
	else if (SAMPLENESS == dataType)
	{
		const Real tCentered = fabsr(tCutCone - 0.5f) * 2.0f;
		Z = 0.5f * viewCone.getLength();
		z = tCentered * Z;
		normFactor = 0.5f;
	}

	// 3D view cone kernel value consists of
	// kernel along ray & kernel within PCD (projective circular disc) 
	const Real rayKernelValue = computeRayKernel(z, Z) * normFactor;
	const Real PCDKernelValue = computeCircularDiscKernelFromSquared(radialDistanceSq, PCDRadius);
	const Real kernelValue = rayKernelValue * PCDKernelValue;
	if (EPSILON > kernelValue)
		return false;
	
	// final kernel value
	Real scaledKernel = sampleConfidence * kernelValue;
	if (EPSILON > scaledKernel)
		return false;

	if (negative)
		scaledKernel = -scaledKernel;

	// update global variables
	#pragma omp atomic
		targetKernelSum += scaledKernel;
	return true;
}

Real SurfaceReconstruction::Occupancy::computeCircularDiscKernelFromSquared(const Real r2, const Real R)
{
	#ifdef CONE_PCD_KERNEL_CONSTANT
		return 1.0f / (Math::PI * R * R);
	#endif

	#ifdef CONE_PCD_KERNEL_POLY_6
		return Math::getKernel2DPoly6FromSquared(r2, R);
	#endif

	#ifdef CONE_PCD_KERNEL_CUBIC
		const Real R2 = R * R;
		if (r2 >= R * R)
			return 0.0f;

		const Real r = sqrtr(r2);
		return Math::getKernel2DPoly3(r, R);
	#endif

	#ifdef CONE_PCD_KERNEL_POS_PARABLE
		const Real R2 = R * R;		
		if (r2 >= R * R)
			return 0.0f;

		const Real r = sqrtr(r2);
		const Real R3 = R * R2;
		const Real R4 = R2 * R2;
		const Real value = (6.0f / (PI * R4)) * r2 - (12.0f / (PI * R3)) * r + (6.0f / (PI * R2));

		return value;
	#endif

	#ifdef CONE_PCD_KERNEL_LINE
		const Real R2 = R * R;
		if (r2 >= R * R)
			return 0.0f;

		const Real r = sqrtr(r2);
		const Real R3 = R * R2;
		const Real value = -(3.0f / (PI * R3)) * r + (3.0f / (PI * R2));

		return value;
		#endif

	#ifdef CONE_PCD_KERNEL_NEG_PARABLE
		const Real R2 = R * R;
		if (r2 >= R2)
			return 0.0f;
		
		const Real notNormed = 1.0f - r2 / R2;
		const Real value = (2.0f / (PI * R2)) * notNormed;
		return value;
	#endif

	#ifdef CONE_PCD_KERNEL_EULER
		const Real R2 = R * R;
		if (r2 >= R2)
			return 0.0f;

		const Real r = sqrtr(r2);
		const Real eR = exp(R);
		const Real denominator = PI * (eR * (R2 - 2.0f * R + 2.0f) - 2.0f);
		const Real value = (-exp(r) + eR) / denominator;

		return value;
	#endif

	#ifdef CONE_PCD_KERNEL_4TH_ORDER
		const Real R2 = R * R;
		if (r2 >= R2)
			return 0.0f;

		const Real R4 = R2 * R2;
		const Real R6 = R2 * R4;
		const Real r4 = r2 * r2;
		const Real value = (3.0f / (PI * R6)) * r4 - (6.0f / (PI * R4)) * r2 + (3.0f / (PI * R2));

		return value;
	#endif
}

Real SurfaceReconstruction::Occupancy::computeRayKernel(const Real z, const Real Z) 
{
	#ifdef CONE_RAY_KERNEL_NEG_PARABLE
		if (z < 0.0f || z >= Z)
			return 0.0f;

		const Real z2 = z * z;
		const Real Z2 = Z * Z;
		const Real value = (3.0f / (2.0f * Z)) * (-z2 / Z2 + 1.0f);
		return value;
	#endif

	#ifdef CONE_RAY_KERNEL_POS_PARABLE
		const Real Z2 = Z * Z;
		const Real Z3 = Z * Z2;
		const Real z2 = z * z;

		return (3.0f / Z3) * z2 - (6.0f / Z2) * z + (3.0f / Z);
	#endif

	#ifdef CONE_RAY_KERNEL_LINE
		// compactly supported function
		if (z <= 0 || z >= Z)
			return 0.0f;

		const Real Z2 = Z * Z;
		const Real value = -(2.0f / Z2) * z + (2.0f / Z);
		return value;
	#endif

	#ifdef CONE_RAY_KERNEL_EULER
		const Real eZ = exp(Z);
		const Real denominator = eZ * (Z - 1.0f) + 1.0f;
		const Real temp = -exp(z) + eZ;
		const Real value = temp / denominator;

		return value;
	#endif

	#ifdef CONE_RAY_KERNEL_CUBIC
		const Real Z3 = Z * Z * Z;
		const Real Z4 = Z3 * Z;
		const Real z2 = z * z;
		const Real z3 = z2 * z;
		const Real value = (4.0f / Z4) * z3 - (6.0f / Z3) * z2 + (2.0f / Z);

		return value;
	#endif
}

bool Occupancy::isEmpty(const uint32 leafIdx) const
{
	// reliable classification by occupancy value?
	if (isUnreliable(leafIdx))
		return false;

	// occupancy <= 0?
	const Real pOfXAndCE = getEmptinessCPD(leafIdx) * getPriorForEmptiness(leafIdx);
	const Real pOfXAndCS = getSamplenessCPD(leafIdx) * getPriorForSampleness(leafIdx);
	return (pOfXAndCE > pOfXAndCS);
}

uint32 Occupancy::forwardAnyChildStateFlag(const uint32 nodeIdx, const NodeStateFlag flag)
{
	// get nodes
	const Tree &tree = *Scene::getSingleton().getTree();
	const Leaves &leaves = tree.getLeaves();
	const Nodes &nodes = tree.getNodes();
	uint32 &state = mNodeStates[nodeIdx];
	
	// leaf -> flag is directly defined by mKernelSums and some condition based on flag
	if (nodes.isLeaf(nodeIdx))
		return state;
	
	// an inner node gets the flag set if any of its children has the flag set 
	state &= ~flag;

	const uint32 firstChild = nodes.getChildBlock(nodeIdx);
	for (uint32 childOffset = 0; childOffset < Nodes::CHILD_COUNT; ++childOffset)
	{
		const uint32 childIdx = firstChild + childOffset;
		const uint32 childState = forwardAnyChildStateFlag(childIdx, flag);
		if (flag & childState)
			state |= flag;
	}
	
	return state;
}

uint32 Occupancy::getMaxDepth(const uint32 sampleIdx) const
{
	return MAX_DEPTH_DIFFERENCE + Scene::getSingleton().getTree()->getSampleDepth(sampleIdx);
}

const DualMarchingCells &Occupancy::extractCrust()
{
	delete mCrust;
	mCrust = new DualMarchingCells(*this, Scene::getSingleton().getMinIsleSize());
	return *mCrust;
}

void Occupancy::computePriors()
{
	// get leaf count
	const Tree &tree = *Scene::getSingleton().getTree();
	const Leaves &leaves = tree.getLeaves();
	const uint32 leafCount = leaves.getCount();

	cout << "Computing occupancy class priors.\n" << endl;

	// class priors for each leaf (scene-varying semi global Bayesian class priors)
	#pragma omp parallel for schedule(dynamic, OMP_PRIOR_LEAF_BATCH_SIZE)
	for (int64 i = 0; i < leafCount; ++i)
	{
		// create spherical nodes checker
		const uint32 centerLeafIdx = (uint32) i;
		const Scope &scope = leaves.getScope(centerLeafIdx);
		const Vector3 leafCenterWS = scope.getCenterPosition();
		const Real radius = scope.getSize() * mRelativeRadiusForPriors;
		const uint32 maxDepth = (uint32) tree.getNodeDepth(scope.getNodeIndex()) + MAX_DEPTH_DIFFERENCE;
		const SphereNodeStatesChecker checker(leafCenterWS, radius, maxDepth, mNodeStates, NODE_FLAG_SAMPLENESS);

		// sum cone lengths around leaf centerLeafIdx
		Real sumOfEmptinessLength = 0.0f;
		Real sumOfSamplenessLengths = 0.0f;
		for (LeavesIterator it(tree, &checker); !it.isAtTheEnd(); ++it)
		{
			const uint32 leafIdx = it.getLeafIndex();
			const Real &localEmptinessLengthSum = mConeLengths[EMPTINESS][leafIdx];
			const Real &localSamplenessLengthSum = mConeLengths[SAMPLENESS][leafIdx];

			sumOfEmptinessLength += localEmptinessLengthSum;
			sumOfSamplenessLengths += localSamplenessLengthSum;
		}
		
		// p(Ce) = eL / (eL + sL), p(Cs) = sL / (eL + sL), p(Ce) + p(Cs) = 1
		const Real emptinessPrior = sumOfEmptinessLength / (sumOfEmptinessLength + sumOfSamplenessLengths);
		mPriorsForEmptiness[centerLeafIdx] = emptinessPrior;
	}
}

Real Occupancy::getOccupancy(const uint32 leafIdx) const
{
	// uncertain?
	const Real &emptiness = mKernelSums[EMPTINESS][leafIdx];
	const Real &sampleness = mKernelSums[SAMPLENESS][leafIdx];
	if (isUnreliable(emptiness, sampleness))
		return -EPSILON;

	// joint probabilities
	const Real pOfXAndCE = getPriorForEmptiness(leafIdx) * (emptiness / mConeCount);
	const Real pOfXAndCS = getPriorForSampleness(leafIdx) * (sampleness / mConeCount);

	// empty?
	if (pOfXAndCE >= pOfXAndCS)
		return -EPSILON;

	return pOfXAndCS - pOfXAndCE;
}

Real Occupancy::getEmptinessCPD(const uint32 leafIdx) const
{
	return mKernelSums[EMPTINESS][leafIdx] / mConeCount;
}

Real Occupancy::getSamplenessCPD(const uint32 leafIdx) const
{	
	return mKernelSums[SAMPLENESS][leafIdx] / mConeCount;
}

Occupancy::Occupancy(const Path &fileName) :
	Occupancy()
{
	loadFromFile(fileName);
}

Occupancy::Occupancy() :
	mCrust(NULL),
	mPriorsForEmptiness(NULL),
	mNodeStates(NULL),
	mConfidenceThreshold(EPSILON),
	mBandwidthFactor(1.0f)
{
	for (DataType type = EMPTINESS; type < DATA_TYPE_COUNT; type = (DataType) (type + 1))
	{
		mConeLengths[type] = NULL;
		mKernelSums[type] = NULL;
	}

	loadParameters();
}

void Occupancy::loadParameters()
{
	// manager & parameters to get
	const ParametersManager &manager = ParametersManager::getSingleton();
	const uint32 PARAMETER_COUNT = 4;
	const string names[PARAMETER_COUNT] = 
	{
		"Occupancy::bandwidthFactor",
		"Occupancy::confidenceThreshold",
		"Occupancy::relativeRadiusForPriors",
		"Occupancy::relativeSampleConeLength"
	};

	// parameters to get & their default values
	Real *parameters[PARAMETER_COUNT] = 
	{
		&mBandwidthFactor,
		&mConfidenceThreshold,
		&mRelativeRadiusForPriors,
		&mRelativeSampleConeLength
	};
	Real defaultValues[PARAMETER_COUNT] = { 1.0f, 10.0f, 15.0f, 4.0f };

	// get values from manager
	for (uint32 parameterIdx = 0; parameterIdx < 4; ++parameterIdx)
	{
		const bool loaded = manager.get(*(parameters[parameterIdx]), names[parameterIdx]);
		if (loaded)
			continue;

		cerr << "Missing occupancy parameter in config file:\t";
		cerr << names[parameterIdx];
		cerr << "\nSetting it to:\t";
		cerr << defaultValues[parameterIdx] << endl;
	}
}

Occupancy::~Occupancy()
{
	clear();
}

void Occupancy::loadFromFile(const Path &fileName)
{
	cout << "Loading occupancy data." << endl;
	clear();

	// get leaf & node count
	const Tree &tree = *Scene::getSingleton().getTree();
	const uint32 leafCount = tree.getLeaves().getCount();
	const uint32 nodeCount = tree.getNodes().getCount();
	if (0 == leafCount)
		return;

	// open file
	File file(fileName, File::OPEN_READING, true, FILE_VERSION);

	// read & check header data
	uint32 countsInFile[2];
	uint32 readCount = file.read(countsInFile, sizeof(uint32) * 2, sizeof(uint32), 2);
	if (2 != readCount || leafCount != countsInFile[0] || nodeCount != countsInFile[1])
		throw FileCorruptionException("Invalid occupancy file format.", fileName);

	// allocate memory
	for (DataType type = EMPTINESS; type < DATA_TYPE_COUNT; type = (DataType) (type + 1))
	{
		mConeLengths[type] = new Real[leafCount];
		mKernelSums[type] = new Real[leafCount];
	}

	mPriorsForEmptiness = new Real[leafCount];
	mNodeStates = new uint32[nodeCount];

	// read body
	{
		// read leaves data
		const size_t leavesRealBytesCount = sizeof(Real) * leafCount;
		for (DataType type = EMPTINESS; type < DATA_TYPE_COUNT; type = (DataType) (type + 1))
		{
			if (leafCount != file.read(mConeLengths[type], leavesRealBytesCount, sizeof(Real), leafCount))
				throw FileCorruptionException("Occupancy file with missing values for leaves.", fileName);
			if (leafCount != file.read(mKernelSums[type], leavesRealBytesCount, sizeof(Real), leafCount))		
				throw FileCorruptionException("Occupancy file with missing values for leaves.", fileName);
		}

		// load priors
		if (leafCount != file.read(mPriorsForEmptiness, leavesRealBytesCount, sizeof(Real), leafCount))
			throw FileCorruptionException("Occupancy file with missing values for leaves.", fileName);

		// load node states
		if (nodeCount != file.read(mNodeStates, sizeof(uint32) * nodeCount, sizeof(uint32), nodeCount))
			throw FileCorruptionException("Occupancy file with missing states for nodes.", fileName);

		// read cone count M
		if (1 != file.read(&mConeCount, sizeof(Real), sizeof(uint32), 1))
			throw FileCorruptionException("Could not read occupancy data.", fileName);
	}
	
	cout << "Finished occupancy data loading." << endl;
}

void Occupancy::saveToFile(const Path &fileName) const
{
	cout << "Saving occupancy to file." << endl;

	// valid data which can be saved?
	const Tree &tree = *Scene::getSingleton().getTree();
	uint32 counts[2] = { tree.getLeaves().getCount(), tree.getNodes().getCount() };
	if (0 == counts[0] || 0 == counts[1])
		return;

	for (DataType type = EMPTINESS; type < DATA_TYPE_COUNT; type = (DataType) (type + 1))
		if(!mKernelSums[type])
			return;

	// create the file
	File file(fileName, File::CREATE_WRITING, true, FILE_VERSION);
	
	// header
	if (2 != file.write(counts, sizeof(uint32), 2))
		throw FileCorruptionException("Could not write occupancy file header.", fileName);

	// body
	{
		// save cone lengths & kernel sums
		for (DataType type = EMPTINESS; type < DATA_TYPE_COUNT; type = (DataType) (type + 1))
		{
			if (counts[0] != file.write(mConeLengths[type], sizeof(Real), counts[0]))
				throw FileCorruptionException("Could not write occupancy data of leaves.", fileName);
			if (counts[0] != file.write(mKernelSums[type], sizeof(Real), counts[0]))
				throw FileCorruptionException("Could not write occupancy data of leaves.", fileName);
		}

		// save priors
		if (counts[0] != file.write(mPriorsForEmptiness, sizeof(Real), counts[0]))
			throw FileCorruptionException("Could not write occupancy data of leaves.", fileName);

		// node states
		if (counts[1] != file.write(mNodeStates, sizeof(uint32), counts[1]))
			throw FileCorruptionException("Could not write node states.", fileName);

		// write cone count M
		if (1 != file.write(&mConeCount, sizeof(Real), 1))
			throw FileCorruptionException("Could not write occupancy data.", fileName);
	}
}

void Occupancy::clear()
{
	// free memory & invalidate pointers
	delete mCrust;
	delete [] mNodeStates;
	delete [] mPriorsForEmptiness;

	mCrust = NULL;
	mNodeStates = NULL;
	mPriorsForEmptiness = NULL;

	for (DataType type = EMPTINESS; type < DATA_TYPE_COUNT; type = (DataType) (type + 1))
	{
		delete [] mConeLengths[type];
		delete [] mKernelSums[type];

		mConeLengths[type] = NULL;
		mKernelSums[type] = NULL;
	}
}

void Occupancy::outputEdgeConflicts() const
{		
	if (!mCrust)
		return;

	// output edge conflicts
	const FlexibleMesh &crustSurface = mCrust->getSurface();
	const vector<EdgeConflict> &conflicts = crustSurface.getEdgeConflicts();

	for (size_t i = 0; i < conflicts.size(); ++i)
	{
		const EdgeConflict &conflict = conflicts[i];
		cout << "Edge conflict set, edge idx: " << conflict.mEdgeIdx << "\n";

		cout << "Triangles: ";
		const vector<uint32> &set = conflict.mTriangles;
		for (size_t j = 0; j < set.size(); ++j)
		{
			cout << set[j] << " ";
		}
		cout << "\n";
	}

	cout << endl;
}