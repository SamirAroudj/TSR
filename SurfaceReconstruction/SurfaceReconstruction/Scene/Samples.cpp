/*
 * Copyright (C) 2018 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */

#include "CollisionDetection/CollisionDetection.h"
#include "Math/MathCore.h"
#include "Math/MathHelper.h"
#include "Platform/FailureHandling/Exception.h"
#include "Platform/Storage/File.h"
#include "Platform/ParametersManager.h"
#include "Platform/Platform.h"
#include "SurfaceReconstruction/Geometry/FlexibleMesh.h"
#include "SurfaceReconstruction/Scene/FileNaming.h"
#include "SurfaceReconstruction/Scene/Samples.h"
#include "SurfaceReconstruction/Scene/Scene.h"
#include "SurfaceReconstruction/Scene/View/View.h"
#include "Utilities/PlyFile.h"
#include "Utilities/RandomManager.h"

using namespace FailureHandling;
using namespace Math;
using namespace Platform;
using namespace std;
using namespace Storage;
using namespace SurfaceReconstruction;
using namespace Utilities;

const uint32 Samples::FILE_VERSION = 0;
const uint32 Samples::INVALID_INDEX = (uint32) -1;

Samples::Samples(const uint32 viewsPerSample, const uint32 sampleCount, const Vector3 *AABBWS) :
	Samples()
{
	if (sampleCount > 0)
		resize(sampleCount);

	if (AABBWS)
	{
		mAABBWS[0] = AABBWS[0];
		mAABBWS[1] = AABBWS[1];
	}

	mViewsPerSample = viewsPerSample;
}

Samples::Samples(const Path &fileName) :
	Samples()
{
	loadFromFile(fileName);
}

Samples::Samples() :
	mAABBWS{Vector3(REAL_MAX, REAL_MAX, REAL_MAX), Vector3(-REAL_MAX, -REAL_MAX, -REAL_MAX)},
	mMaxRelativeSamplingDistance(1.0f),
	mViewConeCount(0), mViewsPerSample(-1)
{
	// required user parameters
	const string samplingDistanceName = "Samples::maxRelativeSamplingDistance";

	// get parameters
	const ParametersManager &manager = ParametersManager::getSingleton();
	bool samplingDistanceLoaded = manager.get(mMaxRelativeSamplingDistance, samplingDistanceName);
	if (samplingDistanceLoaded)
		return;

	// error handling
	string message = "Scene: Could not load parameters:\n";
	if (!samplingDistanceLoaded)
	{
		message += samplingDistanceName;
		message += ", choosing 1.0\n";
	}

	cerr << message << endl;
}

Samples::~Samples()
{
	clear();
}

void Samples::clear()
{
	// release all samples
	mColors.clear();
	mColors.shrink_to_fit();

	mNormals.clear();
	mNormals.shrink_to_fit();

	mPositions.clear();
	mPositions.shrink_to_fit();

	mConfidences.clear();
	mConfidences.shrink_to_fit();

	mScales.clear();
	mScales.shrink_to_fit();

	mParentViews.clear();
	mParentViews.shrink_to_fit();
	
	mAABBWS[0] = Vector3(REAL_MAX, REAL_MAX, REAL_MAX);
	mAABBWS[1] = Vector3(-REAL_MAX, -REAL_MAX, -REAL_MAX);
	mViewConeCount = 0;
	mViewsPerSample = 0;
}

void Samples::addToAABB(Vector3 AABB[2], const uint32 sampleIdx) const
{
	// sample AABB
	Vector3 sampleMin;
	Vector3 sampleMax;
	getAABBWS(sampleMin, sampleMax, sampleIdx);

	AABB[0] = AABB[0].minimum(sampleMin);
	AABB[1] = AABB[1].maximum(sampleMax);
}

void Samples::check()
{
	const int64 sampleCount = getCount();

	// check sample scale / samples' 3D footprint sizes
	#pragma omp parallel for
	for (int64 sampleIdx = 0; sampleIdx < sampleCount; ++sampleIdx)
	{
		const Real &scale = getScale((uint32) sampleIdx);
		assert(scale > 0.0f);
		if (scale <= 0.0f)
			throw Exception("Invalid (non-positive) sample scale detected.");
	}
}

void Samples::computeAABB()
{
	mAABBWS[0].set(REAL_MAX, REAL_MAX, REAL_MAX);
	mAABBWS[1].set(-REAL_MAX, -REAL_MAX, -REAL_MAX);

	const uint32 sampleCount = (uint32) mNormals.size();
	for (uint32 sampleIdx = 0; sampleIdx < sampleCount; ++sampleIdx)
		addToAABB(mAABBWS, sampleIdx);
}

void Samples::compact(const uint32 *sampleOffsets)
{
	// counts for filling
	const uint32 oldSampleCount = getCount();
	const uint32 doomedSampleCount = sampleOffsets[oldSampleCount];
	const uint32 newSampleCount = oldSampleCount - doomedSampleCount;
	
	cout << "Deleting " << doomedSampleCount << " of " << oldSampleCount << " samples, remaining: " << newSampleCount << " samples." << endl;

	// filter colors
	{
		vector<Vector3> newColors(newSampleCount);
		FlexibleMesh::filterData<Vector3>(newColors.data(), mColors.data(), sampleOffsets, oldSampleCount);
		mColors.swap(newColors);
	}

	// filter normals
	{
		vector<Vector3> newNormals(newSampleCount);
		FlexibleMesh::filterData<Vector3>(newNormals.data(), mNormals.data(), sampleOffsets, oldSampleCount);
		mNormals.swap(newNormals);
	}

	// filter positions
	{
		vector<Vector3> newPositions(newSampleCount);
		FlexibleMesh::filterData<Vector3>(newPositions.data(), mPositions.data(), sampleOffsets, oldSampleCount);
		mPositions.swap(newPositions);
	}

	// filter confidences
	{
		vector<Real> newConfidences(newSampleCount);
		FlexibleMesh::filterData<Real>(newConfidences.data(), mConfidences.data(), sampleOffsets, oldSampleCount);
		mConfidences.swap(newConfidences);
	}

	// filter scales
	{
		vector<Real> newScales(newSampleCount);
		FlexibleMesh::filterData<Real>(newScales.data(), mScales.data(), sampleOffsets, oldSampleCount);
		mScales.swap(newScales);
	}

	// filter parent views
	{
		vector<uint32> newParentViews(newSampleCount * mViewsPerSample);
		FlexibleMesh::filterData<uint32>(newParentViews.data(), mParentViews.data(), sampleOffsets, oldSampleCount, mViewsPerSample);
		mParentViews.swap(newParentViews);
		computeParentViewCount();
	}


	cout << "Finished deletion of samples. " << endl;
}

bool Samples::computeMeans(Vector3 &meanColor, Vector3 &meanNormal, Vector3 &meanPosition, Real &meanScale,
	const vector<uint32> &sampleSet, bool weightedByConfidences) const
{
	// weighted mean data of sample set
	const Real MIN_LENGTH_NORMALS	= 0.5f; // todo magic number

	// initial mean values
	meanColor.set(0.0f, 0.0f, 0.0f);
	meanPosition.set(0.0f, 0.0f, 0.0f);
	meanNormal.set(0.0f, 0.0f, 0.0f);
	meanScale = 0.0f;

	const uint32 count = (uint32) sampleSet.size();
	Real sum = 0.0f;	

	for (uint32 i = 0; i < count; ++i)
	{
		const uint32	index		= sampleSet[i];
		const Vector3	&color		= mColors[index];
		const Vector3	&normal		= mNormals[index];
		const Vector3	&position	= mPositions[index];
		const Real		confidence	= mConfidences[index];
		const Real		scale		= mScales[index];
		const Real		weight		= (weightedByConfidences ? mConfidences[index] : 1.0f);

		meanColor		+= color * weight;
		meanNormal		+= normal * weight;
		meanPosition	+= position * weight;
		meanScale		+= scale * weight;

		sum += weight;
	}

	// normalized results
	const Real normFactor = 1.0f / sum;
	
	meanColor	 *= normFactor;
	meanNormal	 *= normFactor;
	meanPosition *= normFactor;
	meanScale	 *= normFactor;

	const Real lengthSq = meanNormal.getLengthSquared();
	meanNormal /= sqrtr(lengthSq);

	// small distribution of sample normals?
	// it does not make sense to compute a model if sample oriantations are too chaotic
	return (lengthSq >= MIN_LENGTH_NORMALS * MIN_LENGTH_NORMALS); // todo magic number
}

bool Samples::computeViewAngles(Real &azimuthAngle, Real &polarAngle, const uint32 parentViewIdx, const uint32 sampleIdx) const
{
	// bad default values if something goes wrong
	azimuthAngle = -REAL_MAX;
	polarAngle	 = -REAL_MAX;

	Vector3 viewDirection;
	if (!computeViewDirection(viewDirection, parentViewIdx, sampleIdx))
		return false;

	Math::transformCartesianToSpherical(azimuthAngle, polarAngle, viewDirection);
	return true;
}

bool Samples::computeViewDirection(Vector3 &viewDirection, const uint32 parentViewIdx, const uint32 sampleIdx) const
{
	// bad default values if something goes wrong
	viewDirection.set(-REAL_MAX, -REAL_MAX, REAL_MAX);

	// check parent view index
	assert(parentViewIdx < mViewsPerSample);
	if (parentViewIdx >= mViewsPerSample)
		throw Exception("Invalid parentViewIdx for a sample.");

	// valid parent view?
	Scene					&scene	= Scene::getSingleton();
	const vector<View *>	&views	= scene.getViews();
	const uint32			viewIdx	= getViewIdx(parentViewIdx, sampleIdx);
	if (!scene.isValidView(viewIdx))
		return false;

	// compute view direction
	const View		*view	= views[viewIdx];
	const Vector4	&camWS	= view->getCamera().getPosition();
	viewDirection = mPositions[sampleIdx] - Vector3(camWS.x, camWS.y, camWS.z);
	viewDirection.normalize();
	return true;
}

void Samples::erase(const vector<uint32> theDoomed, const uint32 doomedCount)
{
	const uint32 temp = (uint32) theDoomed.size();
	uint32 numRemovedSamples = 0;
	for (uint32 i = 0; i < temp - 1; i += 2)
	{
		const uint32 targetIdx = theDoomed[i] - numRemovedSamples;
		const uint32 doomedCount = theDoomed[i + 1];
		const uint32 sourceIdx = theDoomed[i] + doomedCount;
		const uint32 next = theDoomed[i + 2];
		const uint32 moveCount = next - sourceIdx;
		if (0 == moveCount && next == getCount())
			continue;

		memcpy(&mColors[targetIdx], &mColors[sourceIdx], sizeof(Vector3) * moveCount);
		memcpy(&mNormals[targetIdx], &mNormals[sourceIdx], sizeof(Vector3) * moveCount);
		memcpy(&mPositions[targetIdx], &mPositions[sourceIdx], sizeof(Vector3) * moveCount);
		memcpy(&mConfidences[targetIdx], &mConfidences[sourceIdx], sizeof(Real) * moveCount);
		memcpy(&mScales[targetIdx], &mScales[sourceIdx], sizeof(Real) * moveCount);

		memcpy(&mParentViews[targetIdx * mViewsPerSample], &mParentViews[sourceIdx * mViewsPerSample], sizeof(uint32) * mViewsPerSample * moveCount);

		numRemovedSamples += doomedCount;
	}

	resize(getCount() - doomedCount);
	computeParentViewCount();
}

void Samples::computeParentViewCount()
{
	// get & check parent link count
	uint64 linkCount = mViewsPerSample * mNormals.size();
	if (linkCount >= (uint32) -1)
		throw Exception("Number of sample to parent view links exceeds supported maximum = 2^32 - 2.");
	const uint32 parentCount = (uint32) linkCount;

	// count number of valid parent links
	uint32 invalidCount = 0;
	for (uint32 parentIdx = 0; parentIdx < parentCount; ++parentIdx)
		if (View::INVALID_ID == mParentViews[parentIdx])
			++invalidCount;
	mViewConeCount = parentCount - invalidCount;

	cout << "Computed number of valid parent views for all samples.\n";
	cout << "Parent view count: " << mViewConeCount << "; invalid view link count: " << invalidCount << "; parent view link count: " << getCount() * mViewsPerSample << endl;
}

void Samples::getAABBWS(Vector3 &minWS, Vector3 &maxWS, const uint32 sampleIdx) const
{
	const Vector3 &p = mPositions[sampleIdx];
	const Real r = getMaxSamplingDistance(sampleIdx) + EPSILON;

	minWS.set(p.x - r, p.y - r, p.z - r);
	maxWS.set(p.x + r, p.y + r, p.z + r);
}

//Real Samples::getDistanceCosts(const uint32 sampleIdx0, const uint32 sampleIdx1) const
//{
//	// same sample?
//	if (sampleIdx0 == sampleIdx1)
//		return 0.0f;
//
//	// get sample data
//	return getDistanceCosts(mNormals[sampleIdx0], mPositions[sampleIdx0], getSupportRange(sampleIdx0),
//						   mNormals[sampleIdx1], mPositions[sampleIdx1], getSupportRange(sampleIdx1));
//}

//Real Samples::getDistanceCosts(const Vector3 &n0, const Vector3 &p0, const Real supportRange0,
//							  const Vector3 &n1, const Vector3 &p1, const Real supportRange1)
//{
//	// Euclidean distance between sample center points
//	const Real scaleDistance			= supportRange0 + supportRange1; //Real meanScale = (s0.mScale + s1.mScale) * 0.5f;
//	const Real relativeCenterDistance	= (p0 - p1).getLength() / scaleDistance;
//	
//	// angular difference of normals
//	const Real dotProduct = Math::clamp(n0.dotProduct(n1), 1.0f, -1.0f);
//	const Real deltaAngle = acosr(dotProduct);
//
//	// todo magic numbers
//
//	// "Euclidean costs" ec from relative distance between centers
//	// if relative distance == half of the max possible distance -> ec = euclideanCostsFactor
//	const Real maxRelativeDistance	= 1.0f;
//	const Real euclideanCostsFactor	= 0.05f;
//	const Real euclideanCosts		= Math::infinityMaximumCurve(relativeCenterDistance, maxRelativeDistance, euclideanCostsFactor);
//	
//	// "angular costs" ac from angle between normals
//	// if angle offset == half of the max possible angle -> ac = angularCostsFactor 
//	// infinite costs for angle offset >= max angle offset
//	const Real maxAngleDifference	= Math::PI * (45.0f / 180.0f);
//	const Real angularCostsFactor	= 1.0f;
//	const Real orientationCosts		= Math::infinityMaximumCurve(deltaAngle, maxAngleDifference, angularCostsFactor);
//
//	return orientationCosts + euclideanCosts;
//}

//Real Samples::getDistance(const vector<uint32> &sampleSet, const Vector3 &positionWS) const
//{
//	// compute minimum distance to all sample spheres of the entered set sampleSet
//	const uint32 sampleCount = (uint32) sampleSet.size();
//
//	Real minimum = REAL_MAX;
//	for (uint32 i = 0; i < sampleCount; ++i)
//	{
//		const uint32 globalIdx	 = sampleSet[i];
//		const Vector3 &samplePosWS = mPositions[globalIdx];
//		const Real supportRange	 = getSupportRange(globalIdx);
//
//		const Real distance = (samplePosWS - positionWS).getLength() - supportRange;
//		if (distance < 0.0f)
//			return 0.0f;
//
//		if (distance >= minimum)
//			continue;
//
//		minimum = distance;
//	}
//
//	return minimum;
//}

Real Samples::getDistanceToPlane(const Vector3 &pWS, const uint32 sampleIdx) const
{
	CollisionDetection::Plane plane(mPositions[sampleIdx], mNormals[sampleIdx], true);
	return plane.getDistanceToPlane(pWS);
}

Real Samples::getFSSRWeight(const Math::Vector3 &pWS, const uint32 sampleIdx) const
{
	const Vector3 vSS = toSampleSpace(pWS, sampleIdx);
	const Real s = mScales[sampleIdx];

	// FSSR weighting function: weightX along x axis * weightY along y axis
	const Real range	= 3.0f * s;
	const Real scaleSq	= s * s;
	const Real scaleCu	= scaleSq * s;
	const Real xSq		= vSS.x * vSS.x;
		  Real weight	= 0.0f;

	// weight along normal
	if (vSS.x > -range && vSS.x < 0.0f)
		weight = xSq / (9.0f * scaleSq) + 2.0f * vSS.x / range + 1.0f;
	else if (vSS.x >= 0.0f && vSS.x < range)
		weight = 2.0f * xSq * vSS.x / (27.0f * scaleCu) - xSq / (3.0f * scaleSq) + 1.0f;
	else
		return 0.0f;

	// weight along tangent
	const Real tSq	= vSS.y * vSS.y + vSS.z * vSS.z;
	const Real t	= sqrtr(tSq);
	if (t < range)
		weight *= 2.0f * tSq * t / (27.0f * scaleCu) - tSq / (3.0f * scaleSq) + 1.0f;
	else
		return 0.0f;

	return weight;
}

Real Samples::getMeasureDistanceSquared(const uint32 sampleIdx, const uint32 parentViewIdx) const
{
	// get parent view position
	const vector<View *> &views = Scene::getSingleton().getViews();
	const uint32 globalViewIdx = getParentViewIndices(sampleIdx)[parentViewIdx];
	const Vector3 &viewPosWS = views[globalViewIdx]->getPositionWS();
	
	// measurement distance / sample depth
	const Vector3 viewToSample = getPositionWS(sampleIdx) - viewPosWS;
	const Real distanceSq = viewToSample.getLengthSquared();

	return distanceSq;
}

uint32 Samples::getViewIdx(const uint32 parentViewIdx, const uint32 sampleIdx) const
{
	// sanity checks
	const uint32 sampleCount = getCount();
	assert(parentViewIdx < mViewsPerSample);
	assert(sampleIdx < sampleCount);

	if (parentViewIdx >= mViewsPerSample || sampleIdx >= sampleCount)
		return View::INVALID_ID;
	else
		return mParentViews[sampleIdx * mViewsPerSample + parentViewIdx];
}

void Samples::setSample(const uint32 targetIdx, const Vector3 &color, const Vector3 &normal, const Vector3 &positionWS,
	const Real confidence, const Real scale, const uint32 *parentViewIDs)
{
	// update simple sample attributes
	mColors[targetIdx] = color;
	mNormals[targetIdx] = normal;
	mPositions[targetIdx] = positionWS;
	mConfidences[targetIdx] = confidence;
	mScales[targetIdx] = scale;

	// update parent views and parent veiw count
	int32 parentViewCountChange = 0;
	uint32 *parentViews = mParentViews.data() + targetIdx * mViewsPerSample;

	for (uint32 i = 0; i < mViewsPerSample; ++i)
	{
		// is a parent view added or erased?
		if (View::INVALID_ID == parentViews[i] && View::INVALID_ID != parentViewIDs[i])
			++parentViewCountChange;
		else if (View::INVALID_ID != parentViews[i] && View::INVALID_ID == parentViewIDs[i])
			--parentViewCountChange;

		parentViews[i] = parentViewIDs[i];
	}

	mViewConeCount += parentViewCountChange;
}

void Samples::makeNoisy(normal_distribution<Real> noise[3], const uint32 sampleIdx)
{
	// get necessary variables
	RandomManager	&random		= RandomManager::getSingleton();
	Vector3			&normal		= mNormals[sampleIdx];
	Vector3			&position	= mPositions[sampleIdx];
	Real			&scale		= mScales[sampleIdx];
	
	// get view direction
	Vector3 viewDirection;
	if (!computeViewDirection(viewDirection, 0, sampleIdx))
	{
		// todo log this
		viewDirection.set(random.getNormal(noise[2]), random.getNormal(noise[2]), random.getNormal(noise[2]));
		viewDirection.normalize();
	}

	// add noise to position
	const Real positionNoise = scale * random.getNormal(noise[0]);
	position += viewDirection * positionNoise;

	// add noise to scale
	const Real f = 1.0f + random.getNormal(noise[1]);
	scale = scale * Math::clamp<Real>(f, 1.75f, 0.25f);

	// add noise to normal
	Real azimuth;
	Real polar;

	Math::transformCartesianToSpherical(azimuth, polar, normal, 1.0f);
	azimuth += random.getNormal(noise[2]);
	polar	+= random.getNormal(noise[2]);
	Math::transformSphericalToCartesian(normal, azimuth, polar, 1.0f);
}

uint32 Samples::addSample()
{
	const uint32 sampleIdx	= (uint32) mNormals.size();
	const size_t count		= sampleIdx + 1;
	if(count >= (uint32) -1)
		throw Exception("Cannot create more samples. Only up to 2^32 - 2 samples can be created.");

	// resize containers
	mColors.resize(count);
	mNormals.resize(count);
	mPositions.resize(count);
	mConfidences.resize(count, 1.0f);
	mScales.resize(count);
	mParentViews.resize(count * mViewsPerSample);

	invalidateViews(sampleIdx);
	return sampleIdx;
}

void Samples::invalidateViews(const uint32 sampleIdx)
{
	const uint32 offset = sampleIdx * mViewsPerSample;
	for (uint32 i = 0; i < mViewsPerSample; ++i)
		mParentViews[offset + i] = View::INVALID_ID;
}

void Samples::deleteSample(const uint32 sampleIdx)
{
	// swap last sample with replacedIdx and erase the last (replacedIdx)
	const uint32 lastSampleIdx = (uint32) mNormals.size() - 1;
	swap(sampleIdx, lastSampleIdx);
	popBackSample();
}

void Samples::popBackSample()
{
	// erase last sample from all containers
	mColors.pop_back();
	mNormals.pop_back();
	mPositions.pop_back();
	mConfidences.pop_back();
	mScales.pop_back();

	// erase parent views & reduce the number of parent views accordingly
	const uint32 vectorSize = (uint32) mParentViews.size();
	uint32 erasedParentViewsCount = 0;
	for (uint32 parentViewIdx = vectorSize - mViewsPerSample; parentViewIdx < vectorSize; ++parentViewIdx)
		if (View::INVALID_ID != mParentViews[parentViewIdx])
			++erasedParentViewsCount;

	mParentViews.resize(mParentViews.size() - mViewsPerSample);
	mViewConeCount -= erasedParentViewsCount;
}

void Samples::reserve(const size_t sampleCount)
{
	if (sampleCount >= (uint32) -1)
		throw Exception("Sample count is larger than 2^32 - 2 which is not supported.");

	mColors.reserve(sampleCount);
	mNormals.reserve(sampleCount);
	mPositions.reserve(sampleCount);
	mConfidences.reserve(sampleCount);
	mScales.reserve(sampleCount);
	mParentViews.reserve(sampleCount * mViewsPerSample);
}

void Samples::swap(const uint32 i, const uint32 j)
{
	if (i == j)
		return;

	// swap all which belongs to the two normals
	Vector3 temp3;
	Real temp;

	// swap color, normal, position & scale
	temp3 = mColors[i];
	mColors[i] = mColors[j];
	mColors[j] = temp3;

	temp3		= mNormals[i];
	mNormals[i]	= mNormals[j];
	mNormals[j]	= temp3;

	temp3			= mPositions[i];
	mPositions[i]	= mPositions[j];
	mPositions[j]	= temp3;

	temp = mConfidences[i];
	mConfidences[i] = mConfidences[j];
	mConfidences[j] = temp;

	temp		= mScales[i];
	mScales[i]	= mScales[j];
	mScales[j]	= temp;

	// update view indices
	const uint32 offsetI = i * mViewsPerSample;
	const uint32 offsetJ = j * mViewsPerSample;

	for (uint32 viewIdx = 0; viewIdx < mViewsPerSample; ++viewIdx)
	{
		const uint32 viewI = offsetI + viewIdx;
		const uint32 viewJ = offsetJ + viewIdx;

		const uint32 temp = mParentViews[viewI];
		mParentViews[viewI] = mParentViews[viewJ];
		mParentViews[viewJ] = temp;
	}
}

Vector3 Samples::toSampleSpace(const Vector3 &pWS, const uint32 sampleIdx) const
{
	// transform to sample space "SS" which depends on sample position and orientation
	Matrix3x3 offsetWSToSS = Matrix3x3::createBasisFromVector(mNormals[sampleIdx]);
	offsetWSToSS.transpose();

	// convert pWS into coordinate system(mPosition, mNormal, tangent0, tangent1)
	const Vector3 offsetWS = pWS - mPositions[sampleIdx];
	const Vector3 pSS = offsetWS * offsetWSToSS;

	return pSS;
}

void Samples::transform(const uint32 sampleIdx, const Matrix3x3 &transformation, const Vector3 &translation)
{
	mNormals[sampleIdx] = mNormals[sampleIdx] * transformation;
	mPositions[sampleIdx] = (mPositions[sampleIdx] * transformation + translation);
}

void Samples::loadFromFile(const Path &fileName)
{
	cout << "Loading samples from file " << fileName << "." << endl;
	
	// open file & check version
	File file(fileName, File::OPEN_READING, true, FILE_VERSION);

	// get #samples & #views per sample & request memory
	uint32 sampleCount;
	file.read(&sampleCount, sizeof(uint32), sizeof(uint32), 1);
	file.read(&mViewsPerSample, sizeof(uint32), sizeof(uint32), 1);
	file.read(mAABBWS, sizeof(Vector3) * 2, sizeof(Vector3), 2);

	resize(sampleCount);

	// load samples
	uint32 parentCount = sampleCount * mViewsPerSample;

	file.read(mColors.data(), sizeof(Vector3) * sampleCount, sizeof(Vector3), sampleCount);
	file.read(mNormals.data(), sizeof(Vector3) * sampleCount, sizeof(Vector3), sampleCount);
	file.read(mPositions.data(), sizeof(Vector3) * sampleCount, sizeof(Vector3), sampleCount);
	file.read(mConfidences.data(), sizeof(Real) * sampleCount, sizeof(Real), sampleCount);
	file.read(mScales.data(), sizeof(Real) * sampleCount, sizeof(Real), sampleCount);
	file.read(mParentViews.data(), sizeof(uint32) * parentCount, sizeof(uint32), parentCount);
	
	computeParentViewCount();

	cout << "Loaded " << getCount() << " samples." << endl;
}

void Samples::saveToFile(const Path &beginning, const bool saveAsPly, const bool saveAsSamples) const
{
	// is there anything to save?
	if (mNormals.empty())
		return;

	// as Stanford ply mesh?
	if (saveAsPly)
	{
		const Path fileName = Path::extendLeafName(beginning, FileNaming::ENDING_PLY);
		PlyFile file(fileName, File::CREATE_WRITING, true);
		file.saveTriangleMesh(ENCODING_BINARY_LITTLE_ENDIAN, true,
			getCount(), 0, mColors.data(), mNormals.data(), mPositions.data(), 
			mConfidences.data(), mScales.data(), mParentViews.data(), mViewsPerSample, NULL);
	}
	
	// internal mesh format?
	if (saveAsSamples)
	{
		// create file & write version
		const Path fileName = Path::extendLeafName(beginning, FileNaming::ENDING_SAMPLES);
		File file(fileName, File::CREATE_WRITING, true, FILE_VERSION);

		// save sample count, views per sample & AABB
		const uint32 sampleCount = (uint32) mNormals.size();
		file.write(&sampleCount, sizeof(uint32), 1);
		file.write(&mViewsPerSample, sizeof(uint32), 1);
		file.write(mAABBWS, sizeof(Vector3), 2);

		// save all samples
		file.write(mColors.data(), sizeof(Vector3), sampleCount);
		file.write(mNormals.data(), sizeof(Vector3), sampleCount);
		file.write(mPositions.data(), sizeof(Vector3), sampleCount);
		file.write(mConfidences.data(), sizeof(Real), sampleCount);
		file.write(mScales.data(), sizeof(Real), sampleCount);
		file.write(mParentViews.data(), sizeof(uint32), sampleCount * mViewsPerSample);
	}
}

void Samples::resize(const uint32 sampleCount)
{
	const uint32 oldCount = (uint32) mColors.size();
	mColors.resize(sampleCount);
	mNormals.resize(sampleCount);
	mPositions.resize(sampleCount);
	mConfidences.resize(sampleCount, 1.0f);
	mScales.resize(sampleCount);
	mParentViews.resize(sampleCount * mViewsPerSample);

	// invalid views for all new samples
	if (sampleCount <= oldCount)
		return;

	const uint32 newSamples = sampleCount - oldCount;
	const size_t byteCount = newSamples * mViewsPerSample * sizeof(uint32);
	memset(mParentViews.data() + oldCount * mViewsPerSample, View::INVALID_ID, byteCount);
}

void Samples::unifyInvalidParentViewIDs()
{
	// get number of views and sample to parent view links
	const uint32 viewCount = Scene::getSingleton().getViewCount();
	const int64 linkCount = getMaxViewConeCount();

	// set each view ID >= view count to View::INVALID_ID
	#pragma omp parallel for
	for (int64 linkIdx = 0; linkIdx < linkCount; ++linkIdx)
	{
		uint32 &ID = mParentViews[linkIdx];
		if (ID >= viewCount)
			ID = View::INVALID_ID;
	}
}

void Samples::updateParentViews(const map<uint32, uint32> &oldToNewViewIDs)
{
	// update each link
	const int64 linkCount = mPositions.size() * mViewsPerSample;
	#pragma omp parallel for
	for (int64 linkIdx = 0; linkIdx < linkCount; ++linkIdx)
	{
		const uint32 oldParentID = mParentViews[linkIdx];
		map<uint32, uint32>::const_iterator it = oldToNewViewIDs.find(oldParentID);
		if (oldToNewViewIDs.end() != it)
			mParentViews[linkIdx] = it->second;
		else
			mParentViews[linkIdx] = View::INVALID_ID;
	}
}
