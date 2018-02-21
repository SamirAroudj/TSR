/*
 * Copyright (C) 2018 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */
#include "CollisionDetection/CollisionDetection.h"
#include "Graphics/Color.h"
#include "Graphics/ImageManager.h"
#include "Platform/FailureHandling/Exception.h"
#include "Platform/FailureHandling/FileException.h"
#include "Platform/Storage/Directory.h"
#include "Platform/Utilities/ParametersManager.h"
#include "Platform/Utilities/PlyFile.h"
#include "Platform/Utilities/RandomManager.h"
#include "Platform/Utilities/SVGLoader.h"
#include "SurfaceReconstruction/Geometry/FlexibleMesh.h"
#include "SurfaceReconstruction/Geometry/RayTracer.h"
#include "SurfaceReconstruction/Geometry/StaticMesh.h"
#include "SurfaceReconstruction/Image/ColorImage.h"
#include "SurfaceReconstruction/Image/DepthImage.h"
#include "SurfaceReconstruction/Refinement/MeshRefiner.h"
#include "SurfaceReconstruction/Scene/Camera/Cameras.h"
#include "SurfaceReconstruction/Scene/Camera/MVECameraIO.h"
#include "SurfaceReconstruction/Scene/FileNaming.h"
#include "SurfaceReconstruction/Scene/Samples.h"
#include "SurfaceReconstruction/Scene/SyntheticScene.h"
#include "tinyxml2.h"

using namespace CollisionDetection;
using namespace FailureHandling;
using namespace Graphics;
using namespace Math;
using namespace std;
using namespace Storage;
using namespace SurfaceReconstruction;
using namespace tinyxml2;
using namespace Utilities;

// constants
const uint32 SyntheticScene::RANDOM_SEED = 0;

SyntheticScene::SyntheticScene(const Path &fileName, const std::vector<IReconstructorObserver *> &observers) :
	Scene(observers)
{
	if (!getParameters(fileName))
		return;

	mGroundTruth = new StaticMesh(mGroundTruthName);

	// results folder to store the synthetic views and depth maps
	if (!Directory::createDirectory(getResultsFolder()))
		return;

	// scene AABB which encapsulates the mesh and the cameras
	mGroundTruth->computeAABB(mMeshAABB[0], mMeshAABB[1]);
	mAABB[0] = mMeshAABB[0];
	mAABB[1] = mMeshAABB[1];

	const Vector3 size = mAABB[1] - mAABB[0];
	const Vector3 changeFactors = (mRelativeSceneBorder - Vector3(1.0f, 1.0f, 1.0f));
	for (Axis axis = AXIS_X; axis <= AXIS_Z; axis = (Axis) (axis + 1))
	{
		const Real change = 0.5f * changeFactors[axis] * size[axis];

		if (1 == axis)
		{
			mAABB[1][axis] += 2.0f * change;
		}
		else
		{
			mAABB[0][axis] -= change;
			mAABB[1][axis] += change;
		}
	}

	RandomManager::getSingleton().setSeed(RANDOM_SEED);
	createAndSaveSamples();

	// save views to file
	MVECameraIO mveIO(Path::appendChild(getResultsFolder(), mRelativeCamerasFile));
	mveIO.saveCamerasToFile(mCameras);
}

bool SyntheticScene::getParameters(const Path &fileName)
{
	// load base parameters
	if (!Scene::getParameters(fileName))
		return false;

	// load parameters for synthetic scene
	const string missingParameter = "Missing parameter in synthetic scene description file: ";
	ParametersManager &manager = ParametersManager::getSingleton();

	// create empty views
	if (!manager.get(mMaxCameraCount, "cameraCount"))
	{
		cerr << missingParameter << "uint32 cameraCount = <count>\n";
		return false;
	}
	mCameras.reserve(mMaxCameraCount);
	
	// load view resolution
	uint32 width = 400;
	uint32 height = 300;
	if (!manager.get(width, "imageWidth") || !manager.get(height, "imageHeight"))
	{
		cerr << missingParameter << "uint32 imageWidth = <width>; or uint32 imageHeight = <height>;\n";
		return false;
	}
	mImageResolution.set(width, height);

	// load focal length bounds
	mMinFocalLength = 1.0f;
	mMaxFocalLength = 3.0f;
	if (!manager.get(mMinFocalLength, "minimumFocalLength") || !manager.get(mMaxFocalLength, "maximumFocalLength"))
		cerr << missingParameter << "Real minimumFocalLength = <min>; or Real maximumFocalLength = <max>;\n";

	// load minimum depth
	mMinSampleDistance = 1.0f;
	if (!manager.get(mMinSampleDistance, "minimumDepth"))
		cerr << missingParameter << "Real minimumDepth = <depth>\n";

	// load scene extend
	mRelativeSceneBorder.set(3.0f, 3.0f, 3.0f);
	if (!manager.get(mRelativeSceneBorder.x, "relativeSceneBorderX") || !manager.get(mRelativeSceneBorder.y, "relativeSceneBorderY") || !manager.get(mRelativeSceneBorder.z, "relativeSceneBorderZ"))
		cerr << missingParameter << "Real relativeSceneBorderX = <width>; Real relativeSceneBorderY = <height>; or Real relativeSceneBorderZ = <depth>;\n";
	
	// unbalanced camera distribution?
	mCameraBalance = 1.0f;
	if (!manager.get(mCameraBalance, "cameraBalance"))
		cerr << missingParameter << "Real cameraBalance = <balancing value>;\n";
	
	// depth map noise configuration
	mDepthMapNoise[0] = 0.0f;
	mDepthMapNoise[1] = 0.0f;
	if (!manager.get(mDepthMapNoise[0], "relativeNoiseMean") || !manager.get(mDepthMapNoise[1], "relativeNoiseStandardDeviation"))
		cerr << missingParameter << "Real relativeNoiseMean = <mean>; or Real relativeNoiseStandardDeviation = <sigma>;\n";

	// ground truth file name
	string temp;
	if (!manager.get(temp, "groundTruthFile"))
	{
		cerr << missingParameter << "string groundTruthFile = <fileName>;\n";
		return false;
	}
	mGroundTruthName = temp;

	return true;
}

void SyntheticScene::createAndSaveSamples()
{	
	// create folder for images if necessary
	if (!Directory::createDirectory(Scene::getSingleton().getViewsFolder()))
		return;

	// create syntehtic cameras & images
	createSyntheticImages();
	
	// view meshes from synthetic images
	vector<uint32> scales;
	scales.push_back(0); // load s0 images & depth maps
	loadDepthMeshes(scales);
	mSamples.addSamplesViaMeshes(mDepthMeshes);
}

void SyntheticScene::createSyntheticImages()
{
	// ray tracing preparation
	RayTracer rayTracer;
	rayTracer.createStaticScene(*mGroundTruth, true);

	// create the scene and target depth map for the ray tracer
	const uint32 pixelCount = mImageResolution.getElementCount();
	vector<Vector3> positionsWSMap(pixelCount);
	vector<Real> depthMap(pixelCount);
	uint32 validDepthCount;

	// create a camera & depth map until there are mMaxCameraCount cams
	for (uint32 cameraIdx = 0; cameraIdx < mMaxCameraCount; )
	{
		// create candidate camera data
		createSyntheticCamera();

		// trace scene -> depth, position (in world space) & index map
		const bool good = fill(depthMap, positionsWSMap, rayTracer, validDepthCount, mCameras.getCamera(cameraIdx));
		if (0 == validDepthCount || !good)
		{
			mCameras.popBack();
			continue;
		}
		
		// create directory
		const uint32 viewID = mCameras.getViewID(cameraIdx);
		const Path viewFolder = getViewFolder(viewID);
		if (!Directory::createDirectory(viewFolder))
			throw FileException("Could not create view directory!", viewFolder);

		// noise on surface samples
		addNoise(positionsWSMap, depthMap, mCameras.getPositionWS(cameraIdx));
		
		// save depth map as undistorted color image and MVEI depth map
		const Path depthMapName = getRelativeImageFileName(viewID, FileNaming::IMAGE_TAG_DEPTH, 0, false);
		Image::saveAsMVEFloatImage(depthMapName, true, mImageResolution, depthMap.data(), false, false);
		saveColorImage(depthMap, viewID, false);
		++cameraIdx;
	}
}

void SyntheticScene::createSyntheticCamera()
{
	// randomly distribute the cameras in the AABB
	RandomManager &randomManager = RandomManager::getSingleton();

	// camera data
	const Vector3 up(0.0f, 1.0f, 0.0f);
	const Vector2 principalPoint(0.5f, 0.5f);
	const Real aspectRatio = (Real) mImageResolution[0] / (Real) mImageResolution[1];
	const Real distortion[2] = { 0.0f, 0.0f };

	// AABB for camera positions
	const Real centerX = (mAABB[0].x + mAABB[1].x) * 0.5f;
	Vector3 min = mAABB[0];
	Vector3 max = mAABB[1];

	// on which side of the mesh?
	const Real sideRandomness = randomManager.getUniform(0.0f, mCameraBalance + 1.0f);
	const bool firstSide = (sideRandomness < 1.0f);
	if (firstSide)
	{
		min.x = mAABB[0].x;
		max.x = centerX;
	}
	else
	{
		min.x = centerX;
		max.x = mAABB[1].x;
	}

	// random position within mAABB
	Vector3 pWS;
	do
	{
		pWS = randomManager.getUniform(min, max);
	}
	while (CollisionDetection::isPointInAABB(pWS, mMeshAABB));

	// random focal length
	const Real focalLength = randomManager.getUniform(mMinFocalLength, mMaxFocalLength);

	// create camera
	const Vector3 pHWS(pWS.x, pWS.y, pWS.z);
	const Vector3 lookAt = randomManager.getUniform(mMeshAABB[0], mMeshAABB[1]);

	const uint32 cameraIdx = mCameras.getCount();
	mCameras.addCamera(cameraIdx, Quaternion(), pHWS, focalLength, principalPoint, aspectRatio, distortion);
	mCameras.getCamera(cameraIdx).lookAt(pWS, lookAt, up);
}

bool SyntheticScene::fill(vector<Real> &depthMap, vector<Vector3> &positionsWSMap,
	RayTracer &rayTracer, uint32 &validDepthCount, const PinholeCamera &camera)
{
	const Matrix3x3 HPSToNNRayDirWS = camera.computeHPSToNNRayDirWS(mImageResolution, true);
	const Vector3 camPosWS(camera.getPosition().x, camera.getPosition().y, camera.getPosition().z);
	const Real minDepthSq = mMinSampleDistance * mMinSampleDistance;
	const uint32 pixelCount = mImageResolution.getElementCount();

	// find surface points
	validDepthCount = 0;
	rayTracer.render(NULL, mImageResolution, camera, HPSToNNRayDirWS, true);

	// fill depth & index map
	for (uint32 pixelIdx = 0; pixelIdx < pixelCount; ++pixelIdx)
	{
		// initial invalid value
		positionsWSMap[pixelIdx].set(-REAL_MAX, -REAL_MAX, -REAL_MAX);
		depthMap[pixelIdx] = -REAL_MAX;

		// valid position / depth?
		if (!rayTracer.getHitValidity(pixelIdx))
			continue;

		// compute & store world space position & depth map depth
		Vector3 &positionWS = positionsWSMap[pixelIdx];
		positionWS = rayTracer.getHit(NULL, pixelIdx);

		const Real lengthSq = (positionWS - camPosWS).getLengthSquared();
		if (lengthSq < minDepthSq)
			return false;

		const Real depth = sqrtr(lengthSq);
		depthMap[pixelIdx] = depth;
		++validDepthCount;
	}

	return true;
}

void SyntheticScene::addNoise(vector<Vector3> &positionsWSMap, vector<Real> &depthMap, const Vector3 &camPosWS)
{	
	// no noise?
	if (Math::EPSILON >= mDepthMapNoise[0] && Math::EPSILON >= mDepthMapNoise[1])
		return;

	// preparation
	const uint32 pixelCount = mImageResolution.getElementCount();
	RandomManager &manager = RandomManager::getSingleton();
	normal_distribution<Real> depthMapNoise(mDepthMapNoise[0], mDepthMapNoise[1]);

	// add noise to each valid depth value
	for (uint32 pixelIdx = 0; pixelIdx < pixelCount; ++pixelIdx)
	{
		const Real noiseFactor = manager.getNormal(depthMapNoise);

		// valid depth?
		Vector3 &positionWS = positionsWSMap[pixelIdx];
		Real &depth = depthMap[pixelIdx];
		if (-REAL_MAX == depth)
			continue;

		// add noise to depth
		const Real noise = (sqrtr(depth) * noiseFactor);
		depth += noise;  

		// add noise to positionWS
		Vector3 direction = positionWS - camPosWS;
		direction.normalize();
		positionWS = camPosWS + direction * depth;
	}
}

void SyntheticScene::saveColorImage(const vector<Real> &depthMap, const uint32 cameraIdx, const bool withNoise) const
{
	// create file name
	// file name with noise data inside?
	string localName = FileNaming::IMAGE_TAG_COLOR_S0;
	if (withNoise)
	{
		char buffer[File::READING_BUFFER_SIZE];
		snprintf(buffer, File::READING_BUFFER_SIZE, "Mean" REAL_IT "StdDev" REAL_IT, mDepthMapNoise[0], mDepthMapNoise[1]);
		localName += buffer;
	}

	// absolute file name for color image
	const Path relativeFileName = getRelativeImageFileName(cameraIdx, localName, 0, true);
	const Path viewsFolder = Scene::getSingleton().getViewsFolder();
	const Path absoluteName = Path::appendChild(viewsFolder, relativeFileName);

	// convert depth to gray values
	const uint32 pixelCount = mImageResolution.getElementCount();
	const uint32 channelCount = 3;
	uint8 *grayValues = new uint8[pixelCount * channelCount];

	// find depth extrema and correspondingly scale depths to gray values
	Real minDepth, maxDepth;
	DepthImage::findExtrema(minDepth, maxDepth, depthMap.data(), pixelCount);
	DepthImage::convertDepthsToColor(grayValues, depthMap.data(), pixelCount, channelCount, minDepth, maxDepth);

	// save the gray values to file
	File file(absoluteName, File::CREATE_WRITING, true);
	ImageManager::getSingleton().savePNG(file, grayValues, channelCount, mImageResolution);

	delete [] grayValues;
	grayValues = NULL;
}

SyntheticScene::~SyntheticScene()
{
	clear();
}
