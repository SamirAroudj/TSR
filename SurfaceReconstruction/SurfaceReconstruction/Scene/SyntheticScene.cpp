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
#include "SurfaceReconstruction/Scene/FileNaming.h"
#include "SurfaceReconstruction/Scene/Samples.h"
#include "SurfaceReconstruction/Scene/SyntheticScene.h"
#include "SurfaceReconstruction/Scene/View/MVECameraIO.h"
#include "SurfaceReconstruction/Scene/View/View.h"
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
	mveIO.saveCamerasToFile(mViews);
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
	uint32 viewCount;
	if (!manager.get(viewCount, "viewCount"))
	{
		cerr << missingParameter << "uint32 viewCount = <count>\n";
		return false;
	}
	mViews.resize(viewCount);
	memset(mViews.data(), 0, sizeof(View *) * viewCount);
	
	// load view resolution
	uint32 width = 400;
	uint32 height = 300;
	if (!manager.get(width, "imageWidth") || !manager.get(height, "imageHeight"))
	{
		cerr << missingParameter << "uint32 imageWidth = <width>; or uint32 imageHeight = <height>;\n";
		return false;
	}
	mViewResolution.set(width, height);

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
	
	// unbalanced view distribution?
	mViewBalance = 1.0f;
	if (!manager.get(mViewBalance, "viewBalance"))
		cerr << missingParameter << "Real viewBalance = <balancing value>;\n";
	
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

View *SyntheticScene::createSyntheticView(const uint32 viewIdx)
{
	// free previous view for clean start
	delete mViews[viewIdx];
	mViews[viewIdx] = NULL;
	View *&view = mViews[viewIdx];

	// randomly distribute the cameras in the AABB
	RandomManager &randomManager = RandomManager::getSingleton();

	// view data
	const Vector3 up(0.0f, 1.0f, 0.0f);
	const Vector2 principalPoint(0.5f, 0.5f);
	const Real aspectRatio = (Real) mViewResolution[0] / (Real) mViewResolution[1];

	// AABB for view positions
	const Real centerX = (mAABB[0].x + mAABB[1].x) * 0.5f;
	Vector3 min = mAABB[0];
	Vector3 max = mAABB[1];

	// on which side of the mesh?
	const Real sideRandomness = randomManager.getUniform(0.0f, mViewBalance + 1.0f);
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

	// create view
	const Vector4 pHWS(pWS.x, pWS.y, pWS.z, 1.0f);
	const Vector3 lookAt = randomManager.getUniform(mMeshAABB[0], mMeshAABB[1]);

	view = new View(viewIdx, Quaternion(), pHWS, focalLength, principalPoint, aspectRatio);
	view->getCamera().lookAt(pWS, lookAt, up);
	return view;
}

void SyntheticScene::createAndSaveSamples()
{
	// valid mesh?
	if (!mGroundTruth)
		return;

	// create samples
	const uint32 viewsPerSample = 1; // todo: how to get more views per sample?
	mSamples = new Samples(viewsPerSample);

	// ray tracing preparation
	const uint32 pixelCount = mViewResolution.getElementCount();

	// create the scene and target depth map for the ray tracer
	RayTracer rayTracer;
	vector<Vector3> positionsWSMap(pixelCount);
	vector<Real> depthMap(pixelCount);
	vector<vector<uint32>> vertexNeighbors;
	vector<uint32> pixelToVertexIndices;
	vector<uint32> indices;
	uint32 validDepthCount;

	rayTracer.createStaticScene(*mGroundTruth, true);

	// create folder for images if necessary
	if (!Directory::createDirectory(Scene::getSingleton().getViewsFolder()))
		return;

	// create a depth map for each view
	const uint32 viewCount = (uint32) mViews.size();
	for (uint32 viewIdx = 0; viewIdx < viewCount; )
	{
		// get camera data
		const View &view = *createSyntheticView(viewIdx);
		const PinholeCamera &camera = view.getCamera();
		const Vector3 camPosWS = view.getPositionWS();

		// trace scene -> depth, position (in world space) & index map
		const bool good = fill(depthMap, positionsWSMap, rayTracer, validDepthCount, camera);
		if (0 == validDepthCount || !good)
			continue;

		// noise on surface samples
		addNoise(positionsWSMap, depthMap, camPosWS);
		
		// create view directory
		const string viewIDString = getIDString(viewIdx);
		const Path viewFolder = getViewFolder(viewIDString);
		if (!Directory::createDirectory(viewFolder))
			throw FileException("Could not create view directory!", viewFolder);
		
		//if (Math::EPSILON < mDepthMapNoise[0] || Math::EPSILON < mDepthMapNoise[1])
		//	saveColorImage(depthMap, viewIdx, true);

		// save depth map as undistorted color image and MVEI depth map
		const Path depthMapName = getRelativeImageFileName(viewIDString, FileNaming::IMAGE_TAG_DEPTH, 0, false);
		Image::saveAsMVEFloatImage(depthMapName, true, mViewResolution, depthMap.data(), false, false);
		saveColorImage(depthMap, viewIdx, false);

		// triangulate surface samples & create proper input samples
		addToSamples(vertexNeighbors, indices, pixelToVertexIndices,
			depthMapName, positionsWSMap, validDepthCount, viewIdx);

		++viewIdx;
	}
	
	mSamples->check();
	mSamples->computeParentViewCount();
	mSamples->computeAABB();
}

bool SyntheticScene::fill(vector<Real> &depthMap, vector<Vector3> &positionsWSMap,
	RayTracer &rayTracer, uint32 &validDepthCount, const PinholeCamera &camera)
{
	const Matrix3x3 HPSToNNRayDirWS = camera.computeHPSToNNRayDirWS(mViewResolution, true);
	const Vector3 camPosWS(camera.getPosition().x, camera.getPosition().y, camera.getPosition().z);
	const Real minDepthSq = mMinSampleDistance * mMinSampleDistance;
	const uint32 pixelCount = mViewResolution.getElementCount();

	// find surface points
	validDepthCount = 0;
	rayTracer.renderFromView(NULL, mViewResolution, camera, HPSToNNRayDirWS, true);

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
	const uint32 pixelCount = mViewResolution.getElementCount();
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

void SyntheticScene::saveColorImage(const vector<Real> &depthMap, const uint32 viewIdx, const bool withNoise) const
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

	// absolute file name via tag, view index and views folder
	const Path relativeFileName = getRelativeImageFileName(viewIdx, localName, 0, true);
	const Path viewsFolder = Scene::getSingleton().getViewsFolder();
	const Path absoluteName = Path::appendChild(viewsFolder, relativeFileName);

	// convert depth to gray values
	const uint32 pixelCount = mViewResolution.getElementCount();
	const uint32 channelCount = 3;
	uint8 *grayValues = new uint8[pixelCount * channelCount];

	// find depth extrema and correspondingly scale depths to gray values
	Real minDepth, maxDepth;
	DepthImage::findExtrema(minDepth, maxDepth, depthMap.data(), pixelCount);
	DepthImage::convertDepthsToColor(grayValues, depthMap.data(), pixelCount, channelCount, minDepth, maxDepth);

	// save the gray values to file
	File file(absoluteName, File::CREATE_WRITING, true);
	ImageManager::getSingleton().savePNG(file, grayValues, channelCount, mViewResolution);

	delete [] grayValues;
	grayValues = NULL;
}

void SyntheticScene::addToSamples(vector<vector<uint32>> &vertexNeighbors, vector<uint32> &indices, vector<uint32> &pixelToVertexIndices,
	const Path &depthMapName, const vector<Vector3> &positionsWSMap, const uint32 validDepthCount, const uint32 viewIdx)
{
	// reserve memory for the new samples
	const uint32 oldSampleCount = mSamples->getCount();
	const uint32 newSampleCount = validDepthCount + oldSampleCount;
	mSamples->reserve(newSampleCount);

	// camera data
	const View &view = *mViews[viewIdx];
	const PinholeCamera &camera = view.getCamera();

	// load depth map & triangulate it
	const Path colorImageName = getRelativeImageFileName(viewIdx, FileNaming::IMAGE_TAG_COLOR_S0, 0, true);
	const ColorImage *colorImage = ColorImage::request(colorImageName.getString(), colorImageName);

	DepthImage *depthMap = DepthImage::request(depthMapName.getString(), depthMapName);

	FlexibleMesh *viewMesh = depthMap->triangulate(pixelToVertexIndices, vertexNeighbors, indices, camera, colorImage);
	mViewMeshes.push_back(viewMesh);

	// add view mesh / triangulated depth map to all other samples
	//const uint32 pixelCount = mViewResolution.getElementCount();
	const Vector3 *colors = viewMesh->getColors();
	const Vector3 *normals = viewMesh->getNormals();
	const Vector3 *positions = viewMesh->getPositions();
	const Real *scales = viewMesh->getScales();
	const Real confidence = 1.0f; // todo: how to get reasonable confidence values?
	const uint32 vertexCount = viewMesh->getVertexCount();

	for (uint32 vertexIdx = 0, nextSampleIdx = oldSampleCount; vertexIdx < vertexCount; ++vertexIdx, ++nextSampleIdx)
	{
		mSamples->addSample();
		mSamples->setSample(nextSampleIdx,
			colors[vertexIdx], normals[vertexIdx], positions[vertexIdx], 
			confidence, scales[vertexIdx], &viewIdx);
	}
}

SyntheticScene::~SyntheticScene()
{
	clear();
}
