/*
 * Copyright (C) 2017 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */

#include <fstream>
#include "CollisionDetection/CollisionDetection.h"
#include "Graphics/Color.h"
#include "Graphics/ImageManager.h"
#include "Platform/FailureHandling/Exception.h"
#include "Platform/FailureHandling/FileException.h"
#include "Platform/MagicConstants.h"
#include "Platform/ParametersManager.h"
#include "Platform/Platform.h"
#include "Platform/Storage/Directory.h"
#include "SurfaceReconstruction/Geometry/FlexibleMesh.h"
#include "SurfaceReconstruction/Geometry/RayTracer.h"
#include "SurfaceReconstruction/Geometry/StaticMesh.h"
#include "SurfaceReconstruction/Image/Image.h"
#include "SurfaceReconstruction/Refinement/MeshRefiner.h"
#include "SurfaceReconstruction/Scene/Samples.h"
#include "SurfaceReconstruction/Scene/SyntheticScene.h"
#include "SurfaceReconstruction/Scene/View.h"
#include "tinyxml2.h"
#include "Utilities/PlyFile.h"
#include "Utilities/RandomManager.h"
#include "Utilities/SVGLoader.h"

using namespace CollisionDetection;
using namespace FailureHandling;
using namespace Graphics;
using namespace Math;
using namespace std;
using namespace Platform;
using namespace Storage;
using namespace SurfaceReconstruction;
using namespace tinyxml2;
using namespace Utilities;

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

	const Path camFile = Path::appendChild(getResultsFolder(), mRelativeCamerasFile);
	saveCamerasToFile(camFile);
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
	const Vector2 principlePoint(0.5f, 0.5f);
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

	view = new View(viewIdx, Quaternion(), pHWS, focalLength, principlePoint, aspectRatio);
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
	vector<Vector3> hitsMap(pixelCount);
	vector<Real> depthMap(pixelCount);
	vector<uint32> pixelToHitMap(pixelCount);
	vector<vector<uint32>> vertexNeighbors;
	vector<uint32> hitToVertexLinks;
	vector<uint32> indices;
	uint32 hitCount;

	rayTracer.createStaticScene(*mGroundTruth, true);

	// create folder for image if necessary
	const Path &resultsFolder = getResultsFolder();
	const Path imagesFolder = Path::appendChild(resultsFolder, "/Images");
	if (!Directory::createDirectory(imagesFolder))
		return;

	// create a depth map for each view
	const uint32 viewCount = (uint32) mViews.size();
	for (uint32 viewIdx = 0; viewIdx < viewCount; )
	{
		// get camera data
		const View &view = *createSyntheticView(viewIdx);
		const PinholeCamera &camera = view.getCamera();
		const Vector3 camPosWS = view.getPositionWS();

		// trace scene -> depth, hit & index map
		const bool good = fill(depthMap, hitsMap, pixelToHitMap, rayTracer, hitCount, camera);
		if (0 == hitCount || !good)
			continue;

		// noise on surface samples
		//saveToFile(depthMap, viewIdx, false);
		addNoise(hitsMap, depthMap, camPosWS);

		//if (Math::EPSILON < mDepthMapNoise[0] || Math::EPSILON < mDepthMapNoise[1])
		//	saveToFile(depthMap, viewIdx, true);

		// triangulate surface samples & create proper input samples
		addToSamples(vertexNeighbors, indices, hitToVertexLinks,
			hitsMap, depthMap, pixelToHitMap, hitCount, viewIdx);
		
		// view<number>.mvei
		string localName = "view";
		localName += view.getIDString(viewIdx);
		localName += ".mvei";
		
		const Path fileName = Path::appendChild(imagesFolder, localName);
		Image::saveAsMVEFloatImage(fileName, mViewResolution, depthMap.data(), false, false);

		++viewIdx;
	}
	
	mSamples->computeParentViewCount();
	mSamples->computeAABB();
	checkSamples();
}

bool SyntheticScene::fill(vector<Real> &depthMap, vector<Vector3> &hitsMap, vector<uint32> &pixelToHitMap,
	RayTracer &rayTracer, uint32 &hitCount, const PinholeCamera &camera)
{
	const Matrix3x3 HPSToNNRayDirWS = camera.computeHPSToNNRayDirWS(mViewResolution, true);
	const Vector3 camPosWS(camera.getPosition().x, camera.getPosition().y, camera.getPosition().z);
	const Real minDepthSq = mMinSampleDistance * mMinSampleDistance;
	const uint32 pixelCount = mViewResolution.getElementCount();

	// find surface points
	hitCount = 0;
	rayTracer.renderFromView(NULL, mViewResolution, camera, HPSToNNRayDirWS, true);

	// fill depth & index map
	for (uint32 pixelIdx = 0; pixelIdx < pixelCount; ++pixelIdx)
	{
		// initial invalid value
		depthMap[pixelIdx] = -REAL_MAX;

		// valid hit?
		if (!rayTracer.getHitValidity(pixelIdx))
			continue;

		// compute & store hit position & depth depth
		Vector3 &hitPosWS = hitsMap[pixelIdx];
		hitPosWS = rayTracer.getHit(NULL, pixelIdx);

		const Real lengthSq = (hitPosWS - camPosWS).getLengthSquared();
		if (lengthSq < minDepthSq)
			return false;

		const Real depth = sqrtr(lengthSq);
		depthMap[pixelIdx] = depth;
		pixelToHitMap[pixelIdx] = hitCount;
		++hitCount;
	}

	return true;
}

void SyntheticScene::addNoise(vector<Vector3> &hitsMap, vector<Real> &depthMap, const Vector3 &camPosWS)
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
		Vector3 &hitPosWS = hitsMap[pixelIdx];
		Real &depth = depthMap[pixelIdx];
		if (-REAL_MAX == depth)
			continue;

		// add noise to depth
		const Real noise = (sqrtr(depth) * noiseFactor);
		depth += noise;  

		// add noise to hitPosWS
		Vector3 direction = hitPosWS - camPosWS;
		direction.normalize();
		hitPosWS = camPosWS + direction * depth;
	}
}

void SyntheticScene::saveToFile(const vector<Real> &depthMap, const uint32 viewIdx, const bool withNoise) const
{
	// add view index
	char buffer[File::READING_BUFFER_SIZE];
	uint32 length = snprintf(buffer, File::READING_BUFFER_SIZE, "View%.4u", viewIdx);

	// file name with noise appendix?
	if (withNoise)
		length += snprintf(buffer + length, File::READING_BUFFER_SIZE - length, "Mean" REAL_IT "StdDev" REAL_IT, mDepthMapNoise[0], mDepthMapNoise[1]);
	snprintf(buffer + length, File::READING_BUFFER_SIZE - length, ".png");

	// find depth extrema
	const uint32 pixelCount = mViewResolution.getElementCount();
	Real minDepth, maxDepth;
	MeshRefiner::findDepthExtrema(minDepth, maxDepth, depthMap.data(), pixelCount);

	// convert depth to gray values
	const uint32 channelCount = 3;
	uint8 *pixels = new uint8[pixelCount * channelCount];
	ImageManager::convertDepthMap(pixels, depthMap.data(), pixelCount, channelCount, minDepth, maxDepth);

	// save the converted pixels to file
	const Path folder = Path::appendChild(getResultsFolder(), "Images");
	const Path fileName = Path::appendChild(folder, buffer);
	File file(fileName, File::CREATE_WRITING, true);
	ImageManager::getSingleton().savePNG(file, pixels, channelCount, mViewResolution);

	delete [] pixels;
	pixels = NULL;
}

void SyntheticScene::addToSamples(vector<vector<uint32>> &vertexNeighbors, vector<uint32> &indices, vector<uint32> &hitToVertexLinks,
	const vector<Vector3> &hitsMap, const vector<Real> &depthMap, const vector<uint32> &pixelToHitMap, const uint32 hitCount, const uint32 viewIdx)
{
	// reserve memory for the new samples
	const uint32 oldSampleCount = mSamples->getCount();
	const uint32 newSampleCount = hitCount + oldSampleCount;
	mSamples->reserve(newSampleCount);

	// K^1 matrix (inverse kamera calibration)
	const PinholeCamera &camera = mViews[viewIdx]->getCamera();
	const Matrix3x3 invProj = camera.computeInverseProjectionMatrix();
	const Matrix3x3 invViewPort = camera.computeInverseViewportMatrix(mViewResolution, true);
	const Matrix3x3 pixelToViewSpace = invViewPort * invProj;

	// create a depth map triangulation to compute samples' properties
	FlexibleMesh *triangulation = triangulate(vertexNeighbors, indices, hitToVertexLinks,
		hitsMap, depthMap, pixelToHitMap, hitCount, pixelToViewSpace);
	triangulation->computeNormalsWeightedByAngles();
	FlexibleMesh::computeVertexScales(triangulation->getScales(), vertexNeighbors.data(), triangulation->getPositions(), triangulation->getVertexCount());

	// add triangulation to all other samples
	//const uint32 pixelCount = mViewResolution.getElementCount();
	const Vector3 *colors = triangulation->getColors();
	const Vector3 *normals = triangulation->getNormals();
	const Vector3 *positions = triangulation->getPositions();
	const Real *scales = triangulation->getScales();
	const Real confidence = 1.0f; // todo: how to get reasonable confidence values?
	const uint32 vertexCount = triangulation->getVertexCount();

	for (uint32 vertexIdx = 0, nextSampleIdx = oldSampleCount; vertexIdx < vertexCount; ++vertexIdx, ++nextSampleIdx)
	{
		mSamples->addSample();
		mSamples->setSample(nextSampleIdx,
			colors[vertexIdx], normals[vertexIdx], positions[vertexIdx], 
			confidence, scales[vertexIdx], &viewIdx);
	}

	delete triangulation;
	triangulation = NULL;
}

FlexibleMesh *SyntheticScene::triangulate(vector<vector<uint32>> &vertexNeighbors, vector<uint32> &indices, vector<uint32> &hitToVertexLinks,
	const vector<Vector3> &hitsMap, const vector<Real> &depthMap, const vector<uint32> &pixelToHitMap, const uint32 hitCount,
	const Matrix3x3 &pixelToViewSpace) const
{
	// reserve memory & clear buffers
	indices.clear();
	hitToVertexLinks.resize(hitCount);
	memset(hitToVertexLinks.data(), -1, sizeof(uint32) * hitCount);

	// index buffer, vertexNeighbors for the depth map
	uint32 vertexCount = 0;
	for (uint32 y = 0; y < mViewResolution[1] - 1; ++y)
		for (uint32 x = 0; x < mViewResolution[0] - 1; ++x)
			vertexCount = triangulateBlock(indices, hitToVertexLinks, vertexCount,
				hitsMap, depthMap, pixelToHitMap, x, y, pixelToViewSpace);

	// compute neighbors
	vertexNeighbors.resize(vertexCount);
	for (uint32 vertexIdx = 0; vertexIdx < vertexCount; ++vertexIdx)
		vertexNeighbors[vertexIdx].clear();

	// create triangulation
	const uint32 indexCount = (uint32) indices.size();
	FlexibleMesh *triangulation = new FlexibleMesh(vertexCount, indexCount);	

	FlexibleMesh::findVertexNeighbors(vertexNeighbors.data(), indices.data(), indexCount);
	triangulation->setIndices(indices.data(), (uint32) indices.size());	

	// set triangulation mesh vertices
	const Vector3 color(0.5f, 0.5f, 0.5f);
	const uint32 pixelCount = mViewResolution.getElementCount();

	for (uint32 pixelIdx = 0; pixelIdx < pixelCount; ++pixelIdx)
	{
		// valid depth / valid hit?
		if (-REAL_MAX == depthMap[pixelIdx])
			continue;

		// triangulated hit?
		const uint32 hitIdx = pixelToHitMap[pixelIdx];
		const uint32 vertexIdx = hitToVertexLinks[hitIdx];
		if (-1 == vertexIdx)
			continue;
		
		const Vector3 &posWS = hitsMap[pixelIdx];
		triangulation->setColor(color, vertexIdx);
		triangulation->setPosition(posWS, vertexIdx);
	}

	return triangulation;
}

uint32 SyntheticScene::triangulateBlock(vector<uint32> &indices, vector<uint32> &hitToVertexLinks, uint32 vertexCount,
	const vector<Vector3> &hitsMap, const vector<Real> &depthMap, const vector<uint32> &pixelToHitMap,
	const uint32 x, const uint32 y, const Matrix3x3 &pixelToViewSpace) const
{
	// indices of 4 different, reasonable triangles within 2x2 rectangle of vertices
	const uint32 blockTriangles[4][3] =
	{
		{ 0, 2, 1 },
		{ 0, 3, 1 },
		{ 0, 2, 3 },
		{ 1, 2, 3 }
	};

	//const uint32 blockCoords[4][2] = 
	//{
	//	{ x, y },
	//	{ x + 1, y},
	//	{ x, y + 1},
	//	{ x + 1, y + 1}
	//};

	const uint32 width = mViewResolution[0];
	const uint32 height = mViewResolution[1];
	const uint32 pixelIdx = width * y + x;
	const bool inImage[4] =
	{
		(x < width && y < height),
		(x + 1 < width && y < height),
		(x < width && y + 1 < height),
		(x + 1 < width && y + 1 < height)
	};

	// get depths
	const Real blockDepths[4] =
	{
		(inImage[0] ? depthMap[pixelIdx] : -REAL_MAX),
		(inImage[1] ? depthMap[pixelIdx + 1] : -REAL_MAX),
		(inImage[2] ? depthMap[pixelIdx + width] : -REAL_MAX),
		(inImage[3] ? depthMap[pixelIdx + width + 1] : -REAL_MAX)
	};

	// mask-based encoding of available depths
	uint32 mask = 0;
	uint32 availablePixels = 0;
	for (uint32 localDepthIdx = 0; localDepthIdx < 4; ++localDepthIdx)
	{
		if (-REAL_MAX == blockDepths[localDepthIdx])
			continue;

		mask |= 1 << localDepthIdx;
		++availablePixels;
	}

	// 3 or more vertices / at least 1 triangle?
	if (availablePixels < 3)
		return vertexCount;

	// find proper triangles to be created within the block
	uint32 triangleIndices[2] = { Triangle::INVALID_IDX, Triangle::INVALID_IDX };
	switch (mask)
	{
		case 7:  triangleIndices[0] = 0; break;
		case 11: triangleIndices[0] = 1; break;
		case 13: triangleIndices[0] = 2; break;
		case 14: triangleIndices[0] = 3; break;
		case 15:
		{
			// triangle with smaller depth
			const Real diagonalLength0 = fabsr(blockDepths[0] - blockDepths[3]);
			const Real diagonalLength1 = fabsr(blockDepths[1] - blockDepths[2]);

			if (diagonalLength0 < diagonalLength1)
			{
				triangleIndices[0] = 1;
				triangleIndices[1] = 2;
			}
			else
			{
				triangleIndices[0] = 0;
				triangleIndices[1] = 3;
			}
			break;
		}

		default:
			return vertexCount;
	}

	// compute 3D (world space) pixel footprints
	Real footprints[4];
	for (uint32 localDepthIdx = 0; localDepthIdx < 4; ++localDepthIdx)
	{
		if (-REAL_MAX == blockDepths[localDepthIdx])
			continue;

		const uint32 neighborX = x + (localDepthIdx % 2);
		const uint32 neighborY = y + (localDepthIdx / 2);
		const Real &depth = blockDepths[localDepthIdx];
		const Vector3 vVS = Vector3((Real) neighborX, (Real) neighborY, 1.0f) * pixelToViewSpace;

		footprints[localDepthIdx] = pixelToViewSpace.m00 * depth / vVS.getLength();
	}

	// try to avoid to triangulate depth discontinuities
	for (uint32 triangleIdxIdx = 0; triangleIdxIdx < 2; ++triangleIdxIdx)
	{
		// valid triangle?
		const uint32 triangleIdx = triangleIndices[triangleIdxIdx];
		if (Triangle::isInvalidIndex(triangleIdx))
			continue;

		// invalidate triangle if it looks like going over a depth discontinuity
		const uint32 *triangle = blockTriangles[triangleIdx];
		for (uint32 edgeIdx = 0; edgeIdx < 3 && !Triangle::isInvalidIndex(triangleIndices[triangleIdxIdx]); ++edgeIdx)
		{
			const uint32 edgeEnd0 = edgeIdx;
			const uint32 edgeEnd1 = (edgeIdx + 1) % 3;
			if (isDepthDiscontinuity(footprints, blockDepths, triangle[edgeEnd0], triangle[edgeEnd1]))
				triangleIndices[triangleIdxIdx] = Triangle::INVALID_IDX;
		}
	}

	// finally, create the triangles!
	for (uint32 triangleIdxIdx = 0; triangleIdxIdx < 2; ++triangleIdxIdx)
	{
		// valid triangle?
		const uint32 triangleIdx = triangleIndices[triangleIdxIdx];
		if (Triangle::isInvalidIndex(triangleIdx))// || /*0 == triangleIdx ||*/ 1 == triangleIdx || 2 == triangleIdx || 3 == triangleIdx)
			continue;

		// add indices for the triangle
		const uint32 *triangle = blockTriangles[triangleIdx];
		for (uint32 cornerIdx = 0; cornerIdx < 3; ++cornerIdx)
		{
			const uint32 dX = triangle[cornerIdx] % 2;
			const uint32 dY = triangle[cornerIdx] / 2;
			const uint32 hitIdx = pixelToHitMap[pixelIdx + dY * width + dX];

			// get vertex index for hit (only keep triangulated hits)
			uint32 &vertexIdx = hitToVertexLinks[hitIdx];
			if (-1 == vertexIdx)
			{
				vertexIdx = vertexCount;
				++vertexCount;
			}

			indices.push_back(vertexIdx);
		}
	}

	return vertexCount;
}

bool SyntheticScene::isDepthDiscontinuity(const Real footprints[4], const Real blockDepths[4], const uint32 v0, const uint32 v1)
{
	// index of vertex closer (min) and further away from camera (max)
	const uint32 closerVertex = (blockDepths[v0] < blockDepths[v1] ? v0 : v1);
	const uint32 furtherVertex = (blockDepths[v0] < blockDepths[v1] ? v1 : v0);

	// allow larger depth difference for diagonal edge
	Real ddFactor = DEPTH_DIFFERENCE_FACTOR;
	if (3 == v0 + v1)
		ddFactor *= Math::SQRT_TWO;

	// discontinuity depending on whether the edge reaches far away from the camera (relative to the footprint size of the closer pixel / vertex)
	const Real depthDifference = blockDepths[furtherVertex] - blockDepths[closerVertex];
	const bool discontinuity = (depthDifference > footprints[closerVertex] * ddFactor);
	return discontinuity;
}

void SyntheticScene::saveCamerasToFile(const Path &fileName) const
{
	// view count
	const uint32 viewCount = (uint32) mViews.size();
    cout << "SyntheticScene:: Writing synthetic camera infos of " << viewCount << " cameras: " << fileName << "...\n";

    ofstream out(fileName.getString(), ios::binary);
    if (!out.good())
		throw FileException("Could not create a file to save data of synthetic cameras.",fileName);

	// file header
    out << "MVE camera infos 1.0\n";
    out << "camera_count = " << viewCount << "\n";

    // write all cameras infos to the file
	Matrix3x3 rotation;

    for (size_t viewIdx = 0; viewIdx < viewCount; ++viewIdx)
    {
        const View &view = *mViews[viewIdx];
		const PinholeCamera &camera = view.getCamera();
		const Vector4 pHWS = camera.getPosition();
		const Vector3 pWS(pHWS.x, pHWS.y, pHWS.z);
		const Vector2 &principlePoint = camera.getPrinciplePoint();

		// new camera with inverted looking direction for MVE convention
		const Vector3 viewDirection = view.getViewDirection();
		const Vector3 targetWS = pWS - viewDirection;
		PinholeCamera mveCamera(camera);
		mveCamera.lookAt(pWS, targetWS, Vector3(0.0f, -1.0f, 0.0f));

		// 3x3 rotation matrix
		const Matrix4x4 viewMatrix = mveCamera.getViewMatrix();
		for (uint32 rowIdx = 0; rowIdx < 3; ++rowIdx)
			for (uint32 columnIdx = 0; columnIdx < 3; ++columnIdx)
				rotation(rowIdx, columnIdx) = viewMatrix(rowIdx, columnIdx);
		
		//// mve rotation conventions: flip y and z -> y down and z 
		//rotation.m10 = -rotation.m10;
		//rotation.m11 = -rotation.m11;
		//rotation.m12 = -rotation.m12;
		//rotation.m20 = -rotation.m20;
		//rotation.m21 = -rotation.m21;
		//rotation.m22 = -rotation.m22;

        // identifiers
        out << "id = " << viewIdx << "\n";
        out << "name = " << "view" << viewIdx << "\n";

        // intrinsics
        out << "focal_length = " << mveCamera.getFocalLength() << "\n";
        out << "principle_point = " << principlePoint.x << " " << principlePoint.y << "\n";
        out << "pixel_aspect_ratio = " << 1.0f << "\n";
        out << "camera_distortion = " << 0.0f << " " << 0.0f << "\n";

		// extrinsics
        // store translation vector
		const Vector3 translation = -(pWS * rotation);// MVE directly uses this translation vector
        out << "translation = " << translation[AXIS_X] << " " << translation[AXIS_Y] << " " << translation[AXIS_Z] << "\n";
		
		// store rotation with MVE conventions
		rotation.transpose(); // rotation in MVE format (they use column vectors and left matrices, e.g. R * t = rotatedT instead of this projects t * R = rotatedT)
		
        out << "rotation = ";
        out << rotation(0, 0) << " " << rotation(0, 1) << " " << rotation(0, 2) << " ";
        out << rotation(1, 0) << " " << rotation(1, 1) << " " << rotation(1, 2) << " ";
        out << rotation(2, 0) << " " << rotation(2, 1) << " " << rotation(2, 2) << "\n";
    }
}

SyntheticScene::~SyntheticScene()
{
	clear();
}
