/*
 * Copyright (C) 2017 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */

#ifdef _DEBUG
	#include <iostream>
#endif // _DEBUG
#include "Platform/FailureHandling/FileCorruptionException.h"
#include "Platform/MagicConstants.h"
#include "Platform/ParametersManager.h"
#include "SurfaceReconstruction/Image/ColorImage.h"
#include "SurfaceReconstruction/Image/DepthImage.h"
#include "SurfaceReconstruction/Scene/CapturedScene.h"
#include "SurfaceReconstruction/Scene/MVECameraIO.h"
#include "SurfaceReconstruction/Scene/View.h"
#include "Utilities/HelperFunctions.h"
#include "Utilities/PlyFile.h"

using namespace Math;
using namespace Platform;
using namespace FailureHandling;
using namespace Graphics;
using namespace std;
using namespace Storage;
using namespace SurfaceReconstruction;
using namespace Utilities;

const char *CapturedScene::PARAMETER_NAME_IMAGE_SCALE = "imageScale";
const char *CapturedScene::PARAMETER_NAME_INPUT_ORIENTATION = "inputOrientation";
const char *CapturedScene::PARAMETER_NAME_INPUT_ORIGIN = "inputOrigin";
const char *CapturedScene::PARAMETER_NAME_PLY_FILE = "plyFile";

CapturedScene::CapturedScene(const Path &metaFileName, const vector<IReconstructorObserver *> &observers) :
	Scene(observers)
{
	// load what scene / what data to be loaded
	vector<Path> plyCloudFileNames;
	vector<uint32> imageScales;
	loadMetaData(plyCloudFileNames, imageScales, metaFileName);

	// load cameras & create views
	map<uint32, uint32> oldToNewViewIDs;
	loadCameras(oldToNewViewIDs);

	// load samples
	loadSampleClouds(plyCloudFileNames, oldToNewViewIDs);

	// load images
	loadImages(imageScales);
}

void CapturedScene::loadMetaData(vector<Path> &plyCloudFileNames, vector<uint32> &imageScales,
	const Path &fileName)
{
	// get parameters
	if (!getParameters(fileName))
		return;

	// load ply file names and scales of images to determine what images are loaded
	File file(fileName, File::OPEN_READING, false);
	string textLine;
	vector<string> parts;

	while (file.readTextLine(textLine))
	{
		// get & check parameter parts: type name = value;
		Utilities::split(parts, textLine, " \t");
		if (parts.size() != 4)
			continue;
		if (parts[2] != "=")
			continue;
		
		// some ply file or image tag?
		const string value = parts[3].substr(0, parts[3].find_last_of(";"));
		if (parts[0] == "string" && parts[1] == PARAMETER_NAME_PLY_FILE)
		{
			const Path path(value);
			plyCloudFileNames.push_back(path);
		}
		else if (parts[0] == "uint32" && parts[1] == PARAMETER_NAME_IMAGE_SCALE)
		{
			const uint32 scale = Utilities::convert<uint32>(value);
			imageScales.push_back(scale);
		}
		else
		{
			continue;
		}
	}
}

bool CapturedScene::getParameters(const Path &fileName)
{
	// load base parameters
	if (!Scene::getParameters(fileName))
		return false;

	// get parameters for captured scene
	ParametersManager &manager = ParametersManager::getSingleton();

	// load inverse rotation matrix for input data
	Vector3 angles(0.0f, 0.0f, 0.0f);
	if (manager.get(angles, PARAMETER_NAME_INPUT_ORIENTATION))
		mInputOrientation = Matrix3x3::createRotationFromExtrinsicAngles(angles.x, angles.y, angles.z);
	else
		mInputOrientation.setToIdentity();

	// load inverse translation for input data
	mInputOrigin.set(0.0f, 0.0f, 0.0f);
	manager.get(mInputOrigin, PARAMETER_NAME_INPUT_ORIGIN);
	return true;
}

void CapturedScene::loadCameras(map<uint32, uint32> &oldToNewViewIDs)
{
	Matrix3x3 inverseRotation(mInputOrientation);
	inverseRotation.transpose();
	const Vector3 translation = -mInputOrigin * inverseRotation;

	try
	{
		// load cameras from single MVE cameras file?
		if (!mRelativeCamerasFile.getString().empty())
		{
			// load views from some cameras file containing all cameras
			const Path camFile = Path::appendChild(mFolder, mRelativeCamerasFile);
			MVECameraIO loader(camFile);
			loader.loadFromCamerasFile(mViews, oldToNewViewIDs, inverseRotation, translation);
			return;
		}
	}
	catch (Exception &exception)
	{
		cerr << "Could not open cameras file!" << endl;
		exception;
	}

	// load views from folders within MVE views folder?
	MVECameraIO loader(getViewsFolder());
	loader.loadFromMetaIniFiles(mViews, oldToNewViewIDs, inverseRotation, translation);
}

void CapturedScene::loadSampleClouds(const vector<Path> &plyCloudFileNames, const map<uint32, uint32> &oldToNewViewIDs)
{
	// load each point cloud
	const uint32 fileCount = (uint32) plyCloudFileNames.size();
	for (uint32 fileIdx = 0; fileIdx < fileCount; ++fileIdx)
		loadSampleCloud(plyCloudFileNames[fileIdx]);

	// no samples?
	if (!mSamples)
		return;
	mSamples->check();

	// transform samples
	Matrix3x3 inverseRotation(mInputOrientation);
	inverseRotation.transpose();
	const Vector3 translation = -mInputOrigin * inverseRotation;
	const uint32 sampleCount = mSamples->getCount();

	#pragma omp parallel for
	for (int64 sampleIdx = 0; sampleIdx < sampleCount; ++sampleIdx)
		mSamples->transform((uint32) sampleIdx, inverseRotation, translation);

	// update and count parent view links
	mSamples->updateParentViews(oldToNewViewIDs);
	mSamples->computeParentViewCount();

	// world space AABB of all samples
	mSamples->computeAABB();
}

void CapturedScene::loadSampleCloud(const Path &plyCloudFileName)
{
	// open the file
	#ifdef _DEBUG
		cout << "\nStarting loading of ply sample cloud: " << plyCloudFileName << endl;
	#endif // _DEBUG

	PlyFile file(plyCloudFileName, File::OPEN_READING, true);

	// process ply header
	VerticesDescription verticesFormat;
	file.loadHeader(verticesFormat);

	// process ply body
	const uint32 existingSamplesCount = (mSamples ? mSamples->getCount() : 0);
	const uint32 maxNewSampleCount = ((uint32) -1) -1 - existingSamplesCount;
	const uint32 loadedSamplesCount = loadSamples(file, plyCloudFileName, verticesFormat, maxNewSampleCount);

	#ifdef _DEBUG
		cout << "\nFinished loading ply sample cloud, sample count: " << loadedSamplesCount << endl;
	#endif // _DEBUG
}

uint32 CapturedScene::loadSamples(PlyFile &file, const Path &fileName, const VerticesDescription &verticesFormat,	const uint32 maxNewSampleCount)
{
	cout << "Loading samples from " << fileName << "." << endl;
	
	// get access to vertex structure & samples
	const ElementsSyntax &types = verticesFormat.getTypeStructure();
	const ElementsSemantics &semantics = verticesFormat.getSemantics();
	const uint32 propertyCount = (uint32) types.size();

	// get #parents per sample & create samples container
	uint32 viewsPerSample = 0;
	for (uint32 propertyIdx = 0; propertyIdx < propertyCount; ++propertyIdx)
		if (semantics[propertyIdx] >= VerticesDescription::SEMANTIC_VIEWID0)
			++viewsPerSample;
	if (!mSamples)
		mSamples = new Samples(viewsPerSample);

	// reserve memory for the samples to be loaded
	const uint32 vertexCount = verticesFormat.getElementCount();
	const uint32 oldSampleCount = mSamples->getCount();
	const size_t newTotalSampleCount = oldSampleCount + vertexCount;
	mSamples->reserve(newTotalSampleCount);

	// read each sample / vertex
	uint32 sampleIdx = oldSampleCount;
	for (uint32 fileSampleIdx = 0;
		fileSampleIdx < vertexCount && fileSampleIdx < maxNewSampleCount;
		++fileSampleIdx)
	{
		if (!file.hasLeftData())
			throw FileCorruptionException("Could not read all vertices which were defined by the ply header.", fileName);

		// create sample
		mSamples->addSample();

		// load data for current sample
		for (uint32 propertyIdx = 0; propertyIdx < propertyCount; ++propertyIdx)
			readSampleProperty(file, sampleIdx, types[propertyIdx], (VerticesDescription::SEMANTICS) semantics[propertyIdx]);
		
		// ignore zero confidence samples
		if (Math::EPSILON >= mSamples->getConfidence(sampleIdx))
		{
			mSamples->popBackSample();
			continue;
		}

		// ignore samples with invalid scale
		if (Math::EPSILON >= mSamples->getScale(sampleIdx))
		{
			mSamples->popBackSample();
			continue;
		}

		++sampleIdx;
	}
	
	const uint32 sampleCount = sampleIdx - oldSampleCount;
	cout << "Loaded " << sampleCount << " samples from " << fileName << "." << endl;
	
	return sampleCount;
}

void CapturedScene::readSampleProperty(PlyFile &file, const uint32 sampleIdx,
	const ElementsDescription::TYPES type, const VerticesDescription::SEMANTICS semantic)
{
	// get sample data destinations
	Vector3 *color = mSamples->mColors.data() + sampleIdx;
	Vector3 *normal = mSamples->mNormals.data() + sampleIdx;
	Vector3 &position = mSamples->mPositions[sampleIdx];
	Real *confidence = mSamples->mConfidences.data() + sampleIdx;
	Real *scale = mSamples->mScales.data() + sampleIdx;
	uint32 *viewIDs = mSamples->mParentViews.data() + mSamples->mViewsPerSample * sampleIdx;

	file.readVertexProperty(color, normal, position, NULL, confidence, scale, viewIDs, type, semantic);
}

#include "SurfaceReconstruction/Geometry/FlexibleMesh.h"
void CapturedScene::loadImages(const vector<uint32> &imageScales)
{
	// load the corresponding images for all views at all scales
	const uint32 scaleCount = (uint32) imageScales.size();
	const uint32 viewCount = (uint32) mViews.size();
	vector<vector<uint32>> tempVertexNeighbors;
	vector<uint32> tempIndices;
	vector<uint32> tempPixelToVertexIndices;

	for (uint32 viewIdx = 0; viewIdx < viewCount; ++viewIdx)
	{
		for (uint32 scaleIdx = 0; scaleIdx < scaleCount; ++scaleIdx)
		{
			const uint32 &scale = imageScales[scaleIdx];

			// load corresponding images
			const string colorImageName = getLocalImageName((0 == scale ? "undistorted" : "undist"), scale, true);
			const string depthImageName = getLocalImageName("depth", scale, false);

			const ColorImage *colorImage = ColorImage::request(colorImageName, colorImageName);
			if (!colorImage)
				continue;

			const DepthImage *depthImage = DepthImage::request(depthImageName, depthImageName);
			if (!depthImage)
				continue;

			//const ViewsImage *viewsImage = ???;
			//if (!viewsImage)
			//	continue;

			//FlexibleMesh *triangulation = depthImage->triangulate(tempVertexNeighbors, tempIndices, tempPixelToVertexIndices,
			//	positionsWSMap, pixelToViewSpace, colorImage);
		}
	}
}

CapturedScene::~CapturedScene()
{

}
