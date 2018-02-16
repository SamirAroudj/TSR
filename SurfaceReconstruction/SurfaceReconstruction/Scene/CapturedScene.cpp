/*
 * Copyright (C) 2018 by Author: Aroudj, Samir
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
#include "Platform/Utilities/HelperFunctions.h"
#include "Platform/Utilities/ParametersManager.h"
#include "Platform/Utilities/PlyFile.h"
#include "SurfaceReconstruction/Image/ColorImage.h"
#include "SurfaceReconstruction/Image/DepthImage.h"
#include "SurfaceReconstruction/Scene/Camera/Cameras.h"
#include "SurfaceReconstruction/Scene/Camera/MVECameraIO.h"
#include "SurfaceReconstruction/Scene/CapturedScene.h"
#include "SurfaceReconstruction/Scene/FileNaming.h"

using namespace Math;
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

	// load data
	loadCameras();
	loadImages(imageScales);
	if (mViewMeshes.empty())
		mSamples->loadClouds(plyCloudFileNames, mViewToCameraIndices, mInputOrientation, mInputOrigin);
	else
		mSamples->addSamplesFromMeshes(mViewMeshes);
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
		if (parts.size() < 4)
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
			const uint32 scale = Converter::to<uint32>(value);
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

void CapturedScene::loadCameras()
{
	// input transformation
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
			loader.loadFromCamerasFile(mCameras, mViewToCameraIndices, inverseRotation, translation);
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
	loader.loadFromMetaIniFiles(mCameras, mViewToCameraIndices, inverseRotation, translation);
}

void CapturedScene::loadImages(const vector<uint32> &imageScales)
{
	// load the corresponding images for all views at all scales
	const uint32 scaleCount = (uint32) imageScales.size();
	const uint32 cameraCount = mCameras.getCount();
	vector<vector<uint32>> vertexNeighbors;
	vector<uint32> indices;
	vector<uint32> pixelToVertexIndices;

	for (uint32 cameraIdx = 0; cameraIdx < cameraCount; ++cameraIdx)
	{
		// get camera data
		const PinholeCamera &camera = mCameras.getCamera(cameraIdx);
		const uint32 viewID = mCameras.getViewID(cameraIdx);

		for (uint32 scaleIdx = 0; scaleIdx < scaleCount; ++scaleIdx)
		{
			const uint32 &scale = imageScales[scaleIdx];

			// load corresponding images
			const char *colorImageTag = (0 == scale ? FileNaming::IMAGE_TAG_COLOR_S0 : FileNaming::IMAGE_TAG_COLOR);
			const ColorImage *colorImage = getColorImage(viewID, colorImageTag, scale);
			if (!colorImage)
				continue;

			const DepthImage *depthImage = getDepthImage(viewID, FileNaming::IMAGE_TAG_DEPTH, scale);
			if (!depthImage)
				continue;
			//depthImage->erode(5);

			//const ViewsImage *viewsImage = ???;
			//if (!viewsImage)
			//	continue;


			FlexibleMesh *mesh = depthImage->triangulate(pixelToVertexIndices, vertexNeighbors, indices, camera, colorImage);
			mViewMeshes.push_back(mesh);
		}
	}
}

CapturedScene::~CapturedScene()
{

}
