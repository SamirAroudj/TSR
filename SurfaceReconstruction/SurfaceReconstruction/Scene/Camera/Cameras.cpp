/*
 * Copyright (C) 2018 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */
#include "Math/MathHelper.h"
#include "Math/Vector3.h"
#include "Graphics/Viewport.h"
#include "Graphics/Camera3D.h"
#include "Platform/FailureHandling/FileException.h"
#include "Platform/Storage/File.h"
#include "Platform/Utilities/RandomManager.h"
#include "SurfaceReconstruction/Image/ColorImage.h"
#include "SurfaceReconstruction/Scene/Camera/Cameras.h"
#include "SurfaceReconstruction/Scene/CapturedScene.h"

using namespace FailureHandling;
using namespace Graphics;
using namespace Math;
using namespace std;
using namespace Storage;
using namespace SurfaceReconstruction;
using namespace Utilities;

const uint32 Cameras::FILE_VERSION = 1;
const uint32 Cameras::INVALID_ID = (uint32) -1;

Vector3 Cameras::getRay(const uint32 x, const uint32 y, const Matrix3x3 &pixelToRay)
{
	const Vector3 ray = Vector3((Real) x, (Real) y, 1.0f) * pixelToRay;
	return ray;
}

Cameras::~Cameras()
{
	clear();
	shrinkToFit();
}

void Cameras::shrinkToFit()
{
	// free memory
	mCameras.shrink_to_fit();
	mViewIDs.shrink_to_fit();
}

void Cameras::clear()
{
	// no cameras & IDs
	mCameras.clear();
	mViewIDs.clear();
}

void Cameras::addCamera(const uint32 viewID, const Vector3 AABB[2], const Real minSampleDistance, const Real maxSampleDistance,
	const Real meanFOV, const Real maxFOVDeviation, const Real pixelAspectRatio)
{
	// place the camera with some randomndess
	RandomManager &manager = RandomManager::getSingleton();
	const Vector3 AABBSize = (AABB[1] - AABB[0]);
	const Vector3 maxRndDiff = AABBSize * 0.1f;
	const Vector3 target = (AABB[0] + AABB[1]) * 0.5f + manager.getUniform(-maxRndDiff, maxRndDiff);
	const Vector3 position = manager.getUniform(AABB[0], AABB[1]);

	const Quaternion orientation(0.0f, 0.0f, 0.0f, 1.0f);
	const Vector2 principalPoint(0.5f, 0.5f);
	const Real focalLength = manager.getUniform(meanFOV - maxFOVDeviation, meanFOV + maxFOVDeviation); 
	const Real distortion[2] = { 0.0f, 0.0f };

	addCamera(viewID, orientation, position, focalLength, principalPoint, pixelAspectRatio, distortion);
	mCameras.back().lookAt(position, target, Vector3(0.0f, 1.0f, 0.0f));
}

void Cameras::addCamera(const uint32 viewID, const Quaternion &orientation, const Vector3 &position,
	const Real focalLength, const Vector2 &principalPoint, const Real pixelAspectRatio, const Real distortion[2])
{
	// create camera
	mCameras.resize(mCameras.size() + 1);
	mViewIDs.push_back(viewID);

	// set camera data
	PinholeCamera &camera = mCameras.back();
	camera.setOrientation(orientation);
	camera.setPosition(position.x, position.y, position.z);
	camera.setProjectionProperties(focalLength, pixelAspectRatio);
	camera.setPrincipalPoint(principalPoint);
	camera.setDistortion(distortion);
}

void Cameras::computeHWSToNNPS(Matrix4x4 &WSToPS, const ImgSize &resolution, const bool considerPixelCenterOffset, const uint32 cameraIdx) const
{
	// compute viewport transformation: device coordinates to pixel coordinates
	const PinholeCamera &cam = mCameras[cameraIdx];
	WSToPS = cam.computeWorldSpaceToPixelSpaceMatrix(resolution, considerPixelCenterOffset);
}
	
const ColorImage *Cameras::getColorImage(const string &tag, const uint32 &scale, const uint32 &cameraIdx) const
{
	return Scene::getColorImage(mViewIDs[cameraIdx], tag, scale);
}

const DepthImage *Cameras::getDepthImage(const string &tag, const uint32 &scale, const uint32 &cameraIdx) const
{
	return Scene::getDepthImage(mViewIDs[cameraIdx], tag, scale);
}

const Vector3 Cameras::getViewDirection(const uint32 cameraIdx) const
{
	const PinholeCamera &cam = mCameras[cameraIdx];
	const Matrix4x4 &V = cam.getViewMatrix();
	return -Vector3(V.m02, V.m12, V.m22);
}

void Cameras::loadFromFile(const Path &fileName)
{
	clear();
	cout << "Loading views from file \"" << fileName << "\"." << endl;
	
	// first store the data in rawCams for fast loading
	uint32 cameraCount = 0;
	CameraData *rawCams = NULL;

	// read data from file
	{
		// open file & read meta data
		File file(fileName, File::OPEN_READING, true, FILE_VERSION);
		file.read(&cameraCount, sizeof(uint32), sizeof(uint32), 1);

		// read each camera's data from file
		const uint32 eleSize = sizeof(CameraData);
		const uint64 byteCount = eleSize * cameraCount;
		CameraData *rawCams = new CameraData[cameraCount];
		file.read(rawCams, byteCount, eleSize, cameraCount);
	}
	
	// create every camera
	mCameras.reserve(cameraCount);
	mViewIDs.reserve(cameraCount);
	for (uint32 cameraIdx = 0; cameraIdx < cameraCount; ++cameraIdx)
		addCamera(rawCams[cameraIdx]);
	
	delete [] rawCams;
	rawCams = NULL;

	cout << "Loaded " << cameraCount << " cameras." << endl;
}

void Cameras::saveToFile(const Path &fileName) const
{
	// is there anything to save?
	if (mCameras.empty())
		return;

	// raw camera data for fas writing
	const uint32 cameraCount = getCount();
	CameraData *rawCams = new CameraData[cameraCount];

	// store every view
	for (uint32 cameraIdx = 0; cameraIdx < cameraCount; ++cameraIdx)
		rawCams[cameraIdx].set(mViewIDs[cameraIdx], mCameras[cameraIdx]);

	// write data to file
	{
		// create file
		File file(fileName, File::CREATE_WRITING, true, FILE_VERSION);

		// write version & camera count
		file.write(&cameraCount, sizeof(uint32), 1);
		file.write(rawCams, sizeof(CameraData), cameraCount);
	}

	// free resources
	delete [] rawCams;
	rawCams = NULL;
}
