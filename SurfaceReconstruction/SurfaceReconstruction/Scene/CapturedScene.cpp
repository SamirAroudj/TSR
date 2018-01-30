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
#include "Math/MathHelper.h"
#include "Math/Matrix4x4.h"
#include "Math/Quaternion.h"
#include "Math/Vector2.h"
#include "Platform/FailureHandling/FileCorruptionException.h"
#include "Platform/MagicConstants.h"
#include "Platform/ParametersManager.h"
#include "SurfaceReconstruction/Scene/CapturedScene.h"
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

CapturedScene::CapturedScene(const Path &metaFileName, const vector<IReconstructorObserver *> &observers) :
	Scene(observers)
{
	// load what scene / what data to be loaded
	vector<Path> plyCloudFileNames;	
	loadMetaData(plyCloudFileNames, metaFileName);

	// load views & samples
	const Path camFile = Path::appendChild(mFolder, mRelativeCamerasFile);
	map<uint32, uint32> oldToNewViewIDs;

	loadViews(oldToNewViewIDs, camFile);
	loadSampleClouds(plyCloudFileNames, oldToNewViewIDs);
	mSamples->computeAABB();
}

void CapturedScene::loadMetaData(vector<Path> &plyCloudFileNames, const Path &fileName)
{
	// get parameters
	if (!getParameters(fileName))
		return;

	// load ply file names
	File file(fileName, File::OPEN_READING, false);
	string textLine;
	vector<string> parts;

	while (file.readTextLine(textLine))
	{
		Utilities::split(parts, textLine, " \t");
		if ("string" != parts[0] || "plyFile" != parts[1] || "=" != parts[2])
			continue;

		const string fileName = parts[3].substr(0, parts[3].find_last_of(";"));
		const Path path(fileName);
		plyCloudFileNames.push_back(path);
	}
}

bool CapturedScene::getParameters(const Path &fileName)
{
	// load base parameters
	if (!Scene::getParameters(fileName))
		return false;

	// get parameters for captured scene
	ParametersManager &manager = ParametersManager::getSingleton();
	if (!manager.get(mImageTag, "imageTag"))
		mImageTag = "undistorted.png";

	// load inverse rotation matrix for input data
	bool angles = false;
	Real alpha = 0.0f;
	Real beta = 0.0f;
	Real gamma = 0.0f;

	angles |= manager.get(alpha, "extrinsicAngleX");
	angles |= manager.get(beta,  "extrinsicAngleY");
	angles |= manager.get(gamma, "extrinsicAngleZ");
	if (angles)
	{
		mInvInputRotation = Matrix3x3::createRotationFromExtrinsicAngles(alpha, beta, gamma);
		mInvInputRotation.transpose();
	}
	else
	{
		mInvInputRotation.setToIdentity();
	}

	// load inverse translation for input data
	mInvInputTranslation.set(0.0f, 0.0f, 0.0f);
	manager.get(mInvInputTranslation.x, "translationX");
	manager.get(mInvInputTranslation.y, "translationY");
	manager.get(mInvInputTranslation.z, "translationZ");

	return true;
}

void CapturedScene::loadViews(map<uint32, uint32> &oldToNewViewIDs, const Path &camerasFileName)
{
	// contents of a MVE cameras file
	const char *header				= "MVE camera infos 1.0\n";
	const char *cameraCountFormat	= "camera_count = %d\n";
	const char *viewIDFormat		= "id = %d\n";
	const char *cameraNameFormat0	= "name = ";
	const char *cameraNameFormat1	= "\n";
	const char *focalLengthFormat	= "focal_length = " REAL_IT "\n";
	const char *ppFormat			= "principle_point = " REAL_IT " " REAL_IT "\n";
	const char *aspectRatioFormat	= "pixel_aspect_ratio = " REAL_IT "\n";
	const char *distortionFormat	= "camera_distortion = " REAL_IT " " REAL_IT "\n";
	const char *translationFormat	= "translation = " REAL_IT " " REAL_IT " " REAL_IT "\n";
	const char *rotationFormat		= "rotation = " REAL_IT " " REAL_IT " " REAL_IT " " REAL_IT " " REAL_IT " " REAL_IT " " REAL_IT " " REAL_IT " " REAL_IT "\n";

	// variables to store read data
	string line;
	Matrix3x3 rotation;
	Quaternion orientation;
	Vector3 camPosition;
	Vector2 principlePoint;
	Vector2 distortion;
	Real aspectRatio;
	Real focalLength;
	uint32 viewID;
	uint32 cameraCount;
	char nameBuffer[File::READING_BUFFER_SIZE];

	oldToNewViewIDs.clear();

	// open the file
	File file(camerasFileName, File::OPEN_READING, false);

	// read header
	file.readTextLine(line);
	if (string::npos == line.find(header))
		throw FileCorruptionException("First line of MVE cameras file is not in correct format.", camerasFileName);

	if (1 != file.scanf(cameraCountFormat, &cameraCount))
		throw FileCorruptionException("Camera count isn't provided in correct MVE cameras file format.", camerasFileName);
	mViews.reserve(cameraCount);

	// read each camera
	uint32 newViewID = 0;
	while (file.hasLeftData())
	{
		
		if (1 != file.scanf(viewIDFormat, &viewID))
			throw FileCorruptionException("View ID isn't provided in correct MVE cameras file format.", camerasFileName);
		if (oldToNewViewIDs.end() != oldToNewViewIDs.find(viewID))
			throw FileCorruptionException("Duplicate view ID. All view IDs in MVE cameras file must be unique!", camerasFileName);
		oldToNewViewIDs[viewID] = newViewID;	

		if (1 != file.scanfString(cameraNameFormat0, cameraNameFormat1, nameBuffer, File::READING_BUFFER_SIZE))
			throw FileCorruptionException("Camera name isn't provided in correct MVE cameras file format.", camerasFileName);

		if (1 != file.scanf(focalLengthFormat, &focalLength))
			throw FileCorruptionException("Camera focal length isn't provided in correct MVE cameras file format.", camerasFileName);

		if (2 != file.scanf(ppFormat, &principlePoint.x, &principlePoint.y))
			throw FileCorruptionException("Principle point isn't provided in correct MVE cameras file format.", camerasFileName);

		if (1 != file.scanf(aspectRatioFormat, &aspectRatio))
			throw FileCorruptionException("Camera focal length isn't provided in correct MVE cameras file format.", camerasFileName);

		if (2 != file.scanf(distortionFormat, &distortion.x, &distortion.y))
			throw FileCorruptionException("Camera distortion parameters aren't provided in correct MVE cameras file format.", camerasFileName);

		if (3 != file.scanf(translationFormat, &camPosition.x, &camPosition.y, &camPosition.z))
			throw FileCorruptionException("Camera translation vector isn't provided in correct MVE cameras file format.", camerasFileName);

		Real *p = (Real *) rotation.values;
		if (9 != file.scanf(rotationFormat, p, p + 1, p + 2, p + 3, p + 4, p + 5, p + 6, p + 7, p + 8, p + 9))
			throw FileCorruptionException("Camera rotation matrix isn't provided in correct MVE cameras file format.", camerasFileName);

		// adapt camera data to this project's conventions
		{
			// translation = -R^-1 * cam position -> position = -R * translation (loaded rotation is R as it is (R^-1)^t = R)
			const Vector3 temp = -camPosition * rotation;
			camPosition = temp;

			// here a camera looks along negative z-direction in order to have a local camera frame which is right handed and
			// the camera has x pointing right and y up
			rotation.m10 = -rotation.m10;
			rotation.m11 = -rotation.m11;
			rotation.m12 = -rotation.m12;
			rotation.m20 = -rotation.m20;
			rotation.m21 = -rotation.m21;
			rotation.m22 = -rotation.m22;
		}

		// check parameters
		if (EPSILON >= focalLength)
			continue;

		// transform camera into the coordinate system dataBasis
		orientation = Math::createQuaternionFromMatrix(rotation * mInvInputRotation);
		camPosition = camPosition * mInvInputRotation - mInvInputTranslation;

		// create view
		const Vector4 temp(camPosition.x, camPosition.y, camPosition.z, 1.0f);
		View *newView = new View(newViewID, orientation, temp, focalLength, principlePoint, aspectRatio);
		mViews.push_back(newView);
		++newViewID;
		
		// debug output
		#ifdef _DEBUG
			cout << "\nRead & created view " << viewID << " " << nameBuffer;
		#endif // _DEBUG
	}

	#ifdef _DEBUG
		cout << endl;
	#endif // _DEBUG
}

void CapturedScene::loadSampleClouds(const vector<Path> &plyCloudFileNames, const map<uint32, uint32> &oldToNewViewIDs)
{
	// load each point cloud
	const uint32 fileCount = (uint32) plyCloudFileNames.size();
	for (uint32 fileIdx = 0; fileIdx < fileCount; ++fileIdx)
		loadSampleCloud(plyCloudFileNames[fileIdx]);

	// transform samples
	const uint32 sampleCount = mSamples->getCount();
	#pragma omp parallel for
	for (int64 sampleIdx = 0; sampleIdx < sampleCount; ++sampleIdx)
		mSamples->transform((uint32) sampleIdx, mInvInputRotation, -mInvInputTranslation);

	mSamples->updateParentViews(oldToNewViewIDs);
	mSamples->computeParentViewCount();
	checkSamples();
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

CapturedScene::~CapturedScene()
{

}
