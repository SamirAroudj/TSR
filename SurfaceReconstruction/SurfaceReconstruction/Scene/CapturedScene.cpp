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

CapturedScene::CapturedScene(const Path &metaFileName, const vector<IReconstructorObserver *> &observers) :
	Scene(observers)
{
	// load what scene / what data to be loaded
	map<uint32, uint32> oldToNewViewIDs;
	vector<Path> plyCloudFileNames;	
	loadMetaData(plyCloudFileNames, metaFileName);


	// load views from some cameras file containing all cameras
	const Path camFile = Path::appendChild(mFolder, mRelativeCamerasFile);
	MVECameraIO loader(camFile);
	loader.loadFromCamerasFile(mViews, oldToNewViewIDs, mInvInputRotation, mInvInputTranslation);

	// load samples
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
