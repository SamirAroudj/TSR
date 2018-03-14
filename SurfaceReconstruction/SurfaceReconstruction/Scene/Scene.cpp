/*
 * Copyright (C) 2018 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */
#include "Platform/FailureHandling/Exception.h"
#include "Platform/FailureHandling/FileAccessException.h"
#include "Platform/Storage/File.h"
#include "Platform/Storage/Directory.h"
#include "Platform/Utilities/ParametersManager.h"
#include "SurfaceReconstruction/Image/ColorImage.h"
#include "SurfaceReconstruction/Image/DepthImage.h"
#include "SurfaceReconstruction/Image/ViewsImage.h"
#include "SurfaceReconstruction/Geometry/StaticMesh.h"
#include "SurfaceReconstruction/Refinement/FSSFRefiner.h"
#ifdef PCS_REFINEMENT
	#include "SurfaceReconstruction/Refinement/PCSRefiner.h"
#endif // PCS_REFINEMENT
#include "SurfaceReconstruction/Scene/Camera/Cameras.h"
#include "SurfaceReconstruction/Scene/FileNaming.h"
#include "SurfaceReconstruction/Scene/Samples.h"
#include "SurfaceReconstruction/Scene/Scene.h"
#include "SurfaceReconstruction/Scene/Tree/Tree.h"
#include "SurfaceReconstruction/SurfaceExtraction/DualMarchingCells.h"
#include "SurfaceReconstruction/SurfaceExtraction/Occupancy.h"

using namespace FailureHandling;
using namespace Graphics;
using namespace Math;
using namespace std;
using namespace Storage;
using namespace SurfaceReconstruction;
using namespace Utilities;

// parameter names
const char *Scene::PARAMETER_NAME_RELATIVE_CAMERAS_FILE = "relativeCamerasFileName";
const char *Scene::PARAMETER_NAME_TRIANGLE_ISLE_SIZE_MINIMUM = "Scene::minimumTriangleIsleSize";
const char *Scene::PARAMETER_NAME_SCENE_FOLDER = "sceneFolder";

const uint32 Scene::PARAMETER_VALUE_REFINEMENT_VIA_PHOTOS_MESH_OUTPUT_FREQUENCY = 25;
const uint32 Scene::PARAMETER_VALUE_TRIANGLE_ISLE_SIZE_MINIMUM = 2500;

ColorImage *Scene::getColorImage(const uint32 &viewID, const string &tag, const uint32 &scale)
{
	const Path relativeFileName = getRelativeImageFileName(viewID, tag, scale, true);
	return ColorImage::request(relativeFileName.getString(), relativeFileName);
}

DepthImage *Scene::getDepthImage(const uint32 &viewID, const string &tag, const uint32 &scale)
{
	const Path relativeFileName = getRelativeImageFileName(viewID, tag, scale, false);
	return DepthImage::request(relativeFileName.getString(), relativeFileName);
}

Path Scene::getRelativeImageFileName(const uint32 &viewID, const string &tag, const uint32 scale, const bool colorImage)
{
	// complete image file name
	const Path viewFolder = Scene::getRelativeViewFolder(viewID);
	const string fileName = getLocalImageName(tag, scale, colorImage);

	return Path::appendChild(viewFolder, fileName);
}

Path Scene::getRelativeViewFolder(const uint32 &viewID)
{
	string temp(FileNaming::BEGINNING_VIEW_FOLDER);
	temp += Converter::from<uint32>(viewID, 4);
	temp += FileNaming::ENDING_VIEW_FOLDER;
	return Path(temp);
}

const string Scene::getLocalImageName(const string &tag, const uint32 scale, const bool colorImage)
{
	// special case: color image scale 0 (no downscaling)
	if (colorImage && 0 == scale)
		return (tag + FileNaming::ENDING_COLOR_IMAGE);

	// name = <tag>-L<scale><ending>, e.g., undist-L1.png
	const char *ending = (colorImage ? FileNaming::ENDING_COLOR_IMAGE : FileNaming::ENDING_MVE_IMAGE);
	char scaleString[4];
	snprintf(scaleString, 4, "-L%u", scale);
	string name = ((tag + scaleString) + ending);

	return name;
}

Scene::Scene(const Path &rootFolder, const Path &FSSFReconstruction,
	const vector<IReconstructorObserver *> &observers) :
	Scene(observers)
{
	loadFromFile(rootFolder, FSSFReconstruction);
}

Scene::Scene(const vector<IReconstructorObserver *> &observers) : 
	mGroundTruth(NULL),
	mCameras(),
	mFSSFRefiner(NULL),
	mOccupancy(NULL),
	#ifdef PCS_REFINEMENT
		mPCSRefiner(NULL),
	#endif // PCS_REFINEMENT
	mTree(NULL),
	mRefinerObservers(observers),
	mFolder(""),
	mRelativeCamerasFile(""),
	mMinIsleSize(PARAMETER_VALUE_TRIANGLE_ISLE_SIZE_MINIMUM)
{
	for (uint32 meshIdx = 0; meshIdx < RECONSTRUCTION_TYPE_COUNT; ++meshIdx)
		mReconstructions[meshIdx] = NULL;
	
	// required parameters
	// get required parameters
	const ParametersManager &manager = ParametersManager::getSingleton();
	const bool loadedIsleSize = manager.get(mMinIsleSize, PARAMETER_NAME_TRIANGLE_ISLE_SIZE_MINIMUM);
	if (loadedIsleSize)
		return;

	// error handling
	string message = "Scene: Could not load all parameters:\n";
	if (!loadedIsleSize)
		message += PARAMETER_NAME_TRIANGLE_ISLE_SIZE_MINIMUM;

	cerr << message << endl;
}

Scene::~Scene()
{
	clear();
}

bool Scene::reconstruct()
{
	// nothing to reconstruct without cameras or samples
	const uint32 cameraCount = mCameras.getCount();
	const uint32 sampleCount = mSamples.getCount();
	cout << "Starting reconstruction with " << cameraCount << " views and " << sampleCount << " samples.\n";
	if (0 == cameraCount || 0 == sampleCount)
	{
		cout << "Stopping early. Necessary input data for surface reconstruction is missing." << endl;
		return false;
	}

	// check links between samples and parent cameras
	if (0 == mSamples.getValidParentLinkCount())
	{
		cout << "Stopping early. No valid links between surface samples and views available for reconstruction." << endl;
		return false;
	}

	// create results folder
	if (!Directory::createDirectory(getResultsFolder()))
	{
		cerr << "Could not create results folder!\n";
		cerr << "Folder: " << getResultsFolder() << endl;
		return false;
	}

	// save cameras in binary format
	const Path beginning = getFileBeginning();
	mCameras.saveToFile(Path::extendLeafName(beginning, FileNaming::ENDING_CAMERAS));

	// save unfiltered / initial non-zero confidence surface samples
	if (!mOccupancy && !mTree)
		mSamples.saveToFile(beginning, true, true);
		
	if (!mTree)
	{
		// scene tree reorders samples in memory
		mTree = new Tree();
		mTree->getNodes().checkSamplesOrder(mTree->getRootScope());

		// save tree & samples
		mSamples.saveToFile(Path::extendLeafName(beginning, FileNaming::REORDERED_SAMPLES), true, true);
		mTree->saveToFiles(Path::extendLeafName(beginning, FileNaming::ENDING_OCTREE));
	}

	// create scene tree and estimate free space?
	if (!mOccupancy)
	{
		// estimate free space / space occupancy scalar field
		mOccupancy = new Occupancy(mTree);

		// save filtered samples, free space & scene tree
		mOccupancy->saveToFile(Path::extendLeafName(beginning, FileNaming::ENDING_OCCUPANCY));
	}

	if (!mReconstructions[RECONSTRUCTION_VIA_OCCUPANCIES])
	{
		// get & check surface mesh	
		const FlexibleMesh &crustSurface = mOccupancy->extractCrust().getSurface();
		if (0 == crustSurface.getVertexCount() || 0 == crustSurface.getIndexCount())
			return false;
		takeReconstructionFromOccupancy();
	}	

	// refinement via samples?
	if (!mFSSFRefiner)
	{
		cout << "Mesh refinement via input samples." << endl;
		createFSSFRefiner();
	}
	refine(RECONSTRUCTION_VIA_SAMPLES);

	return true;
}

void Scene::refine(const ReconstructionType type)
{	
	// refine reconstruction
	if (RECONSTRUCTION_VIA_SAMPLES == type && mReconstructions[RECONSTRUCTION_VIA_OCCUPANCIES] && mFSSFRefiner)
		mFSSFRefiner->refine();
	#ifdef PCS_REFINEMENT
		else if (RECONSTRUCTION_VIA_PCS == type && mReconstructions[RECONSTRUCTION_VIA_SAMPLES] && mPCSRefiner)
			mPCSRefiner->refine(REFINEMENT_VIA_PHOTOS_MESH_OUTPUT_FREQUENCY);
	#endif // PCS_REFINEMENT
	else
		return;
}

void Scene::takeOverReconstruction(FlexibleMesh *mesh, const ReconstructionType type)
{
	if (type >= RECONSTRUCTION_TYPE_COUNT)
	{
		assert(false);
		return;
	}

	delete mReconstructions[type];
	mReconstructions[type] = mesh;
}

bool Scene::onNewReconstruction(FlexibleMesh *mesh,
	const uint32 iteration, const string &inputText, const ReconstructionType type, const bool responsible)
{
	if (type >= RECONSTRUCTION_TYPE_COUNT)
	{
		assert(false);
		return false;
	}

	takeOverReconstruction(mesh, type);
	if (!mReconstructions[type])
		return false;
	mReconstructions[type]->computeNormalsWeightedByAngles();

	// create file name extra part for result identification
	const uint32 BUFFER_SIZE = 100;
	char buffer[BUFFER_SIZE];
	snprintf(buffer, BUFFER_SIZE, "%.4u", iteration);

	string fileNameExtraPart = buffer;
	fileNameExtraPart += inputText;

	// save to files
	saveReconstructionToFiles(type, fileNameExtraPart, true, false);
	
	// check responsibility for deletion
	if (!responsible)
	{
		mReconstructions[type] = NULL;
		return false;
	}

	return true;
}

void Scene::eraseSamples(const bool *inliers, const bool saveResults)
{
	// sampleOffsets & outliers vector
	const uint32 oldSampleCount = mSamples.getCount();
	vector<uint32> sampleOffsets(oldSampleCount + 1);
	vector<uint32> outliers;
	outliers.reserve(oldSampleCount / 3);

	sampleOffsets[0] = 0;
	for (uint32 i = 0; i < oldSampleCount; ++i)
	{
		uint32 &offset = sampleOffsets[i + 1];
		offset = sampleOffsets[i];
		if (inliers[i])
			continue;

		outliers.push_back(i);
		++offset;
	}
	
	// update sample-dependent structures
	cout << "Erasing " << outliers.size() << " outliers of " << oldSampleCount << " samples." << endl;
	
	mOccupancy->eraseSamples(outliers.data(), (uint32) outliers.size());
	mTree->eraseSamples(sampleOffsets.data());
	mSamples.compact(sampleOffsets.data());

	// save data?
	if (!saveResults)
		return;
	
	cout << "Saving data." << endl;
	const Path beginning = getFileBeginning();

	mSamples.saveToFile(Path::extendLeafName(beginning, FileNaming::FILTERED_SAMPLES), true, true);
	mTree->saveToFiles(Path::extendLeafName(beginning, FileNaming::ENDING_OCTREE));
	mOccupancy->saveToFile(Path::extendLeafName(beginning, FileNaming::ENDING_OCCUPANCY));
}

Path Scene::getFileBeginning() const
{
	const Path childPath(FileNaming::BEGINNING_RESULTS);
	return Path::appendChild(getResultsFolder(), childPath);
}

const FlexibleMesh *Scene::getMostRefinedReconstruction() const
{
	const FlexibleMesh *mesh = getReconstruction(RECONSTRUCTION_VIA_PCS);
	if (!mesh)
		mesh = getReconstruction(RECONSTRUCTION_VIA_SAMPLES);
	if (!mesh)
		mesh = getReconstruction(RECONSTRUCTION_VIA_OCCUPANCIES);

	return mesh;
}

Path Scene::getResultsFolder() const
{
	const Path resultsFolder(FileNaming::RESULTS_FOLDER);
	return Path::appendChild(mFolder, resultsFolder);
}

void Scene::loadFromFile(const Path &rootFolder, const Path &FSSFReconstruction)
{
	clear();
	setRootFolder(rootFolder);

	// every scene file address begins like this 
	const Path beginning = getFileBeginning();

	// there must be a valid meta, views and samples file otherwise the scene is useless
	try
	{
		mCameras.loadFromFile(Path::extendLeafName(beginning, FileNaming::ENDING_CAMERAS));
	}
	catch (Exception &exception)
	{
		cerr << exception;
		cerr << "Exception! Could not load views." << endl;
		// todo log this?
		return;
	}
	
	// try to load the scene tree
	try
	{
		mTree = new Tree(Path::extendLeafName(beginning, FileNaming::ENDING_OCTREE));
	}
	catch (Exception &exception)
	{
		cout << exception;
		cout << "There is no saved scene tree which could be loaded." << endl;
	}	

	// load the right sample set
	string ending(mTree ? FileNaming::REORDERED_SAMPLES : "");
	ending += FileNaming::ENDING_SAMPLES;
	const Path samplesPath = Path::extendLeafName(beginning, ending);
	try
	{
		mSamples.loadFromFile(samplesPath);;
	}
	catch (Exception &exception)
	{
		cout << exception;
		cout << "There are no surface samples which could be loaded." << endl;
	}

	// try to load the free space
	if (mTree)
	{
		try
		{
			mOccupancy = new Occupancy(Path::extendLeafName(beginning, FileNaming::ENDING_OCCUPANCY));
		}
		catch(Exception &exception)
		{
			cout << exception;
			cout << "There is no saved free space representation which could be loaded." << endl;
		}
	}

	// there might be a ground truth which can be loaded
	Path fileName;
	try
	{
		fileName = Path::extendLeafName(beginning, "GroundTruth");
		fileName = fileName.extendLeafName(FileNaming::ENDING_MESH);
		mGroundTruth = new StaticMesh(fileName);
	}
	catch (FileException &exception)
	{
		delete mGroundTruth;
		mGroundTruth = NULL;

		exception;
		cout << "There is no ground truth mesh (" << fileName << ") which can be loaded." << endl;
	}

	// is there a previous reconstruction that can be loaded?
	for (uint32 meshIdx = 0; meshIdx < RECONSTRUCTION_TYPE_COUNT; ++meshIdx)
	{
		if (RECONSTRUCTION_VIA_SAMPLES == meshIdx && !FSSFReconstruction.getString().empty())
		{
			fileName = FSSFReconstruction;
		}
		else
		{
			string localName = FileNaming::RECONSTRUCTED_MESHES[meshIdx];
			localName += FileNaming::ENDING_MESH;
			fileName = Path::appendChild(getResultsFolder(), localName);
		}

		try
		{
			mReconstructions[meshIdx] = new FlexibleMesh(fileName);
		}
		catch(Exception &exception)
		{
			exception;
			cout << "Could not load reconstruction: " << fileName << "\n";
			cout << flush;
		}
	}

	if (mOccupancy && !mReconstructions[RECONSTRUCTION_VIA_OCCUPANCIES])
		takeReconstructionFromOccupancy();

	createFSSFRefiner();
}

void Scene::takeReconstructionFromOccupancy()
{
	// create crust?
	const DualMarchingCells *crust = mOccupancy->getCrust();
	if (!crust)
		crust = &mOccupancy->extractCrust();

	// check crust
	if (0 == crust->getSurface().getTriangleCount())
	{
		cout << "Empty crust." << endl;
		return;
	}
	mOccupancy->outputEdgeConflicts();

	// take over
	const ReconstructionType type = RECONSTRUCTION_VIA_OCCUPANCIES;
	const char *text = FileNaming::RECONSTRUCTED_MESHES[type];

	FlexibleMesh *mesh = new FlexibleMesh(crust->getSurface());
	takeOverReconstruction(mesh, type);
	saveReconstructionToFiles(type, text, true, true);
}

void Scene::createFSSFRefiner()
{
	// try to use the latest reconstructed mesh
	const FlexibleMesh *mesh = mReconstructions[RECONSTRUCTION_VIA_SAMPLES];
	if (!mesh)
		mesh = mReconstructions[RECONSTRUCTION_VIA_OCCUPANCIES];
	if (!mesh)
		return;
	if (0 == mesh->getTriangleCount())
		return;

	delete mFSSFRefiner;
	mFSSFRefiner = new FSSFRefiner(*mesh);

	mFSSFRefiner->registerObservers(mRefinerObservers);
	mFSSFRefiner->registerObserver(this);
}


void Scene::loadDepthMeshes(vector<vector<uint32> *> &cameraIndices, vector<uint32> &camerasPerSamples, const vector<uint32> &imageScales)
{
	// load the corresponding images for all views at all scales
	const uint32 scaleCount = (uint32) imageScales.size();
	const uint32 cameraCount = mCameras.getCount();
	vector<vector<uint32>> vertexNeighbors;
	vector<uint32> indices;
	vector<uint32> pixelToVertexIndices;

	// reserve memory for visibility information
	const uint32 imageCount = cameraCount * scaleCount;
	cameraIndices.reserve(imageCount);
	camerasPerSamples.reserve(imageCount);

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

			DepthImage *depthImage = getDepthImage(viewID, FileNaming::IMAGE_TAG_DEPTH, scale);
			if (!depthImage)
				continue;

			depthImage->setDepthConvention(mCameras.getCamera(cameraIdx), DepthImage::DEPTH_ALONG_RAY);
			FlexibleMesh *mesh = depthImage->triangulate(pixelToVertexIndices, vertexNeighbors, indices, camera, colorImage);
			mDepthMeshes.push_back(mesh);
			
			loadCameraIndices(cameraIndices, camerasPerSamples, pixelToVertexIndices, mesh->getVertexCount(), cameraIdx, scale);
		}
	}
}

void Scene::loadCameraIndices(vector<vector<uint32> *> &cameraIndices, vector<uint32> &camerasPerSamples,
	const vector<uint32> &pixelToVertexIndices, const uint32 &vertexCount, const uint32 &cameraIdx, const uint32 &scale) const
{
	cameraIndices.resize(cameraIndices.size() + 1, NULL);
	camerasPerSamples.resize(cameraIndices.size(), 0);
	uint32 &camerasPerSample = camerasPerSamples.back();

	// either views image available -> new camera indices vector for mesh for cmaera cameraIdx or missing image -> invalid pointer instead of vector
	try
	{
		// load views image
		const uint32 viewID = mCameras.getViewID(cameraIdx);
		const Path fileName = getRelativeImageFileName(viewID, FileNaming::IMAGE_TAG_VIEWS, scale, false);
		ViewsImage *viewsImage = ViewsImage::request(fileName.getCString(), fileName);

		// reserve memory & update camerasPerSamples
		camerasPerSample = viewsImage->getChannelCount();
		cameraIndices.back() = new vector<uint32>(vertexCount * camerasPerSample, Cameras::INVALID_ID);

		// set view indices
		const uint32 pixelCount = (uint32) pixelToVertexIndices.size();
		for (uint32 pixelIdx = 0; pixelIdx < pixelCount; ++pixelIdx)
		{
			const uint32 vertexIdx = pixelToVertexIndices[pixelIdx]; 
			if (Triangle::INVALID_INDEX == vertexIdx)
				continue;

			// set camera indices for the current vertex
			uint32 *sampleCameras = cameraIndices.back()->data() + camerasPerSample * vertexIdx;
			for (uint32 channelIdx = 0; channelIdx < camerasPerSample; ++channelIdx)
			{
				const uint32 &parentViewID = viewsImage->get(pixelIdx, channelIdx);
				if (Cameras::INVALID_ID != parentViewID)
					sampleCameras[channelIdx] = mViewToCameraIndices[parentViewID];
			}
		}
	}
	catch (FileException &exception)
	{
		camerasPerSample = 1;
		cameraIndices.back() = new vector<uint32>(vertexCount, cameraIdx);

		cout << exception.getMessage() << " " << exception.getFileName() << "\n";
		cout << "Could not load views IDs for camera " << cameraIdx << " and scale " << scale << ".\n";
		cout << "Using only reference view for visibility constraints.\n" << flush;
	}
}

bool Scene::getParameters(const Path &fileName)
{
	const string missingParameter = "Missing parameter in scene description file: ";
	ParametersManager &manager = ParametersManager::getSingleton();
	manager.loadFromFile(fileName);

	// load scene folder
	string temp; 
	if (!manager.get(temp, PARAMETER_NAME_SCENE_FOLDER))
	{
		cerr << missingParameter << "string " << PARAMETER_NAME_SCENE_FOLDER << " = <folder>;\n";
		return false;
	}
	setRootFolder(temp);
	
	// load where cameras are
	if (manager.get(temp, PARAMETER_NAME_RELATIVE_CAMERAS_FILE))
		mRelativeCamerasFile = temp;

	return true;
}

void Scene::setRootFolder(const Path &rootFolder)
{
	// scene root
	mFolder = rootFolder;

	// set & create views folder
	const Path viewsFolder = getViewsFolder();
	if (!Directory::createDirectory(viewsFolder, false))
		throw FileException("Could not create directory!", viewsFolder);
	Image::setPathToImages(viewsFolder);
}

void Scene::saveReconstructionToFiles(const ReconstructionType type, const string &localName, const bool saveAsPly, const bool saveAsMesh) const
{
	if (type >= RECONSTRUCTION_TYPE_COUNT)
	{
		assert(false);
		return;
	}

	// choose mesh
	const FlexibleMesh *mesh = mReconstructions[type];
	if (!mesh)
		return;
	
	// file name without type
	const Path name = Path::appendChild(getResultsFolder(), localName);
	mesh->saveToFile(name, saveAsPly, saveAsMesh);
}

void Scene::clear()
{
	// free resources
	// free meshes
	delete mGroundTruth;
	mGroundTruth = NULL;

	for (uint32 i = 0; i < RECONSTRUCTION_TYPE_COUNT; ++i)
	{
		delete mReconstructions[i];
		mReconstructions[i] = NULL;
	}

	// free assistant data structures
	#ifdef PCS_REFINEMENT
		delete mPCSRefiner;
		mPCSRefiner = NULL;
	#endif // PCS_REFINEMENT
		
	delete mFSSFRefiner;
	delete mOccupancy;
	delete mTree;
	
	mFSSFRefiner = NULL;
	mOccupancy = NULL;
	mTree = NULL;

	// free triangulated depth maps / depth meshes
	const uint32 meshCount = (uint32) mDepthMeshes.size();
	for (uint32 meshIdx = 0; meshIdx < meshCount; ++meshIdx)
		delete mDepthMeshes[meshIdx];
	mDepthMeshes.clear();
	mDepthMeshes.shrink_to_fit();

	// release samples & cameras
	mSamples.clear();
	mSamples.shrinkToFit();
	mCameras.clear();
	mCameras.shrinkToFit();

	// free volatile resources
	// free cached images
	Image::freeMemory();
}
