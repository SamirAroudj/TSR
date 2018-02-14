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
	temp += viewID;
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
	mFSSFRefiner(NULL),
	mOccupancy(NULL),
	#ifdef PCS_REFINEMENT
		mPCSRefiner(NULL),
	#endif // PCS_REFINEMENT
	mSamples(NULL),
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
	if (!mCameras)
	{
		cout << "Stopping early. No views available for reconstruction ." << endl;
		return false;
	}
	if (!mSamples)
	{
		cout << "Stopping early. No samples available for reconstruction ." << endl;
		return false;
	}

	// view and sample count
	const uint32 cameraCount = mCameras->getCount();
	const uint32 sampleCount = mSamples->getCount();
	cout << "Starting reconstruction with " << cameraCount << " views and " << sampleCount << " samples.\n";

	// create results folder
	if (!Directory::createDirectory(getResultsFolder()))
	{
		cerr << "Could not create results folder!\n";
		cerr << "Folder: " << getResultsFolder() << endl;
		return false;
	}

	// check & save cameras
	if (0 == cameraCount)
	{
		cout << "Stopping early. No registered views available for reconstruction ." << endl;
		return false;
	}
	const Path beginning = getFileBeginning();
	mCameras->saveToFile(Path::extendLeafName(beginning, FileNaming::ENDING_CAMERAS));

	// check samples
	if (0 == sampleCount)
	{
		cout << "Stopping early. No surface samples available for reconstruction ." << endl;
		return false;
	}
	if (0 == mSamples->getViewConeCount())
	{
		cout << "Stopping early. No valid links between surface samples and views available for reconstruction." << endl;
		return false;
	}

	// save unfiltered / initial non-zero confidence surface samples
	if (!mOccupancy && !mTree)
		mSamples->saveToFile(beginning, true, true);
		
	if (!mTree)
	{
		// scene tree & reordered samples
		Samples *reorderedSamples = NULL;
		mTree = new Tree(reorderedSamples);

		// replace unordered samples
		delete mSamples;
		mSamples = reorderedSamples;
		reorderedSamples = NULL;
		mTree->getNodes().checkSamplesOrder(mTree->getRootScope());

		// save tree & samples
		mSamples->saveToFile(Path::extendLeafName(beginning, FileNaming::REORDERED_SAMPLES), true, true);
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

	//// further refinement?
	//if (!mPCSRefiner && mReconstructions[RECONSTRUCTION_VIA_SAMPLES])
	//{

	//	mPCSRefiner = new PCSRefiner(*mReconstructions[RECONSTRUCTION_VIA_SAMPLES]);
	//	mPCSRefiner->registerObserver(this);
	//	if (observers)
	//		mFSSFRefiner->registerObservers(*observers);
	//	refine(RECONSTRUCTION_VIA_PCS);
	//}

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
	const uint32 oldSampleCount = mSamples->getCount();
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
	mSamples->compact(sampleOffsets.data());

	// save data?
	if (!saveResults)
		return;
	
	cout << "Saving data." << endl;
	const Path beginning = getFileBeginning();

	mSamples->saveToFile(Path::extendLeafName(beginning, FileNaming::FILTERED_SAMPLES), true, true);
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
		mCameras = new Cameras(Path::extendLeafName(beginning, FileNaming::ENDING_CAMERAS));
	}
	catch (Exception &exception)
	{
		delete mCameras;
		mCameras = NULL;

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
	catch(Exception &exception)
	{
		cout << exception;
		cout << "There is no saved scene tree which could be loaded." << endl;
	}	

	// load the right sample set
	string ending(mTree ? FileNaming::REORDERED_SAMPLES : "");
	ending += FileNaming::ENDING_SAMPLES;
	const Path samplesPath = Path::extendLeafName(beginning, ending);
	mSamples = new Samples(samplesPath);

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

void Scene::swapSamples(const uint32 index0, const uint32 index1)
{
	if (mTree)
		throw Exception("Cannot change sample memory layout after scene tree construction" \
						"as the scene tree assumes a specific sample ordering according to its tree structure.");
	mSamples->swap(index0, index1);
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
		
	delete mCameras;
	delete mFSSFRefiner;
	delete mOccupancy;
	delete mTree;
	delete mSamples;
	
	mCameras = NULL;
	mFSSFRefiner = NULL;
	mOccupancy = NULL;
	mSamples = NULL;
	mTree = NULL;

	// free view meshes
	const uint32 viewMeshCount = (uint32) mViewMeshes.size();
	for (uint32 meshIdx = 0; meshIdx < viewMeshCount; ++meshIdx)
		delete mViewMeshes[meshIdx];
	mViewMeshes.clear();
	mViewMeshes.shrink_to_fit();

	// free volatile resources
	// free cached images
	Image::freeMemory();
}
