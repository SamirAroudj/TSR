/*
 * Copyright (C) 2017 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */
#include "Platform/FailureHandling/Exception.h"
#include "Platform/FailureHandling/FileAccessException.h"
#include "Platform/Storage/File.h"
#include "Platform/MagicConstants.h"
#include "Platform/ParametersManager.h"
#include "Platform/Platform.h"
#include "Platform/Storage/Directory.h"
#include "SurfaceReconstruction/Image/Image.h"
#include "SurfaceReconstruction/Geometry/StaticMesh.h"
#include "SurfaceReconstruction/Refinement/FSSFRefiner.h"
#ifdef PCS_REFINEMENT
	#include "SurfaceReconstruction/Refinement/PCSRefiner.h"
#endif // PCS_REFINEMENT
#include "SurfaceReconstruction/Scene/Samples.h"
#include "SurfaceReconstruction/Scene/Scene.h"
#include "SurfaceReconstruction/Scene/Tree/Tree.h"
#include "SurfaceReconstruction/Scene/View.h"
#include "SurfaceReconstruction/SurfaceExtraction/DualMarchingCells.h"
#include "SurfaceReconstruction/SurfaceExtraction/Occupancy.h"

using namespace FailureHandling;
using namespace Math;
using namespace std;
using namespace Platform;
using namespace Storage;
using namespace SurfaceReconstruction;
using namespace Utilities;

const uint32 Scene::VIEWS_FILE_VERSION = 0;

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
	mImageTag("undistorted"),
	mRelativeCamerasFile("Cameras.txt")
{
	for (uint32 meshIdx = 0; meshIdx < RECONSTRUCTION_TYPE_COUNT; ++meshIdx)
		mReconstructions[meshIdx] = NULL;
	
	// required parameters
	const string isleSizeName = "Scene::minimumTriangleIsleSize";

	// get required parameters
	const ParametersManager &manager = ParametersManager::getSingleton();
	const bool loadedIsleSize = manager.get(mMinIsleSize, isleSizeName);
	if (loadedIsleSize)
		return;

	// error handling
	string message = "Scene: Could not load all parameters:\n";
	
	if (!loadedIsleSize)
	{
		message += isleSizeName;
		message += ", choosing 2500\n";
		mMinIsleSize = 2500;
	}

	cerr << message << endl;
}

Scene::~Scene()
{
	clear();

	// free volatile resources
	// free cached images
	Image::freeMemory();
}

bool Scene::reconstruct()
{
	// no views/samples for tree creation?
	if (mViews.empty())
	{
		cout << "Stopping early. No views available for reconstruction ." << endl;
		return false;
	}

	// valid scene?
	if (!mSamples)
	{
		cout << "Stopping early. No samples available for reconstruction ." << endl;
		return false;
	}

	// view and sample count
	const uint32 viewCount = (uint32) mViews.size();
	const uint32 sampleCount = mSamples->getCount();
	cout << "Starting reconstruction with " << mViews.size() << " views and " << sampleCount << " samples.\n";

	// create results folder
	if (!Directory::createDirectory(getResultsFolder()))
	{
		cerr << "Could not create results folder!\n";
		cerr << "Folder: " << getResultsFolder() << endl;
		return false;
	}

	// save mata data & views
	const Path beginning = getFileBeginning();
	saveViewsToFile(Path::extendLastName(beginning, ".Views"));

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
		mSamples->saveToFile(Path::extendLastName(beginning, "Samples"), true, true);
		
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
		mSamples->saveToFile(Path::extendLastName(beginning, "SamplesReordered"), true, true);
		mTree->saveToFiles(Path::extendLastName(beginning, ".Tree"));
	}

	// create scene tree and estimate free space?
	if (!mOccupancy)
	{
		// estimate free space / space occupancy scalar field
		mOccupancy = new Occupancy(mTree);

		// compute free space and find samples in free space
		//eraseSamplesInEmptySpace();
		//mSamples->saveToFile(beginning + "SamplesFiltered", true, true);

		// save filtered samples, free space & scene tree
		mOccupancy->saveToFile(Path::extendLastName(beginning, ".Occupancy"));
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

	//// refinement via photos?
	//if (!mPCSRefiner && mReconstructions[RECONSTRUCTION_VIA_SAMPLES])
	//{
	//	cout << "Mesh refinement using global photo consistency score." << endl;

	//	mPCSRefiner = new PCSRefiner(*mReconstructions[RECONSTRUCTION_VIA_SAMPLES]);
	//	mPCSRefiner->registerObserver(this);
	//	if (observers)
	//		mFSSFRefiner->registerObservers(*observers);
	//	refine(RECONSTRUCTION_VIA_PHOTOS);
	//}

	return true;
}

//void Scene::eraseSamplesInEmptySpace()
//{
//	cout << "Starting to erase free space outlier surface samples." << endl;
//	vector<uint32> sampleOffsets(mSamples->getCount() + 1);
//	vector<uint32> doomedSamples;
//
//	while (true)
//	{
//		// sample offsets for deletion of outliers in free space and corresponding compaction of samples
//		const uint32 oldSampleCount = (uint32) mSamples->getCount();
//		sampleOffsets.resize(oldSampleCount + 1);
//	
//		// find samples in free space & erase them from these nodes
//		mTree->eraseSamplesInNodes(sampleOffsets.data(), mOccupancy->getNodeStates(), oldSampleCount, Occupancy::NODE_FLAG_EMPTY);
//		const uint32 doomedSampleCount = sampleOffsets[oldSampleCount];
//		if (0 == doomedSampleCount)
//			return;
//	
//		// remove outliers from occupancy & samples
//		doomedSamples.resize(doomedSampleCount);
//		#pragma omp parallel for
//		for (int64 sampleIdx = 0; sampleIdx < oldSampleCount; ++sampleIdx)
//		{
//			const uint32 offset = sampleOffsets[sampleIdx];
//			if (offset != sampleOffsets[sampleIdx + 1])
//				doomedSamples[offset] = (uint32) sampleIdx;
//		}
//
//		mOccupancy->eraseSamples(doomedSamples.data(), doomedSampleCount);
//		mSamples->compact(sampleOffsets.data());
//
//		// output
//		const uint32 newSampleCount = mSamples->getCount();
//		const uint32 removalCount = sampleOffsets[oldSampleCount];
//		cout << "Finished removal iteration of free space outliers.\n";
//		cout << "Number of removed outliers: " << removalCount << ", new sample count: " << newSampleCount << "\n";
//	}
//
//	cout << flush;
//}

void Scene::checkSamples()
{
	const int64 sampleCount = mSamples->getCount();

	#pragma omp parallel for
	for (int64 sampleIdx = 0; sampleIdx < sampleCount; ++sampleIdx)
	{
		const Real &scale = mSamples->getScale((uint32) sampleIdx);
		assert(scale > 0.0f);
		if (0.0f >= scale)
			throw Exception("Invalid (non-positive) sample scale detected.");
	}
}

void Scene::refine(const ReconstructionType type)
{	
	// refine reconstruction
	if (RECONSTRUCTION_VIA_SAMPLES == type && mReconstructions[RECONSTRUCTION_VIA_OCCUPANCIES] && mFSSFRefiner)
		mFSSFRefiner->refine();
	#ifdef PCS_REFINEMENT
		else if (RECONSTRUCTION_VIA_PHOTOS == type && mReconstructions[RECONSTRUCTION_VIA_SAMPLES] && mPCSRefiner)
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

	mSamples->saveToFile(Path::extendLastName(beginning, "SamplesFiltered2"), true, true);
	mTree->saveToFiles(Path::extendLastName(beginning, ".Tree"));
	mOccupancy->saveToFile(Path::extendLastName(beginning, ".Occupancy"));
}

Path Scene::getFileBeginning() const
{
	const Path childPath("Scene");
	return Path::appendChild(getResultsFolder(), childPath);
}

const FlexibleMesh *Scene::getMostRefinedReconstruction() const
{
	const FlexibleMesh *mesh = getReconstruction(RECONSTRUCTION_VIA_PHOTOS);
	if (!mesh)
		mesh = getReconstruction(RECONSTRUCTION_VIA_SAMPLES);
	if (!mesh)
		mesh = getReconstruction(RECONSTRUCTION_VIA_OCCUPANCIES);

	return mesh;
}

Path Scene::getResultsFolder() const
{
	const Path resultsFolder("Results");
	return Path::appendChild(mFolder, resultsFolder);
}

Path Scene::getRelativeImageFileName(const uint32 viewID) const
{
	const string viewIDString = View::getIDString(viewID);
	return getRelativeImageFileName(viewIDString);
}

Path Scene::getRelativeImageFileName(const string &viewID) const
{
	// complete image file name
	const string relativPath = (("Views/view_" + viewID) + ".mve/");
	const string relativeFileName = relativPath + mImageTag;

	return relativeFileName;
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
		loadViewsFromFile(Path::extendLastName(beginning, ".Views"));
	}
	catch (Exception &exception)
	{
		mViews.clear();

		cerr << exception;
		cerr << "Exception! Could not load views." << endl;
		// todo log this?
		return;
	}
	
	// try to load the scene tree
	try
	{
		mTree = new Tree(Path::extendLastName(beginning, ".Tree"));
	}
	catch(Exception &exception)
	{
		cout << exception;
		cout << "There is no saved scene tree which could be loaded." << endl;
	}	

	// load the right sample set
	if (mTree)
		mSamples = new Samples(Path::extendLastName(beginning, "SamplesReordered.Samples"));
	else
		mSamples = new Samples(Path::extendLastName(beginning, "Samples.Samples"));
	
	// try to load the free space
	if (mTree)
	{
		try
		{
			mOccupancy = new Occupancy(Path::extendLastName(beginning, ".Occupancy"));
		}
		catch(Exception &exception)
		{
			cout << exception;
			cout << "There is no saved free space representation which could be loaded." << endl;
		}
	}

	//if (mOccupancy)
	//{
	//	try
	//	{
	//		Samples *samples = new Samples(beginning + "SamplesFiltered.Samples");
	//		delete mSamples;
	//		mSamples = samples;
	//	}
	//	catch (Exception &exception)
	//	{
	//		cout << exception;
	//		cout << "There are no saved with free space filtered samples which could be loaded." << endl;
	//	}
	//}

	// there might be a ground truth which can be loaded
	Path fileName;
	try
	{
		fileName = Path::extendLastName(beginning, "GroundTruth.Mesh");
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
			string localName = IReconstructorObserver::RECONSTRUCTION_TYPE_TEXTS[meshIdx];
			localName += ".Mesh";
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
	const char *text = RECONSTRUCTION_TYPE_TEXTS[type];

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
	const string missingParameter = "Missing parameter in synthetic scene description file: ";
	ParametersManager &manager = ParametersManager::getSingleton();
	manager.loadFromFile(fileName);

	// load scene folder
	string temp; 
	if (!manager.get(temp, "sceneFolder"))
	{
		cerr << missingParameter << "string sceneFolder = <folder>;\n";
		return false;
	}
	setRootFolder(temp);
	
	// load cameras file name
	if (!manager.get(temp, "relativeCamerasFileName"))
		cerr << missingParameter << "string relativeCamerasFileName = <fileName>;\n";
	mRelativeCamerasFile = temp;

	return true;
}

void Scene::setRootFolder(const Path &rootFolder)
{
	mFolder = rootFolder;
	Image::setPathToImages(mFolder);
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

void Scene::loadViewsFromFile(const Path &fileName)
{
	cout << "Loading views from file \"" << fileName << "\"." << endl;
	
	// open file
	File file(fileName, File::OPEN_READING, true, VIEWS_FILE_VERSION);

	// load #views & reserve memory
	uint32 viewCount;
	file.read(&viewCount, sizeof(uint32), sizeof(uint32), 1);
	mViews.resize(viewCount);
	memset(mViews.data(), 0, sizeof(View *) * viewCount);

	// load every view
	uint32 viewIdx;
	for (uint32 i = 0; i < viewCount; ++i)
	{
		// valid camera?
		file.read(&viewIdx, sizeof(uint32), sizeof(uint32), 1);
		if (viewIdx >= viewCount)
		{
			cout << "View " << i << " has the invalid ID " << viewIdx << "( view count = " << viewCount << ")." << endl;
			continue;
		}

		mViews[viewIdx] = new View(viewIdx, file, fileName);
	}
	
	cout << "Loaded " << viewCount << " views." << endl;
}

void Scene::saveViewsToFile(const Path &fileName) const
{
	// is there anything to save?
	if (mViews.empty())
		return;

	// create file
	File file(fileName, File::CREATE_WRITING, true, VIEWS_FILE_VERSION);

	// write version & view count
	const uint32 viewCount = (uint32) mViews.size();
	file.write(&viewCount, sizeof(uint32), 1);

	// write every view
	for (uint32 viewIdx = 0; viewIdx < viewCount; ++viewIdx)
	{
		const View *view = mViews[viewIdx];
		if (!view)
		{
			file.write(&View::INVALID_ID, sizeof(uint32), 1);
			continue;
		}

		const uint32 viewID = view->getID();
		file.write(&viewID, sizeof(uint32), 1);
		view->saveToFile(file);
	}
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

	delete mFSSFRefiner;
	delete mOccupancy;
	delete mTree;
	delete mSamples;

	mFSSFRefiner = NULL;
	mOccupancy = NULL;
	mSamples = NULL;
	mTree = NULL;

	// free views
	const uint32 viewCount = (uint32) mViews.size();
	for (uint32 viewIdx = 0; viewIdx < viewCount; ++viewIdx)
		delete mViews[viewIdx];
	mViews.clear();
	mViews.shrink_to_fit();
}
