/*
 * Copyright (C) 2018 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */
#include "App/TSR.h"
#include "Git.h"
#include "Graphics/GraphicsManager.h"
#include "Graphics/ImageManager.h"
#include "Graphics/MagicConstants.h"
#include "Math/MathCore.h"
#include "Math/MathHelper.h"
#include "Platform/Input/InputManager.h"
#include "Platform/Timing/ApplicationTimer.h"
#include "Platform/Utilities/ParametersManager.h"
#include "SurfaceReconstruction/Geometry/FlexibleMesh.h"
#include "SurfaceReconstruction/Geometry/StaticMesh.h"
#include "SurfaceReconstruction/Refinement/FSSFRefiner.h"
#ifdef PCS_REFINEMENT
	#include "SurfaceReconstruction/Refinement/PCSRefiner.h"
#endif // PCS_REFINEMENT
#include "SurfaceReconstruction/Scene/CapturedScene.h"
#include "SurfaceReconstruction/Scene/Samples.h"
#include "SurfaceReconstruction/Scene/Tree/DualCells.h"
#include "SurfaceReconstruction/Scene/Tree/Nodes.h"
#include "SurfaceReconstruction/Scene/Tree/Tree.h"
#include "SurfaceReconstruction/Scene/SyntheticScene.h"

using namespace std;
using namespace Graphics;
using namespace Input;
using namespace Math;
using namespace Platform;
using namespace ResourceManagement;
using namespace SurfaceReconstruction;
using namespace Timing;
using namespace Utilities;

TSR::TSR
(
	#ifdef _WINDOWS
		HINSTANCE applicationHandle, 
	#endif // _WINDOWS
	const vector<string> &arguments
) :
	Platform::Application
	(
		#ifdef _WINDOWS
			applicationHandle, 
		#endif // _WINDOWS
		arguments.front()
	),
	mCamera(NULL),
	mRenderer(NULL),
	mScene(NULL),
	mScale(1.0f),
	mSceneIndex(0)
{
	// output git data
	cout << Git::getStatus() << endl;

	// check command line arguments
	uint32 argumentCount = (uint32) arguments.size();
	if (2 != argumentCount && 3 != argumentCount)
	{
		// log this
		cerr << "Invalid argument count!\n";
		cerr << "Required argument variants:\n";
		cerr << "Variant 0: <path/program config file> <path>InputData.txt/InputDataSynthetic.txt\n";
		cerr << "Variant 1: <path/program config file> <pathToPreviousRunRootFolder> [optional: relative name of FSSF reconstruction result as new start (name without path to results folder)]\n";
		cerr << flush;

		mRunning = false;
		return;
	}

	// create image manager & rendering objects
	ImageManager *imageManager = new ImageManager();

	if (Platform::Window::exists())
	{
		const uint32 colorResolution = 32;
		GraphicsManager *graphicsManager = new GraphicsManager(colorResolution, ::BACK_BUFFER_CLEAR_COLOR);

		// create camera
		Platform::Window &window = Platform::Window::getSingleton();
		mCamera = new Camera3D(Math::HALF_PI, window.getAspectRatio(), 0.1f, 25.0f);
		mCamera->setAsActiveCamera();
		resetCamera();

		// create mesh renderer
		mMeshRenderer = new MeshRenderer();
	}

	// create scene from MVE, synthetic or previous run data?
	if (string::npos != arguments[1].find("InputData.txt"))
		createNewScene(SCENE_CREATION_FROM_MVE_DATA, arguments);
	else if (string::npos != arguments[1].find("InputDataSynthetic.txt"))
		createNewScene(SCENE_CREATION_SYNTHETIC, arguments);
	else
		createNewScene(SCENE_CREATION_FROM_INTERMEDIATE_RESULTS, arguments);

	// if there is no window then reconstruct stuff but do not execute the main loop
	if (!Platform::Window::exists())
	{
		mScene->reconstruct();
		mRunning = false;
	}
}

TSR::~TSR()
{
	// free scene data
	delete mScene;

	// free visualization stuff
	delete mCamera;
	delete mMeshRenderer;
	delete mRenderer;
	
	// free managers
	if (ImageManager::exists())
		delete ImageManager::getSingletonPointer();
	if (GraphicsManager::exists())
		delete GraphicsManager::getSingletonPointer();
}

void TSR::render()
{
	GraphicsManager::getSingleton().clearBackAndDepthStencilBuffer();
		mRenderer->render(mScale);
		mMeshRenderer->renderUploadedMeshes();
	GraphicsManager::getSingleton().presentBackBuffer();
}

void TSR::update()
{
	// quit application?
	if (!Platform::Window::exists() || InputManager::getSingleton().getKeyboard().isKeyDown(KEY_ESCAPE))
	{
		mRunning = false;
		return;
	}
	
	mRenderer->update();
	
	if (controlScene())
		return;

	controlCamera();
	controlRenderer();
}

void TSR::controlCamera()
{
	// variable definitions
	Vector3 relativeCameraMovement;
	const Real deltaTime = ApplicationTimer::getSingleton().getDeltaTime();
	const Mouse &mouse = InputManager::getSingleton().getMouse();
	const Keyboard &keyboard = InputManager::getSingleton().getKeyboard();

	// move camera
	if (keyboard.isKeyDown(KEY_UP))
		relativeCameraMovement.z += 1.0f;
	else if (keyboard.isKeyDown(KEY_DOWN))
		relativeCameraMovement.z -= 1.0f;
	if (keyboard.isKeyDown(KEY_LEFT))
		relativeCameraMovement.x -= 1.0f;
	else if (keyboard.isKeyDown(KEY_RIGHT))
		relativeCameraMovement.x = 1.0f;
	if (keyboard.isKeyDown(KEY_PGUP))
		relativeCameraMovement.y += 1.0f;
	else if (keyboard.isKeyDown(KEY_PGDN))
		relativeCameraMovement.y -= 1.0f;

	relativeCameraMovement *= deltaTime * 3.0f;
	mCamera->moveSidewards(relativeCameraMovement.x);
	mCamera->moveUpwards(relativeCameraMovement.y);
	mCamera->moveForwards(relativeCameraMovement.z);

	// rotate camera
	Vector2 rotation(mouse.getRelativeXMotion(), mouse.getRelativeYMotion());
	rotation *= 50.0f;
	if (keyboard.isKeyDown(KEY_DELETE))
		rotation.y += Math::QUARTER_PI;
	else if (keyboard.isKeyDown(KEY_END))
		rotation.y -= Math::QUARTER_PI;
	if (keyboard.isKeyDown(KEY_HOME))
		rotation.x += Math::QUARTER_PI;
	else if (keyboard.isKeyDown(KEY_INSERT))
		rotation.x -= Math::QUARTER_PI;
	rotation *= deltaTime;
	mCamera->yaw(-rotation.x, true);
	mCamera->pitch(rotation.y);

	// zoom effect
	const Real SCALE_CHANGE_FACTOR = 1.5f;
	const Real MIN_SCALE	= 0.01f;
	if (mouse.getRelativeZMotion() > 0.0f)
	{
		mScale /= SCALE_CHANGE_FACTOR;
		if (mScale < MIN_SCALE)
			mScale = MIN_SCALE;
	}
	else if (mouse.getRelativeZMotion() < 0.0f)
	{
		mScale *= SCALE_CHANGE_FACTOR;
	}

	// look at center?
	if (mouse.isButtonDown(Mouse::BUTTON_MIDDLE) || keyboard.isKeyDown(KEY_SCROLL))
		mCamera->lookAt(mCamera->getPosition(), Vector4(), Vector3(0.0f, 1.0f, 0.0f));
	
	//set camera to special position with orientation and zoom
	if (keyboard.isKeyReleased(KEY_M))
	{
		Vector3 positionWS(3.5198004731925328f, 5.2519589545972964f, -0.90308998605132462f);
		Quaternion orientation(-0.39311368061284629f, 0.59087414733766663f, 0.40100736229725070f, 0.57924301593235306f);

		mCamera->setPosition(positionWS.x, positionWS.y, positionWS.z);
		mCamera->setOrientation(orientation);
		mScale = 1.0f;
	}
}

void TSR::controlRenderer()
{
	// rendering at all?
	if (!Platform::Window::exists())
		return;

	// keyboard, octree & reconstruction
	const Keyboard &keyboard = InputManager::getSingleton().getKeyboard();
	const Tree *tree = mScene->getTree();
	const FlexibleMesh *mesh = mScene->getMostRefinedReconstruction();
	
	// change visibility / rendering type of a particular thingy?
	// tree rendering & tree-mesh/samples data rendering
	if (keyboard.isKeyDown(KEY_LCONTROL) && tree)
	{
		// specific triangle / tree intersection data
		if (mesh)
		{
			if (keyboard.isKeyReleased(KEY_F1))
				mRenderer->showPreviousTreeIntersectionTriangle();
			if (keyboard.isKeyReleased(KEY_F2))
				mRenderer->showNextTreeIntersectionTriangle();
			if (keyboard.isKeyReleased(KEY_F3))
			{
				const uint32 triangleCount = mesh->getTriangleCount();
				const uint32 randomIdx = (uint32) (triangleCount * (rand() / (double) RAND_MAX));
				mRenderer->showTreeIntersectionTriangle(randomIdx);
			}
		}

		// rendering of samples in a specific node rendering
		if (keyboard.isKeyReleased(KEY_F4))
		{
			const Nodes &nodes = tree->getNodes();
			const uint32 count = nodes.getCount();

			uint32 nodeIndex = 0;
			do
			{
				nodeIndex = (uint32) (count * (rand() / (double) RAND_MAX));
				mRenderer->showNodes(nodeIndex);
			}
			while (0 == nodes.getSampleCount(nodeIndex));
		}

		// octree rendering
		if (keyboard.isKeyReleased(KEY_F5))
			mRenderer->toggleTreeRendering(Renderer::SCENE_TREE_RENDERING_SHOW_LEAF_NEIGHBORHOODS);
		if (keyboard.isKeyReleased(KEY_F6))
			mRenderer->toggleTreeRendering(Renderer::SCENE_TREE_RENDERING_SHOW_LEAVES);
		if (keyboard.isKeyReleased(KEY_F7))
			mRenderer->toggleLeafResultsRendering(Renderer::LEAF_RESULTS_RENDERING_EMPTY);
		if (keyboard.isKeyReleased(KEY_F8))
			mRenderer->toggleLeafResultsRendering(Renderer::LEAF_RESULTS_RENDERING_NEAR_SURFACE);

		// dual octree
		const DualCells &dualCells = tree->getDualCells();
		const uint32 cellCount = dualCells.getCellCount();

		if (keyboard.isKeyReleased(KEY_F9))
		{
			const uint32 index = (uint32) (cellCount * (rand() / (double) RAND_MAX));
			mRenderer->showDualCell(index);
		}
		if (keyboard.isKeyReleased(KEY_F10))
			mRenderer->showDualCell(cellCount);
		if (keyboard.isKeyReleased(KEY_F11))
			mRenderer->showDualCell((uint32) -1);

		return;
	}

	// reconstruction & refiner rendering
	if (keyboard.isKeyDown(KEY_RCONTROL) && mesh)
	{
		// normals
		if (keyboard.isKeyReleased(KEY_1))
			mRenderer->toggleMeshNormalsVisibility();
			
		// triangle neighbors
		if (keyboard.isKeyReleased(KEY_F2))
			mRenderer->showTriangleNeighborsOfNeighbor(*mesh, 0);
		else if (keyboard.isKeyReleased(KEY_F3))
			mRenderer->showTriangleNeighborsOfNeighbor(*mesh, 1);
		else if (keyboard.isKeyReleased(KEY_F4))
			mRenderer->showTriangleNeighborsOfNeighbor(*mesh, 2);

		if (keyboard.isKeyReleased(KEY_F5))
			mRenderer->showPreviousTriangleNeighbors();
		else if (keyboard.isKeyReleased(KEY_F6))
			mRenderer->showNextTriangleNeighbors();
		if (keyboard.isKeyReleased(KEY_F7))
		{
			const uint32 count = mesh->getTriangleCount();
			const uint32 index = (uint32) (count * (rand() / (float) RAND_MAX));
			mRenderer->showTriangleNeighbors(index);
		}

		// edge neighbors
		if (keyboard.isKeyReleased(KEY_F8))
			mRenderer->showPreviousEdgeNeighbors();
		else if (keyboard.isKeyReleased(KEY_F9))
			mRenderer->showNextEdgeNeighbors();
		if (keyboard.isKeyReleased(KEY_F10))
		{
			const uint32 count = mesh->getEdgeCount();
			const uint32 index = (uint32) (count * (rand() / (float) RAND_MAX));
			mRenderer->showEdgeNeighbors(index);
		}

		// PCS refiner
		if (keyboard.isKeyReleased(KEY_F11) && mScene->getPCSRefiner())
			mRenderer->toggleMeshRefinerRendering(Renderer::MESH_REFINER_PCS_MOVEMENTS);

		//if (keyboard.isKeyReleased(KEY_F10))
		//	mRenderer->showTriangleNeighborsOfNeighbor(0);
		//else if (keyboard.isKeyReleased(KEY_F11))
		//	mRenderer->showTriangleNeighborsOfNeighbor(1)
		//else if (keyboard.isKeyReleased(KEY_F12))
		//	mRenderer->showTriangleNeighborsOfNeighbor(1);
		return;
	}

	// other stuff
	// no CONTROL key down -> general rendering control

	// backface culling
	if (keyboard.isKeyReleased(KEY_BACKSLASH))
		mRenderer->toggleBackfaceCulling();
	
	// sample rendering modes
	if (keyboard.isKeyReleased(KEY_F1))
		mRenderer->shiftSampleRendering();
	
	// show ground truth?
	const Mesh *groundTruth = mScene->getGroundTruth();
	if (keyboard.isKeyReleased(KEY_F2) && groundTruth)
	{
		if (mMeshRenderer->isUploaded(*groundTruth))
			mMeshRenderer->deleteUploadedMesh(*groundTruth);
		else
			mMeshRenderer->uploadData(*groundTruth);
		return;
	}

	// occupancy rendering
	if (keyboard.isKeyReleased(KEY_F4))
		mRenderer->toggleOccupancyRenderingFlags(Renderer::OCCUPANCY_RENDERING_FLAGS_EMPTINESS);
	if (keyboard.isKeyReleased(KEY_F5))
		mRenderer->toggleOccupancyRenderingFlags(Renderer::OCCUPANCY_RENDERING_FLAGS_OCCUPANCY);
	if (keyboard.isKeyReleased(KEY_F6))
		mRenderer->toggleOccupancyRenderingFlags(Renderer::OCCUPANCY_RENDERING_FLAGS_SAMPLENESS);
	if (keyboard.isKeyReleased(KEY_F7))
		mRenderer->toggleOccupancyRenderingFlags(Renderer::OCCUPANCY_RENDERING_FLAGS_LOG_SCALE);
	
	// views & camera
	// highlighted view
	if (keyboard.isKeyReleased(KEY_F10))
		mRenderer->highlightNextView(mScene->getViewCount());
	// view rendering mode
	if (keyboard.isKeyReleased(KEY_F11))
		mRenderer->shiftViewRendering();
	// camera
	if (keyboard.isKeyReleased(KEY_F12))
		resetCamera();
}

bool TSR::controlScene()
{
	const Keyboard &keyboard = InputManager::getSingleton().getKeyboard();

	// nothing to do
	if (!keyboard.isKeyReleased(KEY_RETURN) && !keyboard.isKeyReleased(KEY_F) && !keyboard.isKeyReleased(KEY_P))
		return false;

	// complete reconstruction?
	if (keyboard.isKeyReleased(KEY_RETURN))
		mScene->reconstruct();

	// refinement via samples?
	if (keyboard.isKeyReleased(KEY_F))
		mScene->refine(IReconstructorObserver::RECONSTRUCTION_VIA_SAMPLES);
		
	// refinement via PCS?
	if (keyboard.isKeyReleased(KEY_P))
		mScene->refine(IReconstructorObserver::RECONSTRUCTION_VIA_PCS);

	// show the reconstruction!
	const FlexibleMesh *mesh = mScene->getMostRefinedReconstruction();
	mMeshRenderer->clear();
	if (mesh)
		mMeshRenderer->uploadData(*mesh);
	return true;
}

void TSR::createNewScene(const uint32 sceneCreationType, const vector<string> &arguments)
{
	// create new 2D scene / reconstruction input data
	delete mScene;
	mScene = NULL;
	
	// renderer for the scene?
	vector<IReconstructorObserver *> observers;
	if (Platform::Window::exists())
	{
		// reset rendering
		mMeshRenderer->clear();

		delete mRenderer;
		mRenderer = new Renderer();

		observers.push_back(mRenderer);
		resetCamera();
	}

	switch (sceneCreationType)
	{
		case SCENE_CREATION_FROM_MVE_DATA:
		{
			mScene = new CapturedScene(arguments[1], observers);
			break;
		}

		case SCENE_CREATION_FROM_INTERMEDIATE_RESULTS:
		{
			const string &rootFolder = arguments[1];
			const string &previousFSSFResult = (arguments.size() > 2 ? arguments[2] : "");
			mScene = new Scene(rootFolder, previousFSSFResult, observers);
			break;
		}

		case SCENE_CREATION_SYNTHETIC:
		{
			mScene = new SyntheticScene(arguments[1], observers);
			break;
		}

		default:
			assert(false);
	}

	// render all view meshes from depth maps
	//#ifdef _DEBUG
		if (Platform::Window::exists())
		{
			const vector<FlexibleMesh *> &viewMeshes = mScene->getViewMeshes();
			const uint32 meshCount = (uint32) viewMeshes.size();
			for (uint32 meshIdx = 0; meshIdx < meshCount; ++meshIdx)
				mMeshRenderer->uploadData(*viewMeshes[meshIdx]);
		}	
	//#endif // _DEBUG
}

void TSR::resetCamera()
{
	mScale = 1.0f;
	mCamera->lookAt(Vector3(2.0f, 1.0f, 2.0f), Vector3(), Vector3(0.0f, 1.0f, 0.0f));
}
