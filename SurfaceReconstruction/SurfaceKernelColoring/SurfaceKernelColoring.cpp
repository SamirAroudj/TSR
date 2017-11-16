/*
 * Copyright (C) 2017 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */
#ifdef _WINDOWS
	#include <Windows.h>
#endif // _WINDOWS
#include "Git.h"
#include "Graphics/GraphicsManager.h"
#include "Graphics/MagicConstants.h"
#include "Math/MathCore.h"
#include "Math/MathHelper.h"
#include "Platform/ApplicationTimer.h"
#include "Platform/Input/InputManager.h"
#include "Platform/ParametersManager.h"
#include "SurfaceKernelColoring/SurfaceKernelColoring.h"
#include "SurfaceReconstruction/Refinement/MeshDijkstra.h"

using namespace std;
using namespace Graphics;
using namespace Input;
using namespace Math;
using namespace Platform;
using namespace Storage;
using namespace SurfaceReconstruction;
using namespace Utilities;

SurfaceKernelColoring::SurfaceKernelColoring
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
	mMesh(NULL),
	mRayTracer(NULL),
	mRenderer(NULL),
	mScale(1.0f),
	mOutputMeshCounter(0)
{
	// output git data
	cout << Git::getStatus() << endl;

	// check command line arguments
	uint32 argumentCount = (uint32) arguments.size();
	if (3 != argumentCount)
	{
		// log this
		cerr << "Invalid argument count!\n";
		cerr << "Required arguments:\n";
		cerr << "<path/program config file> <path>/outputFolder <path>/Mesh.ply\n";
		cerr << flush;

		mRunning = false;
		return;
	}

	// create image manager & rendering objects
	if (Platform::Window::exists())
	{
		const uint32 colorResolution = 32;
		GraphicsManager *graphicsManager = new GraphicsManager(colorResolution, ::BACK_BUFFER_CLEAR_COLOR);
		
		// create camera & renderer
		Platform::Window &window = Platform::Window::getSingleton();
		mCamera = new Camera3D(Math::HALF_PI, window.getAspectRatio(), 0.1f, 25.0f);
		mCamera->setAsActiveCamera();
		resetCamera();
		window.hideOperatingSystemCursor();

		mRenderer = new Renderer();
	}
	else // if there is no window then reconstruct stuff but do not execute the main loop
	{
		mRunning = false;
	}

	// set output folder
	mOutputFolder = arguments[1];

	// load mesh data
	mMesh = new FlexibleMesh(arguments[2]);
	mMesh->getVertexNeighbors(mVertexNeighbors, mVertexNeighborsOffsets);
	mMeshTriangleNormals = new Vector3[mMesh->getTriangleCount()];
	mMesh->computeNormalsOfTriangles(mMeshTriangleNormals);

	// scale mesh
	const Real scale = 5.0f;
	const uint32 vertexCount = mMesh->getVertexCount();
	for (uint32 vertexIdx = 0; vertexIdx < vertexCount; ++vertexIdx)
		mMesh->setPosition(mMesh->getPosition(vertexIdx) * scale, vertexIdx);

	// get mesh name
	Path path(arguments[2]);
	mMeshName = path.getLeafName();
	mMeshName.resize(mMeshName.length() - 4);

	// ray tracing data
	mRayTracer = new RayTracer();
	mRayTracer->createStaticScene(*mMesh, false);

	const Graphics::Color backbufferColor(1.0f, 1.0f, 1.0f, 1.0f);
	GraphicsManager::getSingleton().setClearColor(backbufferColor);
}

SurfaceKernelColoring::~SurfaceKernelColoring()
{
	// free data
	delete mRayTracer;
	delete mMeshTriangleNormals;
	delete mMesh;
	delete mRenderer;
	delete mCamera;

	if (GraphicsManager::exists())
		delete GraphicsManager::getSingletonPointer();
}

void SurfaceKernelColoring::render()
{
	// mouse coords
	const Mouse &mouse =InputManager::getSingleton().getMouse();
	const Vector2 mouseNDC(mouse.getAbsoluteX(), mouse.getAbsoluteY());

	// render scene
	GraphicsManager::getSingleton().clearBackAndDepthStencilBuffer();
		mRenderer->render(mScale);
		mRenderer->render(*mMesh);
		mRenderer->renderCursor(mouseNDC, Color(1.0f, 0.0f, 1.0f, 1.0f), 0.05f);
	GraphicsManager::getSingleton().presentBackBuffer();
}

void SurfaceKernelColoring::update()
{
	InputManager &inputManager = InputManager::getSingleton();
	const Keyboard &keyboard = inputManager.getKeyboard();

	// quit application?
	if (!Platform::Window::exists() || keyboard.isKeyDown(KEY_ESCAPE))
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

void SurfaceKernelColoring::controlCamera()
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
	const Real MIN_SCALE = 0.01f;
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

void SurfaceKernelColoring::controlRenderer()
{
	// keyboard & tree
	const Keyboard &keyboard = InputManager::getSingleton().getKeyboard();

	
	// general rendering control
	
	// backface culling
	if (keyboard.isKeyReleased(KEY_BACKSLASH))
		mRenderer->toggleBackfaceCulling();
	// camera
	if (keyboard.isKeyReleased(KEY_F12))
		resetCamera();
}

bool SurfaceKernelColoring::controlScene()
{
	InputManager &inputManager = InputManager::getSingleton();
	const Keyboard &keyboard = inputManager.getKeyboard(); 
	const Mouse &mouse = inputManager.getMouse();

	if (keyboard.isKeyPressed(KEY_S))
		saveMesh();

	if (keyboard.isKeyPressed(KEY_C))
		clearMeshColors();

	if (mouse.isButtonDown(Mouse::BUTTON_PRIMARY))
	{
		const Vector2 mouseCoords(mouse.getAbsoluteX(), mouse.getAbsoluteY());
		createKernel(mouseCoords);
	}

	return false;
}

void SurfaceKernelColoring::createKernel(const Vector2 &mouseCoords)
{
	// trace a ray from the mouse 2D coords along the corresponding pixel ray through the scene 
	// ray start
	const Vector4 &camHWS = mCamera->getPosition();
	const Vector3 rayStart(camHWS.x, camHWS.y, camHWS.z);
	
	// ray direction
	const Matrix3x3 hNDCToNNRayDirWS = mCamera->computeHNDCToNNRayDirWS();
	const Vector3 rayHNDC(mouseCoords.x, mouseCoords.y, 1.0f);
	Vector3 rayDir = rayHNDC * hNDCToNNRayDirWS;
	rayDir.normalize();

	// trace ray
	Surfel surfel;
	if (!mRayTracer->findIntersection(surfel, rayStart, rayDir))
		return;
	
	spreadKernel(surfel);
}

void SurfaceKernelColoring::spreadKernel(const Surfel &startSurfel)
{	
	const uint32 *hitTriangle = mMesh->getTriangle(startSurfel.mTriangleIdx);
	const Real maxCosts = 0.25f;

	// spread kernel
	MeshDijkstra dijkstra;
	dijkstra.findVertices(mMesh, mMeshTriangleNormals,
		mVertexNeighbors.data(), mVertexNeighborsOffsets.data(),
		startSurfel, hitTriangle,
		maxCosts, mDijkstraParams);

	// get vertices within range
	vector<RangedVertexIdx> &vertices = dijkstra.getVertices();
	const vector<uint32> &order = dijkstra.getOrder();
	const uint32 neighborhoodSize = (uint32) vertices.size();
	
	// kernel weights & maximum weight
	Real maximum = -FLT_MAX;

	for (uint32 localIdx = 0; localIdx < neighborhoodSize; ++localIdx)
	{
		RangedVertexIdx &v = vertices[localIdx];
		const Real costs = v.getCosts();
		const Real weight = getKernel2DPoly3Spiky(costs, maxCosts);
		v.setCosts(weight);
		if (weight > maximum)
			maximum = weight;
	}

	// color vertices within range according to weights
	const Real minColor = 0.5f;
	Vector3 color;

	for (uint32 localIdx = 0; localIdx < neighborhoodSize; ++localIdx)
	{
		const RangedVertexIdx &v = vertices[localIdx];
		const Real extra = (1.0f - minColor) * v.getCosts() / maximum;
		color.set(minColor + extra, minColor + 0.5f * extra, minColor + extra);

		mMesh->setColor(color, v.getGlobalVertexIdx());
	}	
}

void SurfaceKernelColoring::clearMeshColors()
{
	// default color for all vertices = grey
	Vector3 color(0.5f, 0.5f, 0.5f);
	const uint32 vertexCount = mMesh->getVertexCount();
	for (uint32 vertexIdx = 0; vertexIdx < vertexCount; ++vertexIdx)
		mMesh->setColor(color, vertexIdx);
}

void SurfaceKernelColoring::saveMesh()
{
	// output mesh name
	char buffer[1000];
	sprintf(buffer, "%.4u", mOutputMeshCounter);
	const string counter = buffer;
	const string name = counter + mMeshName;

	// save to file
	const Path fileName = Path::appendChild(mOutputFolder, name);
	mMesh->saveToFile(fileName, true, false);
	++mOutputMeshCounter;
}

void SurfaceKernelColoring::resetCamera()
{
	mScale = 1.0f;
	mCamera->lookAt(Vector3(2.0f, 1.0f, 2.0f), Vector3(), Vector3(0.0f, 1.0f, 0.0f));
}
