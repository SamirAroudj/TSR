/*
 * Copyright (C) 2017 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */
#include "Git.h"
#include "Graphics/GraphicsManager.h"
#include "Graphics/MagicConstants.h"
#include "Math/MathCore.h"
#include "Math/MathHelper.h"
#include "Platform/ApplicationTimer.h"
#include "Platform/Input/InputManager.h"
#include "Platform/ParametersManager.h"
#include "SurfaceKernelColoring/SurfaceKernelColoring.h"

using namespace std;
using namespace Graphics;
using namespace Input;
using namespace Math;
using namespace Platform;
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
	mScale(1.0f)
{
	// output git data
	cout << Git::getStatus() << endl;

	// check command line arguments
	uint32 argumentCount = (uint32) arguments.size();
	if (2 != argumentCount)
	{
		// log this
		cerr << "Invalid argument count!\n";
		cerr << "Required arguments:\n";
		cerr << "<path/program config file> <path>/Mesh.ply\n";
		cerr << flush;

		mRunning = false;
		return;
	}

	// create image manager & rendering objects

	if (Platform::Window::exists())
	{
		const uint32 colorResolution = 32;
		GraphicsManager *graphicsManager = new GraphicsManager(colorResolution, ::BACK_BUFFER_CLEAR_COLOR);

		// create camera
		Platform::Window &window = Platform::Window::getSingleton();
		mCamera = new Camera3D(Math::HALF_PI, window.getAspectRatio(), 0.1f, 25.0f);
		mCamera->setAsActiveCamera();
	}
	else // if there is no window then reconstruct stuff but do not execute the main loop
	{
		mRunning = false;
	}
	
	mMesh = new StaticMesh(arguments[1]);
	mRayTracer = new RayTracer();
	mRayTracer->createStaticScene(*mMesh);
}

SurfaceKernelColoring::~SurfaceKernelColoring()
{
	// free visualization stuff
	delete mRayTracer;
	delete mMesh;
	delete mCamera;
	delete mRenderer;

	if (GraphicsManager::exists())
		delete GraphicsManager::getSingletonPointer();
}

void SurfaceKernelColoring::render()
{
	GraphicsManager::getSingleton().clearBackAndDepthStencilBuffer();
		mRenderer->render(mScale);
		mRenderer->render(*mMesh);
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

	if (mouse.isButtonDown(Mouse::BUTTON_PRIMARY))
	{
		// trace a ray from the mouse 2D coords along the corresponding pixel ray through the scene 
		// ray start
		const Vector4 &camHWS = mCamera->getPosition();
		const Vector3 rayStart(camHWS.x, camHWS.y, camHWS.z);
		
		// ray direction
		const Vector3 mouseCoords(mouse.getAbsoluteX(), mouse.getAbsoluteY(), 1.0f);
		const ImgSize &windowSize = Window::getSingleton().getSize();
		const Matrix3x3 hPSToNNRayDirWS = mCamera->computeHPSToNNRayDirWS(windowSize);
		Vector3 rayDir = mouseCoords * hPSToNNRayDirWS;
		rayDir.normalize();

		// trace ray
		Surfel surfel;
		if (mRayTracer->findIntersection(surfel, rayStart, rayDir))
		{
			spreadKernel(surfel);
		}

		return true;
	}

	return false;
}

void SurfaceKernelColoring::spreadKernel(const Surfel &startSurfel)
{

}

void SurfaceKernelColoring::resetCamera()
{
	mScale = 1.0f;
	mCamera->lookAt(Vector3(2.0f, 1.0f, 2.0f), Vector3(), Vector3(0.0f, 1.0f, 0.0f));
}
