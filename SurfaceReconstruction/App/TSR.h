/*
 * Copyright (C) 2017 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */
#ifndef _MY_APP_H_
#define _MY_APP_H_

#include <vector>
#include "Graphics/Camera2D.h"
#include "Platform/Application.h"
#include "SurfaceReconstruction/Rendering/Renderer.h"
#include "SurfaceReconstruction/Scene/Scene.h"

/// This is the main object which represents the whole application.
class TSR : public Platform::Application
{
public:
	/// Identifies how a the scene for reconstruction shall be created.
	enum SceneCreationType
	{
		SCENE_CREATION_FROM_MVE_DATA,				/// Create the scene from MVE data, such as views, sample positions, normlas etc.
		SCENE_CREATION_FROM_INTERMEDIATE_RESULTS,	/// Create scene from already computed intermediate results.
		SCENE_CREATION_SYNTHETIC,					/// Create scene by randomly placing cameras around a ground truth mesh and sampling it by these cameras.
		SCENE_CREATION_TYPE_COUNT					/// Defines the number of ways to create a scene.
	};

public:
	/** Creates the main application object.
	@param applicationHandle Set this to the WinMain HINSTANCE application handle.
	@param arguments Set this to the command line arguments without program name. */
	TSR
	(
		#ifdef _WINDOWS
			HINSTANCE applicationHandle, 
		#endif // _WINDOWS
		const std::vector<std::string> &arguments
	);

	/** Releases resources. */
	virtual ~TSR();

protected:
	/** Is called on window activation, see Application.
	virtual void onActivation() { Platform::Application::onActivation(); }

	/** Is called on window deactivation, see Application. */
	virtual void onDeactivation() { Platform::Application::onDeactivation(); }

	/** Is directly called after render() has finished. */
	virtual void postRender() { }

	/** Is called every frame to visualize stuff. */
	virtual void render();

	/** Is called every frame to update changing stuff. */
	virtual void update();

private:

	/** Processes input to control / change the camera. */
	void controlCamera();

	/** Processes input regarding renderer control / changes / configuration. */
	void controlRenderer();

	/** Processes input regarding scene control / changes. */
	bool controlScene();

	/** Possibly deletes the old and creates a new scene deterministicly.
	todo */
	void createNewScene(const uint32 sceneCreationType, const std::vector<std::string> &arguments);

	/** Sets camera orientation and position to its start values. */
	void resetCamera();

private:
	Graphics::Camera3D *mCamera;					/// This is the one and only main rendering camera of the application.
	
	SurfaceReconstruction::Renderer *mRenderer;		/// Is used to draw data like ground truth surface, samples etc.
	SurfaceReconstruction::Scene *mScene;			/// Represents the scene. (objects, cameras, views & required parameters)

	Real mScale;									/// Defines camera zoom. Is used in a uniform scaling matrix to achieve the zoom effect.
	uint32 mSceneIndex;								/// Identifies the scene file for the current scene to be used.
};

#endif // _MY_APP_H_
