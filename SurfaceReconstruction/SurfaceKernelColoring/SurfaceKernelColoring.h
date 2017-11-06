/*
 * Copyright (C) 2017 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */
#ifndef _SURFACE_KERNEL_COLORING_APP_H_
#define _SURFACE_KERNEL_COLORING_APP_H_

#include <vector>
#include "Platform/Application.h"
#include "SurfaceReconstruction/Geometry/RayTracer.h"
#include "SurfaceReconstruction/Geometry/StaticMesh.h"
#include "SurfaceReconstruction/Rendering/Renderer.h"
#include "SurfaceReconstruction/Scene/Scene.h"

/// This is the main object which represents the whole application.
class SurfaceKernelColoring : public Platform::Application
{

public:
	/** Creates the main application object.
	@param applicationHandle Set this to the WinMain HINSTANCE application handle.
	@param arguments Set this to the command line arguments without program name. */
	SurfaceKernelColoring
	(
		#ifdef _WINDOWS
			HINSTANCE applicationHandle, 
		#endif // _WINDOWS
		const std::vector<std::string> &arguments
	);

	/** Releases resources. */
	virtual ~SurfaceKernelColoring();

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

	/** Sets camera orientation and position to its start values. */
	void resetCamera();

	/** Spreads a data-driven kernel from the entered start surfel over the mesh.
	@param startSurfel Defines where to start the Dijkstra search. */
	void spreadKernel(const SurfaceReconstruction::Surfel &startSurfel);

private:
	Graphics::Camera3D *mCamera;					/// This is the one and only main rendering camera of the application.
	SurfaceReconstruction::StaticMesh *mMesh;		/// This is the rendered mesh over which the example data-driven surface kernel spreads
	SurfaceReconstruction::RayTracer *mRayTracer;	/// For finding the start point of the surface kernel spread.
	SurfaceReconstruction::Renderer *mRenderer;		/// Is used to draw data like ground truth surface, samples etc.

	Real mScale;									/// Defines camera zoom. Is used in a uniform scaling matrix to achieve the zoom effect.
};

#endif // _SURFACE_KERNEL_COLORING_APP_H_
