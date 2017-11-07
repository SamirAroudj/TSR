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
#include "SurfaceReconstruction/Geometry/FlexibleMesh.h"
#include "SurfaceReconstruction/Geometry/RayTracer.h"
#include "SurfaceReconstruction/Refinement/MeshDijkstra.h"
#include "SurfaceReconstruction/Refinement/MeshDijkstraParameters.h"
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

	/** Traces a ray from the camera projection center through the homogeneous pixel space (window coordinates) and possibly creates a surface kernel at a potential hit position.
	@param rayHPS Set this to the homogeneous pixel space coordinates of the pixel through which the ray from the camera shall go. For example, use mouse coordinates and z = 1. */
	void createKernel(const Math::Vector3 &rayHPS);

	/** Sets camera orientation and position to its start values. */
	void resetCamera();

	/** Spreads a data-driven kernel from the entered start surfel over the mesh.
	@param startSurfel Defines where to start the Dijkstra search. */
	void spreadKernel(const SurfaceReconstruction::Surfel &startSurfel);

private:
	// Dijkstra data
	SurfaceReconstruction::MeshDijkstra mDijkstra;					/// Dijkstra search object for data-driven surface kernel computation.
	SurfaceReconstruction::MeshDijkstraParameters mDijkstraParams;	/// Parameters for running a Dijkstra search (for data-driven surface kernel calculation) over the mesh.

	// mesh data
	SurfaceReconstruction::FlexibleMesh *mMesh;						/// This is the rendered mesh over which the example data-driven surface kernel spreads.
	Math::Vector3 *mMeshTriangleNormals;							/// Necessary for Dijkstra search. Contains a normal for each triangle of mMesh.
	std::vector<uint32> mVertexNeighbors;							/// Contains for each vertex of mMesh the indices of its direct neighbors which are connected by an edge.
	std::vector<uint32> mVertexNeighborsOffsets;					/// Contains for each vertex the index of the first direct neighbor index of its neighborhood in mVertexNeighbors. mVertexNeighborsOffsets[vertexCount] = end index

	// rendering & ray tracing stuff
	Graphics::Camera3D *mCamera;									/// This is the one and only main rendering camera of the application.
	SurfaceReconstruction::RayTracer *mRayTracer;					/// For finding the start point of the surface kernel spread.
	SurfaceReconstruction::Renderer *mRenderer;						/// Is used to draw data like ground truth surface, samples etc.

	Real mScale;													/// Defines camera zoom. Is used in a uniform scaling matrix to achieve the zoom effect.
};

#endif // _SURFACE_KERNEL_COLORING_APP_H_
