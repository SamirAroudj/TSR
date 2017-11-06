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

#include <omp.h>
#include "GL/gl.h"
#include "Graphics/Color.h"
#include "Graphics/GraphicsManager.h"
#include "Math/MathCore.h"
#include "Platform/ApplicationTimer.h"
#include "SurfaceReconstruction/Geometry/FlexibleMesh.h"
#include "SurfaceReconstruction/Geometry/StaticMesh.h"
#include "SurfaceReconstruction/Refinement/FSSFRefiner.h"
#include "SurfaceReconstruction/Refinement/MeshDijkstra.h"
#ifdef PCS_REFINEMENT
	#include "SurfaceReconstruction/Refinement/PCSRefiner.h"
#endif // PCS_REFINEMENT
#include "SurfaceReconstruction/Rendering/Renderer.h"
#include "SurfaceReconstruction/Scene/Scene.h"
#include "SurfaceReconstruction/Scene/Tree/DualCells.h"
#include "SurfaceReconstruction/Scene/Tree/Leaves.h"
#include "SurfaceReconstruction/Scene/Tree/LeavesIterator.h"
#include "SurfaceReconstruction/Scene/Tree/Nodes.h"
#include "SurfaceReconstruction/Scene/Tree/Tree.h"
#include "SurfaceReconstruction/Scene/Tree/TriangleNodesChecker.h"
#include "SurfaceReconstruction/Scene/SyntheticScene.h"
#include "SurfaceReconstruction/Scene/View.h"
#include "SurfaceReconstruction/SurfaceExtraction/Occupancy.h"

using namespace Graphics;
using namespace Math;
using namespace Platform;
using namespace SurfaceReconstruction;
using namespace std;
using namespace Utilities;

TreeIntersectionTriangle::TreeIntersectionTriangle() :
	mTriangleIdx(Triangle::INVALID_IDX)
{

}

Renderer::Renderer(const Color &highlightColor, const Graphics::Color sampleColors[3],
				   const Color surfaceColors[3], const Color &viewColor) :
	mHighlightColor(highlightColor),
	mViewColor(viewColor),

	mLightAzimuth(0.0f),

	mHighlightedView(0),
	
	mCrustFlags(0),
	mLeafResultsFlags(0),
	mMeshRefinerFlags(0),
	mNeighborsSize(1001),
	mOccupancyFlags(0),
	mTreeFlags(0),

	mShownDualCell((uint32) -1),
	mShownEdge((uint32) -1),
	mShownNodes((uint32) -1),
	mShownReconstruction((uint32) -1),
	mShownSample((uint32) -1),
	mShownTriangle((uint32) -1),
	
	mSampleRendering(SAMPLE_RENDERING_INVISIBLE),
	mViewRendering(VIEW_RENDERING_ARROWS),

	mBackfaceCulling(true),
	mShowGroundTruth(false),
	mShowMeshNormals(false)
{
	// forward colors
	for (uint32 i = 0; i < 3; ++i)
	{
		mSampleColors[i] = sampleColors[i];
		mSurfaceColors[i] = surfaceColors[i];
	}

	// enable back face culling
	if (mBackfaceCulling)
	{
		glEnable(GL_CULL_FACE);
		glCullFace(GL_BACK);
		glFrontFace(GL_CCW);
	}

	// enable depth test
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);

	// configure shading
	glShadeModel(GL_SMOOTH); // GL_FLAT
	glEnable(GL_COLOR_MATERIAL);
	glEnable(GL_NORMALIZE);

	// lights
	const float lightAmbient[4]		= { 0.3f, 0.3f, 0.3f, 1.0f };
	const float lightDiffuse[4]		= { 0.2f, 0.2f, 0.2f, 1.0f };
	const float lightSpecular[4]	= { 0.3f, 0.3f, 0.3f, 1.0f };
	
	glLightfv(GL_LIGHT0, GL_AMBIENT, lightAmbient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, lightDiffuse);
	glLightfv(GL_LIGHT0, GL_SPECULAR, lightSpecular);
	glEnable(GL_LIGHT0);

	glLightfv(GL_LIGHT1, GL_AMBIENT, lightAmbient);
	glLightfv(GL_LIGHT1, GL_DIFFUSE, lightDiffuse);
	glLightfv(GL_LIGHT1, GL_SPECULAR, lightSpecular);
	glEnable(GL_LIGHT1);

	glLightfv(GL_LIGHT2, GL_AMBIENT, lightAmbient);
	glLightfv(GL_LIGHT2, GL_DIFFUSE, lightDiffuse);
	glLightfv(GL_LIGHT2, GL_SPECULAR, lightSpecular);
	glEnable(GL_LIGHT2);

	// set point size & line width
	mElementSizes[0] = 0.5f;
	mElementSizes[1] = 0.5f;
	glPointSize(mElementSizes[0]);
	glLineWidth(mElementSizes[1]);
}

Renderer::~Renderer()
{

}

bool Renderer::onNewReconstruction(FlexibleMesh *mesh,
	const uint32 iteration, const std::string &text, const Scene::ReconstructionType type, const bool responsible)
{
	return false;
}

void Renderer::render(const Real zoom)
{
	const Camera3D *cam = Camera3D::getActiveCamera();
	if (!cam)
		return;

	// render the complete scene
	const Scene		&scene	= Scene::getSingleton();
		  Matrix4x4 zoomMatrix;
					zoomMatrix.m00 = zoomMatrix.m11 = zoomMatrix.m22 = zoom;

	// apply projection matrix
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glMultMatrixr(cam->getProjectionMatrix().getData());

	// camera view & zoom matrix
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glMultMatrixr(cam->getViewMatrix().getData());
	glMultMatrixr(zoomMatrix.getData());

	const Vector4 testWS(0.0f, 0.0f, 0.0f, 1.0f);
	const Vector4 testVS = testWS * cam->getViewMatrix();
	const Vector4 testDC = testVS * cam->getProjectionMatrix();
	const Vector3 testNDC = Vector3(testDC.x / testDC.w, testDC.y / testDC.w, testDC.z / testDC.w);
	
	// render assistant structures
	renderLeafResults();
	renderTree();

	// render samples & views
	render(scene.getViews());
	
	// render ground truth & reconstruction
	glEnable(GL_LIGHTING);
		renderGroundTruthMesh();
		renderReconstructedMesh();
		renderMeshRefinementData();
	glDisable(GL_LIGHTING);

	// and finally the origin
	renderOrigin();
}

void Renderer::renderOrientedPoint(const Vector3 &positionWS, const Vector3 &normalWS) const
{
	Color color = Color::createFromNormal(normalWS);
	glColor4fv(color.getComponents());
	glVertex3rv(&positionWS.x);
}

void Renderer::renderGroundTruthMesh()
{
	if (!mShowGroundTruth)
		return;

	// get mesh
	const StaticMesh *mesh = Scene::getSingleton().getGroundTruth();
	if (!mesh)
		return;

	render(*mesh, true);
}

void Renderer::renderReconstructedMesh()
{
	// show a reconstruction?
	if (mShownReconstruction >= Scene::RECONSTRUCTION_TYPE_COUNT)
		return;
	
	// get mesh
	const Scene &scene = Scene::getSingleton();
	const FlexibleMesh *reconstruction = scene.getReconstruction((Scene::ReconstructionType) mShownReconstruction);
	if (!reconstruction)
		return;

	// render it!
	render(*reconstruction, false);
	if (Scene::RECONSTRUCTION_VIA_SAMPLES == mShownReconstruction)
		renderRangedVertexSets();
}

void Renderer::renderRangedVertexSets() const
{
	const Scene &scene = Scene::getSingleton();
	if (!scene.getFSSFRefiner())
		return;

	// get Dijkstra search objects
	const FSSFRefiner &refiner = *scene.getFSSFRefiner();
	const MeshDijkstra *dijkstras = refiner.getDijkstras();
	const uint32 numDijkstras = omp_get_max_threads();
	if (!dijkstras)
		return;

	// get mesh data
	const FlexibleMesh &mesh = refiner.getFlexibleMesh();
	const Vector3 *normals = mesh.getNormals();
	const Vector3 *positions = mesh.getPositions();

	// render each vertex set
	glPointSize(3.0f);
	glColor3f(0.14574f, 0.623f, 0.784f);
	glBegin(GL_POINTS);
		for (uint32 setIdx = 0; setIdx < numDijkstras; ++setIdx)
		{
			const vector<RangedVertexIdx> &rangedVertexSet = dijkstras[setIdx].getVertices();
			const uint32 vertexCount = (uint32) rangedVertexSet.size();

			// render each vertex of each vertex set
			for (uint32 localIdx = 0; localIdx < vertexCount; ++localIdx)
			{
				const uint32 vertexIdx = rangedVertexSet[localIdx].getGlobalVertexIdx();
				const Vector3 &normal = normals[vertexIdx];
				const Vector3 &position = positions[vertexIdx];

				glNormal3r(normal.x, normal.y, normal.z);
				glVertex3r(position.x, position.y, position.z);
			}
		}
	glEnd();
	glPointSize(mElementSizes[0]);
}

void Renderer::render(const Mesh &mesh, bool perFaceNormal)
{
	// get mesh data
	const Vector3 *colors = mesh.getColors();
	const Vector3 *normals = mesh.getNormals();
	const Vector3 *positions = mesh.getPositions();
	const Real *scales = mesh.getScales();
	const uint32 *indices = mesh.getIndices();
	const uint32 indexCount = mesh.getIndexCount();
	const uint32 vertexCount = mesh.getVertexCount();

	if (perFaceNormal)
	{
		// render triangles with normals from triangles
		glBegin(GL_TRIANGLES);
			for (uint32 i = 0; i < indexCount; i += 3)
			{
				// get vertex data
				Vector3 v[3];
				Vector3 c[3];
				for (uint32 j = 0; j < 3; ++j)
				{
					const uint32 vertexIdx = indices[i + j];
					v[j] = positions[vertexIdx];
					c[j] = colors[vertexIdx];
				}

				// compute normal
				Vector3 normal = (v[1] - v[0]).crossProduct(v[2] - v[0]);
				normal.normalize();

				// vertex: color, position & normal
				for (uint32 j = 0; j < 3; ++j)
				{
					glColor3rv((const GLReal *) (c + j));
					glNormal3rv((const GLReal *) &normal);
					glVertex3rv((const GLReal *) (v + j));
				}
			}
		glEnd();
	}
	else
	{
		// render triangles with defined normals?
		glBegin(GL_TRIANGLES);
			for (uint32 i = 0; i < indexCount; ++i)
			{
				// get data
				const uint32	index		= indices[i];
				const Vector3	&color		= colors[index];
				const Vector3	&normal		= normals[index];
				const Vector3	&position	= positions[index];

				// vertex: color, normal & position
				glColor3rv((const GLReal *) &color);
				glNormal3rv((const GLReal *) &normal);
				glVertex3rv((const GLReal *) &position);
			}
		glEnd();
		
		// render normals?
		if (!mShowMeshNormals)
			return;
		
		const Color normalColor(0.1f, 0.1f, 0.95f, 1.0f);
		glBegin(GL_LINES);
			for (uint32 index = 0; index < vertexCount; ++index)
			{
				// get data
				const Real &scale = scales[index];
				const Vector3 &normal = normals[index];
				const Vector3 &position0 = positions[index];
				const Vector3 &position1 = position0 + normal * NORMAL_SIZE * scale;
				
				// vertex: color, normal & position
				glColor3fv(normalColor.getComponents());
				glVertex3rv(position0.getData());
				glVertex3rv(position1.getData());
			}
		glEnd();
	}
}

void Renderer::renderMeshRefinementData()
{
	// mesh refiner
	const Scene &scene = Scene::getSingleton();
	const FSSFRefiner *fssfRefiner = scene.getFSSFRefiner();

	#ifdef PCS_REFINEMENT
		// photo-consistency score-based stuff
		const PCSRefiner *pcsRefiner = scene.getPCSRefiner();

		if (pcsRefiner)
		{
			// render mesh refinement gradient field?
			if (MESH_REFINER_PHOTOCONSISTENCY_MOVEMENTS & mMeshRefinerFlags)
			{
				const vector<Vector3> &movementField = pcsRefiner->getVectorField();
				const FlexibleMesh &flexibleMesh = pcsRefiner->getFlexibleMesh();
				const Vector3 *positions = flexibleMesh.getPositions();
				const uint32 vertexCount = (uint32) movementField.size();

				glDisable(GL_LIGHTING);
				glBegin(GL_LINES);
					for (uint32 vertexIdx = 0; vertexIdx < vertexCount; ++vertexIdx)
					{
						const Vector3 &p = positions[vertexIdx];
						const Vector3 &m = movementField[vertexIdx];
						if (m.getLengthSquared() <= Math::EPSILON * Math::EPSILON)
							continue;

						// render line from vertex to target position
						glColor3f(0.1f, 0.1f, 0.9f);
						glVertex3rv(p.getData());
				
						const Vector3 source = p - m;
						glColor3f(0.9f, 0.1f, 0.1f);
						glVertex3rv(source.getData());
					}
				glEnd();
				glEnable(GL_LIGHTING);
			}
		}
	#endif // PCS_REFINEMENT
	
	// render mesh stuff
	const FlexibleMesh *mesh = scene.getMostRefinedReconstruction();
	if (!mesh)
		return;

	renderEdgeNeighbors(*mesh);
	renderTriangleNeighbors(*mesh);
	renderTreeIntersectionTriangle(*mesh);
}

void Renderer::renderEdgeNeighbors(const FlexibleMesh &mesh) const
{
	// get triangle indices
	const uint32 edgeCount = mesh.getEdgeCount();
	if (mShownEdge >= edgeCount)
		return;
	
	const Edge &edge = mesh.getEdge(mShownEdge);
	const uint32 *triangles = edge.getTriangleIndices();
	const uint32 *vertices = edge.getVertexIndices();

	// get mesh data
	const Vector3 *positions = mesh.getPositions();
	const uint32 *indices = mesh.getIndices();

	// edge: white, left triangle: red, right triangle: blue, 1st vertex: green, 2nd vertex: yellow
	const Color colors[5] = 
	{
		Color(1.0f, 0.0f, 0.0f, 1.0f),
		Color(0.0f, 0.0f, 1.0f, 1.0f),
		Color(0.0f, 1.0f, 0.0f, 1.0f),
		Color(1.0f, 1.0f, 0.0f, 1.0f),
		Color(1.0f, 1.0f, 1.0f, 1.0f)
	};

	//  render two neighboring triangles
	glBegin(GL_TRIANGLES);
		for (uint32 i = 0; i < 2; ++i)
		{
			const uint32 triangleIdx = triangles[i];
			if (Triangle::isInvalidIndex(triangleIdx))
				continue;

			// get triangle corners
			const uint32 *triangle = indices + 3 * triangleIdx;
			const Vector3 &p0 = positions[triangle[0]];
			const Vector3 &p1 = positions[triangle[1]];
			const Vector3 &p2 = positions[triangle[2]];

			// render triangle
			glColor3fv(colors[i].getComponents());
			glVertex3rv(p0.getData());
			glVertex3rv(p1.getData());
			glVertex3rv(p2.getData());
		}
	glEnd();

	// get positions of edge vertices
	const Vector3 &e0 = positions[vertices[0]];
	const Vector3 &e1 = positions[vertices[1]];

	// render edge itself
	glLineWidth(3.0f);
	glBegin(GL_LINES);
		glColor3fv(colors[4].getComponents());
		glVertex3rv(e0.getData());
		glVertex3rv(e1.getData());
	glEnd();
	glLineWidth(mElementSizes[1]);

	// render edge vertices
	glPointSize(5.0f);
	glBegin(GL_POINTS);
		glColor3fv(colors[2].getComponents());
		glVertex3rv(e0.getData());

		glColor3fv(colors[3].getComponents());
		glVertex3rv(e1.getData());
	glEnd();
	glPointSize(mElementSizes[0]);
}

void Renderer::renderTreeIntersectionTriangle(const FlexibleMesh &mesh) const
{
	// check triangle
	if (Triangle::INVALID_IDX == mInterTriangle.mTriangleIdx)
		return;

	// get tree & refiener
	const Scene &scene = scene.getSingleton();
	const Tree *tree = scene.getTree();
	if (!tree)
		return;

	// get triangle
	const Vector3 *positions = mesh.getPositions();
	const uint32 *tri = mesh.getTriangle(mInterTriangle.mTriangleIdx);
	const Vector3 v[3] = { positions[tri[0]], positions[tri[1]], positions[tri[2]] };

	// render leaves
	const Leaves &leaves = tree->getLeaves();
	const uint32 leafCount = (uint32) mInterTriangle.mLeaves.size();
	
	glColor3f(0.1124351f, 1.0f, 0.7043f);
	for (uint32 leafIdx = 0; leafIdx < leafCount; ++leafIdx)
	{
		const uint32 globalLeafIdx = mInterTriangle.mLeaves[leafIdx];
		const Scope &scope = leaves.getScope(globalLeafIdx);
		renderNode(scope, false);
	}

	// render triangle
	Vector3 normal = (v[1] - v[0]).crossProduct(v[2] - v[0]);
	normal.normalize();

	glColor3f(1.0f, 0.16908f, 0.1236f);
	glBegin(GL_TRIANGLES);
		glNormal3rv(normal.getData());
		glVertex3rv(v[0].getData());
		glVertex3rv(v[1].getData());
		glVertex3rv(v[2].getData());
	glEnd();
}

void Renderer::renderTriangleNeighbors(const FlexibleMesh &mesh) const
{
	if (Triangle::isInvalidIndex(mShownTriangle))
		return;

	// get mesh & check shown triangle index
	const uint32 triangleCount = mesh.getTriangleCount();
	if (mShownTriangle >= triangleCount)
		return;
	
	// get triangle indices
	uint32 neighbors[3];
	mesh.getTriangleNeighbors(neighbors, mShownTriangle);
	const uint32 triangleNeighbors[4] =
	{
		mShownTriangle,
		neighbors[0],
		neighbors[1],
		neighbors[2]
	};

	// get mesh data
	const Vector3 *positions = mesh.getPositions();
	const uint32 *indices = mesh.getIndices();

	// center: white, 1st neighbor: red, 2nd: green, 3rd: blue
	const Color colors[4] = 
	{
		Color(1.0f, 1.0f, 1.0f, 1.0f),
		Color(1.0f, 0.0f, 0.0f, 1.0f),
		Color(0.0f, 1.0f, 0.0f, 1.0f),
		Color(0.0f, 0.0f, 1.0f, 1.0f)
	};

	//  render each of the 4 neighbors triangles of the chosen triangle with a specific highlight color
	glBegin(GL_TRIANGLES);
		for (uint32 i = 0; i < 4; ++i)
		{
			const uint32 triangleIdx = triangleNeighbors[i];
			if (Triangle::isInvalidIndex(triangleIdx))
				continue;

			// get triangle corners
			const uint32 *triangle = indices + 3 * triangleIdx;
			const Vector3 &p0 = positions[triangle[0]];
			const Vector3 &p1 = positions[triangle[1]];
			const Vector3 &p2 = positions[triangle[2]];

			// render triangle
			glColor3fv(colors[i].getComponents());
			glVertex3rv(p0.getData());
			glVertex3rv(p1.getData());
			glVertex3rv(p2.getData());
		}
	glEnd();
}

void Renderer::render(const Polybezier<Vector3> *curves, const std::vector<uint32> *partitions, uint32 curveCount)
{
	// draw anything?
	if (!mShowGroundTruth)
		return;

	if (!curves)
		return;

	if (0 == curveCount)
		return;

	if (!partitions)
		return;

	GraphicsManager::getSingleton().enableAlphaBlending();

	for (uint32 curveIdx = 0; curveIdx < curveCount; ++curveIdx)
	{
		// get data access
		const Polybezier<Vector3>	&curve		= curves[curveIdx];
		const vector<uint32>		&partition	= partitions[curveIdx];
		const Vector3				*points		= curve.getControlPolygon();
		uint32						pointCount	= curve.getControlPolygonSize();
		uint32						subsetIdx	= 0;
		if (0 == pointCount)
			continue;


		uint32 lastSubsetIdx	= 0;
		uint32 colorIdx			= 2;
		uint32 startColorIdx	= colorIdx;
		glColor4fv(mSurfaceColors[startColorIdx].getComponents());
		if (!partition.empty())
			lastSubsetIdx = (uint32) (partition[0] == 0 ? partition.size() - 1 : partition.size() - 2);

		// draw lines with alternating colors depending on partition
		glBegin(GL_LINES);
			for (uint32 i = 0; i < pointCount - 1; ++i)
			{
				// start is a point where a subset begins? yes -> change color, next subset
				if(!partition.empty() && subsetIdx < partition.size() && partition[subsetIdx] == i)
				{
					// is it the first subset again?
					if (partition[0] != 0 && partition[partition.size() - 1] == i)
					{
						colorIdx = startColorIdx;
					}
					// special case: last subset, must be colored differently than the first and previous subset
					else if (subsetIdx == lastSubsetIdx)
					{
						uint32 previousColorIdx = colorIdx;
						for (colorIdx = 0; colorIdx < 3; ++colorIdx)
							if (colorIdx != previousColorIdx && colorIdx != startColorIdx)
								break;
					}
					// simply choose the next color for the next subset
					else if (i != 0)
					{
						++colorIdx;
						if (3 == colorIdx)
							colorIdx = 0;
					}

					glColor4fv(mSurfaceColors[colorIdx].getComponents());
					++subsetIdx;
				}

				// draw line
				const Vector3 &start	= points[i];
				const Vector3 &end		= points[i + 1];
				glVertex3r(start.x, start.y, end.z);
				glVertex3r(end.x, end.y, end.z);
			}
		glEnd();
	}

	GraphicsManager::getSingleton().disableBlending();
}

void Renderer::renderLeafResults()
{
	// visualize occupancy
	if (0 == mLeafResultsFlags)
		return;

	// get scene and check assistant structures
	const Scene &scene = Scene::getSingleton();
	if (!scene.getTree() || !scene.getOccupancy())
		return;

	// get leaves & free space
	const Tree &tree = *scene.getTree();
	const Occupancy &occupancy = *scene.getOccupancy();
	const Leaves &leaves = tree.getLeaves();
	const uint32 count = leaves.getCount();

	for (uint32 leafIdx = 0; leafIdx < count; ++leafIdx)
	{
		const Scope &leaf = leaves.getScope(leafIdx);
		//const bool empty = occupancy.isEmpty(leafIdx);
		bool empty = false; // todo

		// nodes which are certainly outside
		if (0 != (LEAF_RESULTS_RENDERING_EMPTY & mLeafResultsFlags) && empty)
		{
			glColor4f(0.0f, 1.0f, 1.0f, 1.0f);	// cyan
			renderNode(leaf, false);
			continue;
		}

		// rather close to the surface -> white
		if (0 != (LEAF_RESULTS_RENDERING_NEAR_SURFACE & mLeafResultsFlags) && !empty)
		{
			glColor4r(1.0f, 1.0f, 1.0f, 1.0f);

			renderNode(leaf, false);
			continue;
		}
	}
}

void Renderer::render(const vector<View *> &views)
{
	uint32 viewCount = (uint32) views.size();
	if (0 == viewCount)
		return;

	// views
	render(views, viewCount, mViewColor.getComponents());
	render(views, mHighlightedView, mHighlightColor.getComponents());

	if (SAMPLE_RENDERING_INVISIBLE == mSampleRendering)
		return;

	renderSamples();
}

void Renderer::render(const vector<View *> views, uint32 viewIdx, const float *color)
{
	// render views?
	if (VIEW_RENDERING_INVISIBLE == mViewRendering)
		return;

	// render a single view or all of them?
	uint32 endIdx	= (uint32) views.size();
	uint32 startIdx	= 0;
	if (viewIdx < endIdx)
	{
		startIdx	= viewIdx;
		endIdx		= viewIdx + 1;
	}

	// color
	glColor4fv(color);

	// render only points at view origins?
	if (VIEW_RENDERING_POINTS == mViewRendering)
	{
		glBegin(GL_POINTS);
			for (uint32 viewIdx = startIdx; viewIdx < endIdx; ++viewIdx)
			{
				// get & check view
				const View *view = views[viewIdx];
				if (!view)
					continue;

				const Vector4 &p0 = view->getCamera().getPosition();

				glVertex3r(p0.x, p0.y, p0.z);
			}
		glEnd();

		return;
	}

	// render views as arrows to show their positions and orientations
	if (VIEW_RENDERING_ARROWS == mViewRendering)
	{
		glBegin(GL_LINES);
			for (uint32 viewIdx = startIdx; viewIdx < endIdx; ++viewIdx)
			{
				// get & check view
				const View *view = views[viewIdx];
				if (!view)
					continue;

				const PinholeCamera &cam = view->getCamera();
				const Matrix4x4	&viewMatrix	= cam.getViewMatrix();

				const Vector3 orthogonal0	= Vector3(viewMatrix.m00, viewMatrix.m10, viewMatrix.m20) * VIEW_SIZE * 0.15f;
				const Vector3 orthogonal1	= Vector3(viewMatrix.m01, viewMatrix.m11, viewMatrix.m21) * VIEW_SIZE * 0.15f;
				const Vector3 base			= view->getViewDirection() * VIEW_SIZE;
				const Vector3 p0			= Vector3(cam.getPosition().x, cam.getPosition().y, cam.getPosition().z);

				renderArrowVertices(p0, base, orthogonal0, orthogonal1);
			}
		glEnd();

		return;
	}

	if (VIEW_RENDERING_RAY_BUNDLES == mViewRendering)
	{
		// get sample data
		const Scene &scene = Scene::getSingleton();
		const Samples &samples = scene.getSamples();
		const uint32 viewConeCount = samples.getMaxViewConeCount();

		glBegin(GL_LINES);
			for (uint32 viewConeIdx = 0; viewConeIdx < viewConeCount; ++viewConeIdx)
			{
				// get samples
				const uint32 sampleIdx = samples.getSampleIdx(viewConeIdx);
				const Vector3 &s = samples.getPositionWS(sampleIdx);
		
				// get & check view idx
				uint32 viewIdx = samples.getViewIdx(viewConeIdx);
				if (viewIdx < startIdx || viewIdx >= endIdx || !scene.isValidView(viewIdx))
					continue;

				// get view data
				const View &view = *views[viewIdx];
				const Vector3 viewDir = view.getViewDirection();
				const Vector4 &p0 = view.getCamera().getPosition();

				glVertex3r(s.x, s.y, s.z);
				glVertex3r(p0.x, p0.y, p0.z);
			}
		glEnd();

		return;
	}

	assert(false);
}

void Renderer::renderArrowVertices(const Vector3 &arrowHeadWS, const Vector3 &middleVector, const Vector3 &orthogonal0, const Vector3 &orthogonal1)
{
	// middle line
	{
		const Vector3 temp = arrowHeadWS - middleVector;
		glVertex3r(arrowHeadWS.x, arrowHeadWS.y, arrowHeadWS.z);
		glVertex3r(temp.x, temp.y, temp.z);
	}
				
	// arrow head
	{
		const Vector3 p1 = arrowHeadWS - middleVector * 0.25f;

		// first corner
		glVertex3rv((const GLReal *) &arrowHeadWS);
		glVertex3r(p1.x + orthogonal0.x, p1.y + orthogonal0.y, p1.z + orthogonal0.z);
		glVertex3rv((const GLReal *) &arrowHeadWS);
		glVertex3r(p1.x - orthogonal0.x, p1.y - orthogonal0.y, p1.z - orthogonal0.z);

		// second corner
		glVertex3rv((const GLReal *) &arrowHeadWS);
		glVertex3r(p1.x + orthogonal1.x, p1.y + orthogonal1.y, p1.z + orthogonal1.z);
		glVertex3rv((const GLReal *) &arrowHeadWS);
		glVertex3r(p1.x - orthogonal1.x, p1.y - orthogonal1.y, p1.z - orthogonal1.z);
	}
}

void Renderer::renderSamples()
{
	// render each sample
	const Samples &samples = Scene::getSingleton().getSamples();
	const uint32 sampleCount = samples.getCount();

	glBegin(GL_POINTS);
		for (uint32 sampleIdx = 0; sampleIdx < sampleCount; ++sampleIdx)
			renderSample(sampleIdx, false);
	glEnd();
}

void Renderer::render(const std::vector<uint32> &sampleSet, bool highlighted)
{
	// render each sample in sampleSet
	const uint32 sampleCount = (uint32) sampleSet.size();

	GLenum mode = GL_POINTS;
	glBegin(mode);
		for (uint32 sampleIdx = 0; sampleIdx < sampleCount; ++sampleIdx)
			renderSample(sampleSet[sampleIdx], highlighted);
	glEnd();
}

void Renderer::renderSamplesOfChosenNode() const
{
	// render samples of a node?
	if ((uint32) -1 == mShownNodes)
		return;
	
	// check & get tree
	const Scene &scene = Scene::getSingleton();
	if (!scene.getTree())
		return;
	const Tree &tree = *scene.getTree();

	// get samples of chosen node
	uint32 sampleCount = 0;
	const uint32 startIdx = tree.getNodes().getSamples(sampleCount, mShownNodes);
	const uint32 endIdx = startIdx + sampleCount;

	// render samples
	glPointSize(3.0f);
	glLineWidth(3.0f);
		glBegin(GL_POINTS);
			for (uint32 sampleIdx = startIdx; sampleIdx < endIdx; ++sampleIdx)
				renderSample(sampleIdx, false, SAMPLE_RENDERING_POINTS_NORMAL_COLOR);
		glEnd();
	glPointSize(mElementSizes[0]);
	glLineWidth(mElementSizes[1]);
}

void Renderer::renderSample(const uint32 sampleIdx,
	const bool highlighted, SAMPLE_RENDERING sampleRendering) const
{
	const Samples &samples = Scene::getSingleton().getSamples();

	// overwrite sample rendering behaviour?
	if (sampleRendering >= SAMPLE_RENDERING_INVALID)
		sampleRendering = mSampleRendering;
	if (sampleRendering >= SAMPLE_RENDERING_INVALID)
		sampleRendering = SAMPLE_RENDERING_POINTS_NORMAL_COLOR;

	// render this sample?
	if (SAMPLE_RENDERING_INVISIBLE == sampleRendering)
		return;
	
	// get sample data
	const Vector3 &p = samples.getPositionWS(sampleIdx);
	const Vector3 &normal = samples.getNormalWS(sampleIdx);
	const Vector3 &c = samples.getColor(sampleIdx);

	// color
	const Real	saturation	= (highlighted ? 0.5f : 1.0f);
	if (SAMPLE_RENDERING_POINTS_DATA_COLOR == sampleRendering)
	{
		glColor3r(c.x, c.y, c.z);
	}
	else
	{
		const Color color = Color::createFromNormal(normal, saturation);
		glColor4fv(color.getComponents());
	}

	// render as point?
	if (SAMPLE_RENDERING_POINTS_NORMAL_COLOR == sampleRendering || SAMPLE_RENDERING_POINTS_DATA_COLOR == sampleRendering)
	{
		const Camera3D &cam = *Camera3D::getActiveCamera();
		const Vector4 &camWS = cam.getPosition();
		const Vector3 toCam	= Vector3(camWS.x, camWS.y, camWS.z) - p;

		if (!mBackfaceCulling || normal.dotProduct(toCam) > 0.0f)
			glVertex3r(p.x, p.y, p.z);
		return;
	}

	assert(false);
}

void Renderer::renderOrigin()
{
	// render world space origin
	glBegin(GL_LINES);
		// x-axis
		glColor4f(0.2f, 0.0f, 0.0f, 0.0f);
		glVertex3r(-1.0f, 0.0f, 0.0f);
		glColor4f(1.0f, 0.0f, 0.0f, 0.0f);
		glVertex3r(1.0f, 0.0f, 0.0f);

		// y-axis
		glColor4f(0.0f, 0.2f, 0.0f, 0.0f);
		glVertex3r(0.0f, -1.0f, 0.0f);
		glColor4f(0.0f, 1.0f, 0.0f, 0.0f);
		glVertex3r(0.0f, 1.0f, 0.0f);

		// z-axis
		glColor4f(0.0f, 0.0f, 0.2f, 0.0f);
		glVertex3r(0.0f, 0.0f, -1.0f);
		glColor4f(0.0f, 0.0f, 1.0f, 0.0f);
		glVertex3r(0.0f, 0.0f, 1.0f);
	glEnd();
}

void Renderer::showTriangleNeighborsOfNeighbor(const FlexibleMesh &mesh, const uint32 localNeighborIdx)
{
	// valid neighbor?
	if (localNeighborIdx >= 3)
		return;

	// valid triangle?
	if (Triangle::isInvalidIndex(mShownTriangle))
		return;

	// valid shown triangle?
	const uint32 triangleCount = mesh.getTriangleCount();
	if (mShownTriangle >= triangleCount)
		return;
	
	// get triangle neighbors
	uint32 neighbors[3];
	mesh.getTriangleNeighbors(neighbors, mShownTriangle);

	// valid neighbor?
	const uint32 neighborTriangleIdx = neighbors[localNeighborIdx];
	if (Triangle::isInvalidIndex(neighborTriangleIdx))
		return;

	mShownTriangle = neighborTriangleIdx;
}

void Renderer::renderTree()
{
	// check tree
	const Scene &scene = Scene::getSingleton();
	if (!scene.getTree())
		return;
	
	// get leaves data
	const Tree &tree = *scene.getTree();
	const Leaves &leaves = tree.getLeaves();
	const uint32 count = leaves.getCount();

	if (0 == count)
		return;

	// render every leaf = leaf neighbors graph vertices
	if (SCENE_TREE_RENDERING_SHOW_LEAVES & mTreeFlags)
	{
		glColor3f(0.5f, 0.5f, 0.5f);
		for (uint32 i = 0; i < count; ++i)
		{
			const Scope &scope = leaves.getScope(i);
			renderNode(scope, false);
		}
	}

	// render every leaf neighbors graph edge
	if (SCENE_TREE_RENDERING_SHOW_LEAF_NEIGHBORHOODS & mTreeFlags)
	{
		// draw every leaf neighbors (set of edges starting at leaf and ending at its adjacent leaf nodes)
		glColor3f(0.3f, 0.4f, 0.3f);
		const Leaves &leaves = tree.getLeaves();

		for (uint32 leafIdx = 0; leafIdx < count; ++leafIdx)
		{
			const Scope	&leaf = leaves.getScope(leafIdx);
			const Vector3 center = leaf.getCenterPosition();
			uint32 neighborsSize = 0;

			const uint32 *neighbors = leaves.getNeighbors(neighborsSize, leafIdx);

			// an edge from leaf to each neighbor
			glBegin(GL_LINES);
				for (uint32 edgeIdx = 0; edgeIdx < neighborsSize; ++edgeIdx)
				{
					const Scope	&neighborLeaf = leaves.getScope(neighbors[edgeIdx]);
					const Vector3 &neighborCenter = neighborLeaf.getCenterPosition();

					glVertex3r(center.x, center.y, center.z);
					glVertex3r(neighborCenter.x, neighborCenter.y, neighborCenter.z);
				}
			glEnd();
		}
	}

	// render Dual Octree cells
	const DualCells &dualCells = tree.getDualCells();
	const uint32 *dualCellIndices = dualCells.getIndices();
	const uint32 dualCellCount = dualCells.getCellCount();
	if (mShownDualCell > dualCellCount)
		return;
	
	const bool	 showAll = (mShownDualCell == dualCellCount);
	const uint32 start = (showAll ? 0 : mShownDualCell);
	const uint32 end = (showAll ? dualCellCount : mShownDualCell + 1);
	const uint32 *corners = dualCellIndices + start * Nodes::CHILD_COUNT;

	Vector3	vertices[Nodes::CHILD_COUNT];
	glBegin(GL_LINES);
		glColor3f(1.0f, 0.75f, 0.0f);
		for (uint32 cellIdx = start; cellIdx < end; ++cellIdx, corners += Nodes::CHILD_COUNT)
		{
			for (uint32 i = 0; i < Nodes::CHILD_COUNT; ++i)
			{
				const Scope &leaf = leaves.getScope(corners[i]);
				vertices[i] = leaf.getCenterPosition();
			}

			renderDualCubeVertices(vertices);
		}
	glEnd();
}

void Renderer::renderNode(const Scope &node, bool filled) const
{
	// compute node vertices
	const Vector3 min = node.getMinimumCoordinates();
	const Real size	= node.getSize();

	const Vector3 vertices[] =
	{
		Vector3(min.x, min.y, min.z),
		Vector3(min.x + size, min.y, min.z),
		Vector3(min.x, min.y + size, min.z),
		Vector3(min.x + size, min.y + size, min.z),
		Vector3(min.x, min.y, min.z + size),
		Vector3(min.x + size, min.y, min.z + size),
		Vector3(min.x, min.y + size, min.z + size),
		Vector3(min.x + size, min.y + size, min.z + size)
	};

	if (!filled)
	{
		glBegin(GL_LINES);
			// bottom
			glVertex3rv((Real *) &vertices[0]);
			glVertex3rv((Real *) &vertices[1]);
			glVertex3rv((Real *) &vertices[1]);
			glVertex3rv((Real *) &vertices[5]);
			glVertex3rv((Real *) &vertices[5]);
			glVertex3rv((Real *) &vertices[4]);
			glVertex3rv((Real *) &vertices[4]);
			glVertex3rv((Real *) &vertices[0]);

			// top
			glVertex3rv((Real *) &vertices[2]);
			glVertex3rv((Real *) &vertices[3]);
			glVertex3rv((Real *) &vertices[3]);
			glVertex3rv((Real *) &vertices[7]);
			glVertex3rv((Real *) &vertices[7]);
			glVertex3rv((Real *) &vertices[6]);
			glVertex3rv((Real *) &vertices[6]);
			glVertex3rv((Real *) &vertices[2]);

			// vertical ones
			glVertex3rv((Real *) &vertices[0]);
			glVertex3rv((Real *) &vertices[2]);
			glVertex3rv((Real *) &vertices[1]);
			glVertex3rv((Real *) &vertices[3]);
			glVertex3rv((Real *) &vertices[4]);
			glVertex3rv((Real *) &vertices[6]);
			glVertex3rv((Real *) &vertices[5]);
			glVertex3rv((Real *) &vertices[7]);

			// diagonal edges
			glVertex3rv((Real *) &vertices[0]);
			glVertex3rv((Real *) &vertices[7]);
			glVertex3rv((Real *) &vertices[1]);
			glVertex3rv((Real *) &vertices[6]);
			glVertex3rv((Real *) &vertices[2]);
			glVertex3rv((Real *) &vertices[5]);
			glVertex3rv((Real *) &vertices[3]);
			glVertex3rv((Real *) &vertices[4]);
		glEnd();
	}
	else
	{
		glBegin(GL_QUADS);
			glVertex3rv((Real *) &vertices[0]);
			glVertex3rv((Real *) &vertices[1]);
			glVertex3rv((Real *) &vertices[3]);
			glVertex3rv((Real *) &vertices[2]);

			glVertex3rv((Real *) &vertices[4]);
			glVertex3rv((Real *) &vertices[5]);
			glVertex3rv((Real *) &vertices[6]);
			glVertex3rv((Real *) &vertices[7]);

			glVertex3rv((Real *) &vertices[0]);
			glVertex3rv((Real *) &vertices[1]);
			glVertex3rv((Real *) &vertices[5]);
			glVertex3rv((Real *) &vertices[4]);

			glVertex3rv((Real *) &vertices[2]);
			glVertex3rv((Real *) &vertices[3]);
			glVertex3rv((Real *) &vertices[7]);
			glVertex3rv((Real *) &vertices[6]);

			glVertex3rv((Real *) &vertices[0]);
			glVertex3rv((Real *) &vertices[4]);
			glVertex3rv((Real *) &vertices[6]);
			glVertex3rv((Real *) &vertices[2]);

			glVertex3rv((Real *) &vertices[1]);
			glVertex3rv((Real *) &vertices[5]);
			glVertex3rv((Real *) &vertices[7]);
			glVertex3rv((Real *) &vertices[3]);
		glEnd();
	}
}

void Renderer::renderDualCubeVertices(const Vector3 *VB)
{
	// 4 front lines
	glVertex3r(VB[4].x, VB[4].y, VB[4].z);
	glVertex3r(VB[5].x, VB[5].y, VB[5].z);
	glVertex3r(VB[4].x, VB[4].y, VB[4].z);
	glVertex3r(VB[6].x, VB[6].y, VB[6].z);
	glVertex3r(VB[5].x, VB[5].y, VB[5].z);
	glVertex3r(VB[7].x, VB[7].y, VB[7].z);
	glVertex3r(VB[6].x, VB[6].y, VB[6].z);
	glVertex3r(VB[7].x, VB[7].y, VB[7].z);

	// 4 back lines
	glVertex3r(VB[0].x, VB[0].y, VB[0].z);
	glVertex3r(VB[1].x, VB[1].y, VB[1].z);
	glVertex3r(VB[0].x, VB[0].y, VB[0].z);
	glVertex3r(VB[2].x, VB[2].y, VB[2].z);
	glVertex3r(VB[1].x, VB[1].y, VB[1].z);
	glVertex3r(VB[3].x, VB[3].y, VB[3].z);
	glVertex3r(VB[2].x, VB[2].y, VB[2].z);
	glVertex3r(VB[3].x, VB[3].y, VB[3].z);

	// 4 horizontal bottom and top lines
	glVertex3r(VB[0].x, VB[0].y, VB[0].z);
	glVertex3r(VB[4].x, VB[4].y, VB[4].z);
	glVertex3r(VB[1].x, VB[1].y, VB[1].z);
	glVertex3r(VB[5].x, VB[5].y, VB[5].z);
	glVertex3r(VB[2].x, VB[2].y, VB[2].z);
	glVertex3r(VB[6].x, VB[6].y, VB[6].z);
	glVertex3r(VB[3].x, VB[3].y, VB[3].z);
	glVertex3r(VB[7].x, VB[7].y, VB[7].z);
}

void Renderer::update()
{
	const Real deltaTime = ApplicationTimer::getSingleton().getDeltaTime();
	mLightAzimuth += deltaTime / 10.0f;
	while (mLightAzimuth > Math::TWO_PI)
		mLightAzimuth -= Math::TWO_PI;

	const float	radius = 20.0f;
	const float	height = 10.0f;
	const float lightPos0[] =
	{
		(float) (radius * cosr(mLightAzimuth)),
		height,
		(float) (radius * sinr(mLightAzimuth)),
		1.0f
	};

	const float lightPos1[] =
	{
		(float) (radius * cosr(mLightAzimuth + Math::PI)),
		0.5f * height,
		(float) (radius * sinr(mLightAzimuth + Math::PI)),
		1.0f
	};
	const float lightPos2[] =
	{
		(float) (radius * cosr(mLightAzimuth + Math::HALF_PI)),
		0,
		(float) (radius * sinr(mLightAzimuth + Math::HALF_PI)),
		1.0f
	};

	glLightfv(GL_LIGHT0, GL_POSITION, lightPos0);
	glLightfv(GL_LIGHT1, GL_POSITION, lightPos1);
	glLightfv(GL_LIGHT2, GL_POSITION, lightPos2);
}

void Renderer::showTreeIntersectionTriangle(const uint32 triangleIdx)
{
	// get refiner
	const Scene &scene = Scene::getSingleton();
	const FSSFRefiner *refiner = scene.getFSSFRefiner();
	if (!refiner)
		return;
	
	// get triangle count
	const FlexibleMesh &mesh = refiner->getFlexibleMesh();
	const uint32 triangleCount = mesh.getTriangleCount();
	if (0 == triangleCount)
		return;

	// check triangle choice & update mInterTriangle
	if (triangleIdx >= triangleCount)
	{
		mInterTriangle.mTriangleIdx = Triangle::INVALID_IDX;
		mInterTriangle.mLeaves.clear();
		return;
	}
	mInterTriangle.mTriangleIdx = triangleIdx;

	// get tree
	const Tree *tree = scene.getTree();
	if (!tree)
		return;
	
	// get triangle & create checker	
	const Vector3 *positions = mesh.getPositions();
	const uint32 *triangle = mesh.getTriangle(mInterTriangle.mTriangleIdx);
	TriangleNodesChecker checker(positions[triangle[0]], positions[triangle[1]], positions[triangle[2]], (uint32) -1);

	// find intersection nodes
	mInterTriangle.mLeaves.clear();
	for (LeavesIterator it(*tree, &checker); !it.isAtTheEnd(); ++it)
		mInterTriangle.mLeaves.push_back(it.getLeafIndex());
}
