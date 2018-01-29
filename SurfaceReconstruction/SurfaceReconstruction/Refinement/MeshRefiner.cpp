/*
 * Copyright (C) 2017 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */

#include "Math/MathHelper.h"
#include "SurfaceReconstruction/Geometry/FlexibleMesh.h"
#include "SurfaceReconstruction/Geometry/Triangle.h"
#include "SurfaceReconstruction/Refinement/MeshRefiner.h"
#include "SurfaceReconstruction/Scene/Scene.h"
#include "SurfaceReconstruction/Scene/View.h"
#include "Utilities/HelperFunctions.h"

using namespace Graphics;
using namespace FailureHandling;
using namespace Math;
using namespace std;
using namespace SurfaceReconstruction;
using namespace Utilities;


MeshRefiner::MeshRefiner(const string &meshFileName) :
	mMesh(meshFileName)
{
	onNewStartMesh();
}

MeshRefiner::MeshRefiner(const FlexibleMesh &initialMesh) :
	mMesh(initialMesh)
{		
	onNewStartMesh();
}

MeshRefiner::MeshRefiner()
{

}

void MeshRefiner::onNewStartMesh()
{
	mRayTracer.clear();
	resize(mMesh.getVertexCount());
	mMesh.registerObserver(this);
}

MeshRefiner::~MeshRefiner()
{
	clear();
}

void MeshRefiner::clear()
{
	Patterns::Subject<IReconstructorObserver>::clear();

	mRayTracer.clear();
	mMesh.clear();
	
	// clear data of vertices
	mVectorField.clear();
	mWeightField.clear();
}

void MeshRefiner::applyMovementField()
{
	mMesh.applyMovementField(mVectorField.data());
	doSelfCheck();
}

void MeshRefiner::onEdgeMerging(const uint32 targetVertex, const uint32 edgeVertex0, const uint32 edgeVertex1)
{
	mVectorField[targetVertex] = (mVectorField[edgeVertex0] + mVectorField[edgeVertex1]) * 0.5f;
	mWeightField[targetVertex] = (mWeightField[edgeVertex0] + mWeightField[edgeVertex1]) * 0.5f;
}

void MeshRefiner::onEdgeSplitVertex(const uint32 newVertexIdx, const uint32 edgeVertex0, const uint32 edgeVertex1)
{
	// necessary resizing?
	const uint32 newMinVertexCount = newVertexIdx + 1;
	if (newMinVertexCount > mVectorField.size())
		resize(newMinVertexCount);

	// interpolate values for new vertex
	mVectorField[newVertexIdx] = (mVectorField[edgeVertex0] + mVectorField[edgeVertex1]) * 0.5f;
	mWeightField[newVertexIdx] = (mWeightField[edgeVertex0] + mWeightField[edgeVertex1]) * 0.5f;
}

void MeshRefiner::onFilterData(
	const uint32 *vertexOffsets, const uint32 vertexCount,
	const uint32 *edgeOffsets, const uint32 edgeCount,
	const uint32 *triangleOffsets, const uint32 triangleCount)
{
	FlexibleMesh::filterData<Vector3>(mVectorField, vertexOffsets);
	FlexibleMesh::filterData<Real>(mWeightField, vertexOffsets);
}

void MeshRefiner::onNewElements(
	const uint32 firstNewVertex, const uint32 newVertexCount,
	const uint32 firstNewEdge, const uint32 newEdgeCount,
	const uint32 firstNewTriangle, const uint32 newTriangleCount)
{
	mVectorField.resize(newVertexCount, Vector3(0.0f, 0.0f, 0.0f));
	mWeightField.resize(newVertexCount, 0.0f);
}

void MeshRefiner::resize(const uint32 vertexCount)
{
	mVectorField.resize(vertexCount);
	mWeightField.resize(vertexCount);
}

void MeshRefiner::onReserveMemory(const uint32 vertexCapacity, const uint32 edgeCapacity, const uint32 indexCapacity)
{
	// reserve memory
	mVectorField.reserve(vertexCapacity);
	mWeightField.reserve(vertexCapacity);
}

void MeshRefiner::updateObservers(const uint32 iteration, const string extraNameText,
	const IReconstructorObserver::ReconstructionType type) const
{
	// create a mesh and find its owner
	FlexibleMesh *result = new FlexibleMesh(mMesh);
	bool responsible = true;

	const uint32 observerCount = (uint32) mObservers.size();
	for (uint32 observerIdx = 0; observerIdx < observerCount; ++ observerIdx)
	{
		const bool tookResponsibility = mObservers[observerIdx]->onNewReconstruction(result, iteration, extraNameText, type, responsible);
		if (tookResponsibility)
			responsible = false;
	}
	
	// still responsible?
	if (responsible)
		delete result;
}

void MeshRefiner::doSelfCheck()
{
	#ifdef _DEBUG
		mMesh.doSelfCheck();

		// check vertex data
		const uint32 vertexCount = mMesh.getVertexCount();

		for (uint32 vertexIdx = 0; vertexIdx < vertexCount; ++vertexIdx)
		{
			const Vector3 &m = mVectorField[vertexIdx];
			const Real weight = mWeightField[vertexIdx];

			assert(!m.hasNaNComponent());
			assert(!isNaN(weight));
		}
	#endif // _DEBUG
}

MeshRefiner::MeshRefiner(const MeshRefiner &copy) :
	mMesh(copy.mMesh)
{
	mMesh.registerObserver(this);
	assert(false);
}