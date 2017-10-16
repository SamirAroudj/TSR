/*
 * Copyright (C) 2017 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */

#include "Math/MathHelper.h"
#include "SurfaceReconstruction/Scene/Scene.h"
#include "SurfaceReconstruction/Scene/Tree/DualCells.h"
#include "SurfaceReconstruction/Scene/Tree/Leaves.h"
#include "SurfaceReconstruction/Scene/Tree/Tree.h"
#include "SurfaceReconstruction/SurfaceExtraction/DualMarchingCells.h"
#include "SurfaceReconstruction/SurfaceExtraction/Occupancy.h"

using namespace Math;
using namespace std;
using namespace SurfaceReconstruction;

DualMarchingCells::DualMarchingCells(const Occupancy &occupancy, const uint32 minTriangleIsleSize)
{
	cout << "Extracting crust surface using Dual Marching Cubes." << endl;

	extractMesh(occupancy);
	mSurface.deleteIsolatedGeometry(minTriangleIsleSize);
	if (0 == mSurface.getVertexCount() || 0 == mSurface.getIndexCount())
		return;

	mSurface.fillHoles();
	mSurface.computeNormalsWeightedByAngles();

	cout << "Finished surface extraction." << endl;
}

void DualMarchingCells::extractMesh(const Occupancy &occupancy)
{
	// clean start
	clear();
	mSurface.clear();

	// exctract surface data for each dual cell
	// get Octree leaves
	const Scene &scene = Scene::getSingleton();
	const Tree &tree = *scene.getTree();
	const Leaves &leaves = tree.getLeaves();

	// get Octree dual cells
	const DualCells &dualCells = tree.getDualCells();
	const uint32 cellCount = dualCells.getCellCount();
	const uint32 *dualCellIndices = dualCells.getIndices();

	extractSurfaces(occupancy, leaves, dualCellIndices, cellCount);
}

void DualMarchingCells::extractSurfaces(const Occupancy &occupancy, const Leaves &leaves, const uint32 *dualCellIndices, const uint32 cellCount)
{
	// reserve memory for DMC
	const uint32 guessedVertexCount = cellCount;
	const uint32 guessedIndexCount = cellCount * 3;
	const uint32 guessedEdgeCount = (uint32) ceilr(guessedIndexCount * 0.5f);
	mSurface.reserve(guessedVertexCount, guessedEdgeCount, guessedIndexCount);

	// see http://paulbourke.net/geometry/polygonise/ for order of cell corners & edges as well as tables
	// process all dual cells
	for (uint32 cellIdx = 0; cellIdx < cellCount; ++cellIdx, dualCellIndices += Nodes::CHILD_COUNT)
	{
		fillCellCorners(occupancy, leaves, dualCellIndices);
			
		// cell configuration - what kind of cuts & triangulation
		const uint32 cuttingsIdx = mCell.getCuttingsIdx();
		const uint32 cutEdges = DualMarchingCell::MC_EDGE_CUTTINGS_TABLE[cuttingsIdx];
		if (0 == cutEdges)
			continue;
		
		findCellIntersections(cutEdges);

		// add triangles for this cell
		for (const int32 *TRIANGLE_EDGES = DualMarchingCell::MC_TRIANGULATION_TABLE[cuttingsIdx];
			 -1 != TRIANGLE_EDGES[0];
			 TRIANGLE_EDGES += 3)
		{
			// triangle must not be a line or point
			if (mCell.isDegeneratedTriangle(TRIANGLE_EDGES))
				continue;

			// add triangle
			uint32 triangle[3];
			for (uint32 localEdgeIdx = 0; localEdgeIdx < 3; ++localEdgeIdx)
			{
				const uint32 cellEdgeIdx = TRIANGLE_EDGES[localEdgeIdx];
				const uint32 *ends = DualMarchingCell::MC_EDGE_ENDS[cellEdgeIdx];

				const uint32 leafIdx0 = mCell.mCornerLeafIndices[ends[0]];
				const uint32 leafIdx1 = mCell.mCornerLeafIndices[ends[1]];
				const Vector3 &p = mCell.mInterPositions[cellEdgeIdx];
				const Real &scale = mCell.mInterScales[cellEdgeIdx];
				const EdgeVertexIndex edgeVertexIndex(leafIdx0, leafIdx1);

				triangle[localEdgeIdx] = addTriangleCorner(p, scale, edgeVertexIndex);
			}

			mSurface.addTriangle(triangle);
		}
	}
}

void DualMarchingCells::fillCellCorners(const Occupancy &occupancy, const Leaves &leaves, const uint32 *dualCellIndices)
{
	// set cell input data for each corner i
	for (uint32 i = 0; i < Nodes::CHILD_COUNT; ++i)
	{
		// get chosen corner cluster evaluation result
		const uint32 cornerIdx = DualMarchingCell::MC_CORNER_REORDERING[i];
		const uint32 leafIdx = dualCellIndices[cornerIdx];
		const Scope &leaf = leaves.getScope(leafIdx);
		const Real distance = occupancy.getOccupancy(leafIdx);

		// set position, distance & leaf index
		mCell.mCornerPositions[i] = leaf.getCenterPosition();
		mCell.mCornerDistances[i] = distance;
		mCell.mCornerScales[i] = leaf.getSize();
		mCell.mCornerLeafIndices[i] = leafIdx;
	}
}

void DualMarchingCells::findCellIntersections(const uint32 cutEdges)
{
	// get isosurface vertices
	// for each edge cut: get intersection position and other interpolated data
	for (uint32 edgeIdx = 0; edgeIdx < DualMarchingCell::CELL_EDGE_COUNT; ++edgeIdx)
	{
		if (0 == (cutEdges & (1 << edgeIdx)))
			continue;

		const Real f = mCell.getInterpolationFactor(edgeIdx);
		const uint32 *ends = DualMarchingCell::MC_EDGE_ENDS[edgeIdx];
		const Vector3 &p0 = mCell.mCornerPositions[ends[0]];
		const Vector3 &p1 = mCell.mCornerPositions[ends[1]];
		const Real &s0 = mCell.mCornerScales[ends[0]];
		const Real &s1 = mCell.mCornerScales[ends[1]];

		mCell.mInterPositions[edgeIdx] = p0 + (p1 - p0) * f;
		mCell.mInterScales[edgeIdx] = s0 + (s1 - s0) * f;
	}
}

uint32 DualMarchingCells::addTriangleCorner(const Vector3 &position, const Real scale, const EdgeVertexIndex &edgeVertexIdx)
{
	// return index of already contained vertex?
	map<EdgeVertexIndex, uint32>::iterator it = mEvittvi.find(edgeVertexIdx);
	if (it != mEvittvi.end())
		return it->second;

	// add vertex 
	const Vector3 color(SURFACE_COLOR.getRed(), SURFACE_COLOR.getGreen(), SURFACE_COLOR.getBlue());
	const Vector3 normal;
	mSurface.addVertex(color, normal, position, scale);

	// remember and return vertex index
	const uint32 index = mSurface.getVertexCount() - 1;
	mEvittvi.insert(make_pair(edgeVertexIdx, index));
	return index;
}

DualMarchingCells::~DualMarchingCells()
{

}

void DualMarchingCells::clear()
{
	mSurface.clear();
	mEvittvi.clear();
}
