/*
 * Copyright (C) 2018 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */

#include "Platform/Storage/File.h"
#include "SurfaceReconstruction/Scene/Tree/DualCells.h"
#include "SurfaceReconstruction/Scene/Tree/Leaves.h"
#include "SurfaceReconstruction/Scene/Tree/Nodes.h"

using namespace Platform;
using namespace Storage;
using namespace SurfaceReconstruction;
using namespace std;

const uint32 DualCells::FILE_VERSION = 0;

DualCells::DualCells(const Nodes &nodes, const Leaves &leaves) :
	mNodes(nodes), mLeaves(leaves)
{
	cout << "Creating scene octree dual cells." << endl;
	clear();

	// create Dual Octree
	mIndices.reserve(mLeaves.getCount() * 8);
	createDualCellsByNode(0);
	mIndices.shrink_to_fit();
}
	
DualCells::DualCells(const Nodes &nodes, const Leaves &leaves, const Path &fileName) :
	mLeaves(leaves), mNodes(nodes)
{
	loadFromFile(fileName);
}

DualCells::~DualCells()
{
	clear();
}

void DualCells::clear()
{
	mIndices.clear();
}

void DualCells::createDualCellsByNode(const uint32 nodeIdx)
{
	// Recursive Dual Octree Cell creation according to:
	// Schaefer, Scott, and Joe Warren
	// "Dual marching cubes: Primal contouring of dual grids."
	// PG (Pacific Graphics) 2004.
	// Proceedings of IEEE 12th Pacific Conference on Computer Graphics and Applications, 2004.
	if (mNodes.isLeaf(nodeIdx))
		return;

	const uint32 child0 = mNodes.getChildBlock(nodeIdx);
	const uint32 children[8] = { child0, child0 + 1, child0 + 2, child0 + 3, child0 + 4, child0 + 5, child0 + 6, child0 + 7 };

	for (uint32 i = 0; i < Nodes::CHILD_COUNT; ++i)
		createDualCellsByNode(children[i]);
	
	processFacesInYZ(children);
	processFacesInXZ(children);
	processFacesInXY(children);

	processEdgesAlongZ(children);
	processEdgesAlongY(children);
	processEdgesAlongX(children);
	
	createDualCellsByVertex(children);
}

void DualCells::createDualCellsBy2AlongX(const uint32 nodeIdx0, const uint32 nodeIdx1)
{	
	// children?
	bool isLeaf[2] =
	{
		mNodes.isLeaf(nodeIdx0),
		mNodes.isLeaf(nodeIdx1)
	};

	if (isLeaf[0] && isLeaf[1])
		return;

	const uint32 children[8] =
	{
		isLeaf[0] ? nodeIdx0 : mNodes.getChildBlock(nodeIdx0) + 1,
		isLeaf[1] ? nodeIdx1 : mNodes.getChildBlock(nodeIdx1) + 0,
		isLeaf[0] ? nodeIdx0 : mNodes.getChildBlock(nodeIdx0) + 3,
		isLeaf[1] ? nodeIdx1 : mNodes.getChildBlock(nodeIdx1) + 2,
		isLeaf[0] ? nodeIdx0 : mNodes.getChildBlock(nodeIdx0) + 5,
		isLeaf[1] ? nodeIdx1 : mNodes.getChildBlock(nodeIdx1) + 4,
		isLeaf[0] ? nodeIdx0 : mNodes.getChildBlock(nodeIdx0) + 7,
		isLeaf[1] ? nodeIdx1 : mNodes.getChildBlock(nodeIdx1) + 6,
	};

	processFacesInYZ(children);

	processEdgesAlongZ(children);
	processEdgesAlongY(children);

	createDualCellsByVertex(children);
}

void DualCells::createDualCellsBy2AlongY(const uint32 nodeIdx0, const uint32 nodeIdx1)
{
	// children?
	bool isLeaf[2] =
	{
		mNodes.isLeaf(nodeIdx0),
		mNodes.isLeaf(nodeIdx1)
	};

	if (isLeaf[0] && isLeaf[1])
		return;

	const uint32 children[8] =
	{
		isLeaf[1] ? nodeIdx1 : mNodes.getChildBlock(nodeIdx1) + 2,
		isLeaf[1] ? nodeIdx1 : mNodes.getChildBlock(nodeIdx1) + 3,
		isLeaf[0] ? nodeIdx0 : mNodes.getChildBlock(nodeIdx0) + 0,
		isLeaf[0] ? nodeIdx0 : mNodes.getChildBlock(nodeIdx0) + 1,
		isLeaf[1] ? nodeIdx1 : mNodes.getChildBlock(nodeIdx1) + 6,
		isLeaf[1] ? nodeIdx1 : mNodes.getChildBlock(nodeIdx1) + 7,
		isLeaf[0] ? nodeIdx0 : mNodes.getChildBlock(nodeIdx0) + 4,
		isLeaf[0] ? nodeIdx0 : mNodes.getChildBlock(nodeIdx0) + 5
	};

	processFacesInXZ(children);

	processEdgesAlongZ(children);
	processEdgesAlongX(children);

	createDualCellsByVertex(children);
}

void DualCells::createDualCellsBy2AlongZ(const uint32 nodeIdx0, const uint32 nodeIdx1)
{
	// children?
	bool isLeaf[2] =
	{
		mNodes.isLeaf(nodeIdx0),
		mNodes.isLeaf(nodeIdx1)
	};

	if (isLeaf[0] && isLeaf[1])
		return;

	const uint32 children[8] =
	{
		isLeaf[0] ? nodeIdx0 : mNodes.getChildBlock(nodeIdx0) + 4,
		isLeaf[0] ? nodeIdx0 : mNodes.getChildBlock(nodeIdx0) + 5,
		isLeaf[0] ? nodeIdx0 : mNodes.getChildBlock(nodeIdx0) + 6,
		isLeaf[0] ? nodeIdx0 : mNodes.getChildBlock(nodeIdx0) + 7,
		isLeaf[1] ? nodeIdx1 : mNodes.getChildBlock(nodeIdx1) + 0,
		isLeaf[1] ? nodeIdx1 : mNodes.getChildBlock(nodeIdx1) + 1,
		isLeaf[1] ? nodeIdx1 : mNodes.getChildBlock(nodeIdx1) + 2,
		isLeaf[1] ? nodeIdx1 : mNodes.getChildBlock(nodeIdx1) + 3
	};

	processFacesInXY(children);
	
	processEdgesAlongY(children);
	processEdgesAlongX(children);

	createDualCellsByVertex(children);
}

void DualCells::createDualCellsBy4InXY(const uint32 nodeIdx0, const uint32 nodeIdx1, const uint32 nodeIdx2, const uint32 nodeIdx3)
{
	// children?
	bool isLeaf[4] =
	{
		mNodes.isLeaf(nodeIdx0),
		mNodes.isLeaf(nodeIdx1),
		mNodes.isLeaf(nodeIdx2),
		mNodes.isLeaf(nodeIdx3)
	};

	if (isLeaf[0] && isLeaf[1] && isLeaf[2] && isLeaf[3])
		return;

	const uint32 children[8] =
	{
		isLeaf[0] ? nodeIdx0 : mNodes.getChildBlock(nodeIdx0) + 3,
		isLeaf[1] ? nodeIdx1 : mNodes.getChildBlock(nodeIdx1) + 2,
		isLeaf[2] ? nodeIdx2 : mNodes.getChildBlock(nodeIdx2) + 1,
		isLeaf[3] ? nodeIdx3 : mNodes.getChildBlock(nodeIdx3) + 0,
		isLeaf[0] ? nodeIdx0 : mNodes.getChildBlock(nodeIdx0) + 7,
		isLeaf[1] ? nodeIdx1 : mNodes.getChildBlock(nodeIdx1) + 6,
		isLeaf[2] ? nodeIdx2 : mNodes.getChildBlock(nodeIdx2) + 5,
		isLeaf[3] ? nodeIdx3 : mNodes.getChildBlock(nodeIdx3) + 4
	};

	processEdgesAlongZ(children);
	createDualCellsByVertex(children);
}

void DualCells::createDualCellsBy4InXZ(const uint32 nodeIdx0, const uint32 nodeIdx1, const uint32 nodeIdx2, const uint32 nodeIdx3)
{  
	// children?
	bool isLeaf[4] =
	{
		mNodes.isLeaf(nodeIdx0),
		mNodes.isLeaf(nodeIdx1),
		mNodes.isLeaf(nodeIdx2),
		mNodes.isLeaf(nodeIdx3)
	};

	if (isLeaf[0] && isLeaf[1] && isLeaf[2] && isLeaf[3])
		return;

	const uint32 children[8] =
	{
		isLeaf[0] ? nodeIdx0 : mNodes.getChildBlock(nodeIdx0) + 5,
		isLeaf[1] ? nodeIdx1 : mNodes.getChildBlock(nodeIdx1) + 4,
		isLeaf[0] ? nodeIdx0 : mNodes.getChildBlock(nodeIdx0) + 7,
		isLeaf[1] ? nodeIdx1 : mNodes.getChildBlock(nodeIdx1) + 6,
		isLeaf[2] ? nodeIdx2 : mNodes.getChildBlock(nodeIdx2) + 1,
		isLeaf[3] ? nodeIdx3 : mNodes.getChildBlock(nodeIdx3) + 0,
		isLeaf[2] ? nodeIdx2 : mNodes.getChildBlock(nodeIdx2) + 3,
		isLeaf[3] ? nodeIdx3 : mNodes.getChildBlock(nodeIdx3) + 2 
	};

	processEdgesAlongY(children);
	createDualCellsByVertex(children);
}

void DualCells::createDualCellsBy4InYZ(const uint32 nodeIdx0, const uint32 nodeIdx1, const uint32 nodeIdx2, const uint32 nodeIdx3)
{
	// children?
	bool isLeaf[4] =
	{
		mNodes.isLeaf(nodeIdx0),
		mNodes.isLeaf(nodeIdx1),
		mNodes.isLeaf(nodeIdx2),
		mNodes.isLeaf(nodeIdx3)
	};

	if (isLeaf[0] && isLeaf[1] && isLeaf[2] && isLeaf[3])
		return;

	const uint32 children[8] =
	{
		isLeaf[0] ? nodeIdx0 : mNodes.getChildBlock(nodeIdx0) + 6,
		isLeaf[0] ? nodeIdx0 : mNodes.getChildBlock(nodeIdx0) + 7,
		isLeaf[1] ? nodeIdx1 : mNodes.getChildBlock(nodeIdx1) + 4,
		isLeaf[1] ? nodeIdx1 : mNodes.getChildBlock(nodeIdx1) + 5,
		isLeaf[2] ? nodeIdx2 : mNodes.getChildBlock(nodeIdx2) + 2,
		isLeaf[2] ? nodeIdx2 : mNodes.getChildBlock(nodeIdx2) + 3,
		isLeaf[3] ? nodeIdx3 : mNodes.getChildBlock(nodeIdx3) + 0,
		isLeaf[3] ? nodeIdx3 : mNodes.getChildBlock(nodeIdx3) + 1
	};

	processEdgesAlongX(children);
	createDualCellsByVertex(children);
}

void DualCells::processEdgesAlongZ(const uint32 children[8])
{
	// 2 times: process neighbors of 4 nodes in xz-plane within neighbors of 4 parent nodes (4 nodes around an inner edge in y direction)
	// continue with 4 inner nodes in the back and in the front half of inner 4 neighbors of children of 4 adjacent nodes
	createDualCellsBy4InXY(children[0], children[1], children[2], children[3]);
	createDualCellsBy4InXY(children[4], children[5], children[6], children[7]);
}

void DualCells::processEdgesAlongY(const uint32 children[8])
{
	// 2 times: process neighbors of 4 nodes in xz-plane within neighbors of 4 parent nodes (4 nodes around an inner edge in y direction)
	// continue with 4 inner nodes in the bottom and in the top half of inner 4 neighbors of children of 4 adjacent nodes
	createDualCellsBy4InXZ(children[0], children[1], children[4], children[5]);
	createDualCellsBy4InXZ(children[2], children[3], children[6], children[7]);
}

void DualCells::processEdgesAlongX(const uint32 children[8])
{
	// 2 times: process neighbors of 4 nodes in yz-plane within neighbors of 4 parent nodes (4 nodes around an inner edge in x direction)
	// continue with 4 inner nodes in the left and in the right half of inner 4 neighbors of children of 4 adjacent nodes
	createDualCellsBy4InYZ(children[0], children[2], children[4], children[6]);
	createDualCellsBy4InYZ(children[1], children[3], children[5], children[7]);
}

void DualCells::processFacesInYZ(const uint32 children[8])
{
	// process all 4 inner faces which are in the yz-plane
	// back, bottom; front, bottom; back, top; front, top 
	createDualCellsBy2AlongX(children[0], children[1]);
	createDualCellsBy2AlongX(children[4], children[5]);
	createDualCellsBy2AlongX(children[2], children[3]);
	createDualCellsBy2AlongX(children[6], children[7]);
}

void DualCells::processFacesInXZ(const uint32 children[8])
{
	// process all 4 inner faces which are in the xz-plane
	// back, left; back right; front left; front right 
	createDualCellsBy2AlongY(children[2], children[0]);
	createDualCellsBy2AlongY(children[3], children[1]);
	createDualCellsBy2AlongY(children[6], children[4]);
	createDualCellsBy2AlongY(children[7], children[5]);
}

void DualCells::processFacesInXY(const uint32 children[8])
{
	// process all 4 inner faces which are in the xy-plane
	// bottom, left; bottom, right; top, left; top right
	createDualCellsBy2AlongZ(children[0], children[4]);
	createDualCellsBy2AlongZ(children[1], children[5]);
	createDualCellsBy2AlongZ(children[2], children[6]);
	createDualCellsBy2AlongZ(children[3], children[7]);
}

void DualCells::createDualCellsByVertex(const uint32 nodeIndices[8])
{
	// children?
	bool isLeaf[Nodes::CHILD_COUNT];
	bool onlyLeaves = true;

	for (uint32 i = 0; i < Nodes::CHILD_COUNT; ++i)
	{
		isLeaf[i] = mNodes.isLeaf(nodeIndices[i]);
		onlyLeaves &= isLeaf[i];
	}

	if (onlyLeaves)
	{
		const vector<uint32> &nodeToLeafLinks = mLeaves.getNodeToLeafLinks();

		for (uint32 i = 0; i < Nodes::CHILD_COUNT; ++i)
		{
			const uint32 leafIdx = nodeToLeafLinks[nodeIndices[i]];
			mIndices.push_back(leafIdx);
		}
		//createDualCellsAtBorder(nodeIndices);

		return;
	}

	// continue recursively with the right nodes (8 nodes adjacent to the very middle of the current 4x4x4 neighbors of nodeIndices including their children (2x2x2 subdivided))
	const uint32 children[8] =
	{
		isLeaf[0] ? nodeIndices[0] : mNodes.getChildBlock(nodeIndices[0]) + 7,
		isLeaf[1] ? nodeIndices[1] : mNodes.getChildBlock(nodeIndices[1]) + 6,
		isLeaf[2] ? nodeIndices[2] : mNodes.getChildBlock(nodeIndices[2]) + 5,
		isLeaf[3] ? nodeIndices[3] : mNodes.getChildBlock(nodeIndices[3]) + 4,
		isLeaf[4] ? nodeIndices[4] : mNodes.getChildBlock(nodeIndices[4]) + 3,
		isLeaf[5] ? nodeIndices[5] : mNodes.getChildBlock(nodeIndices[5]) + 2,
		isLeaf[6] ? nodeIndices[6] : mNodes.getChildBlock(nodeIndices[6]) + 1,
		isLeaf[7] ? nodeIndices[7] : mNodes.getChildBlock(nodeIndices[7]) + 0
	};
	
	// process 8 nodes adjacent to very inner vertex
	createDualCellsByVertex(children);
}

void DualCells::loadFromFile(const Path &fileName)
{
	// clean previous data
	uint32 indexCount = 0;
	clear();

	// open file
	File file(fileName, File::OPEN_READING, true, FILE_VERSION);

	// read data size & reserve memory
	file.read(&indexCount, sizeof(uint32), sizeof(uint32), 1);
	mIndices.resize(indexCount);

	// load actual cells
	file.read(mIndices.data(), sizeof(uint32) * indexCount, sizeof(uint32), indexCount);
	
	cout << "Loaded " << getCellCount() << " dual cells.\n";
}

void DualCells::saveToFile(const Path &fileName) const
{
	const uint32 indexCount = getIndexCount();

	File file(fileName, File::CREATE_WRITING, true, FILE_VERSION);
	file.write(&indexCount, sizeof(uint32), 1);
	file.write(mIndices.data(), sizeof(uint32), indexCount);
}
