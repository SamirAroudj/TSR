/*
 * Copyright (C) 2018 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */

#include "SurfaceReconstruction/Scene/Tree/LeavesIterator.h"
#include "SurfaceReconstruction/Scene/Tree/Tree.h"

using namespace std;
using namespace SurfaceReconstruction;

LeavesIterator::LeavesIterator(const Tree &tree, const NodesChecker *checker) :
	NodesIterator(tree, checker), mLeaves(tree.getLeaves())
{
	// don't start/stay at a leaf
	if (!isAtLeaf())
		goToNext();
}

void LeavesIterator::goToNext() 
{
	while (!isAtTheEnd())
	{
		NodesIterator::goToNext();
		if (isAtLeaf())
			break;
	}
}