/*
 * Copyright (C) 2018 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */
#ifndef _SCENE_TREE_LEAVES_ITERATOR_H_
#define _SCENE_TREE_LEAVES_ITERATOR_H_

#include "Platform/DataTypes.h"
#include "SurfaceReconstruction/Scene/Tree/Leaves.h"
#include "SurfaceReconstruction/Scene/Tree/NodesIterator.h"

// todo comments

namespace SurfaceReconstruction
{
	// forward declarations
	class Tree;

	class LeavesIterator : public NodesIterator
	{
	public:
		LeavesIterator(const Tree &tree, const NodesChecker *nodesChecker = NULL);

		inline uint32 getLeafIndex() const;

		/** Goes through the nodes as Nodes iterator but only stops at leaf nodes and not at inner nodes.
		@see See class NodesIterator. */
		void goToNext();

		/** Goes through the nodes as defined by goToNext().
		@see See goToNext().*/
		inline void operator ++();

	private:
		const Leaves &mLeaves;
	};

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///   inline function definitions   ////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	inline uint32 LeavesIterator::getLeafIndex() const
	{
		return mLeaves.getLeafIndex(getNodeIndex());
	}

	inline void LeavesIterator::operator ++()
	{
		LeavesIterator::goToNext();
	}
}

#endif // _SCENE_TREE_LEAVES_ITERATOR_H_