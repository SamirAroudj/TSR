/*
 * Copyright (C) 2017 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */
#ifndef _SCENE_TREE_NODES_ITERATOR_H_
#define _SCENE_TREE_NODES_ITERATOR_H_

#include <vector>
#include "Platform/DataTypes.h"
#include "SurfaceReconstruction/Scene/Tree/NodesChecker.h"
#include "SurfaceReconstruction/Scene/Tree/Nodes.h"

// todo comments

namespace SurfaceReconstruction
{
	// forward Declarations
	class Tree;

	class NodesIterator
	{
	public:
		NodesIterator(const Tree &tree, const NodesChecker *nodesChecker = NULL);
		~NodesIterator();

		inline uint32 getDepth() const;

		/** Returns the index of the node (relative to the nodes entered in the constructor) at which this iterator currently points or
			Nodes::INVALID_INDEX if the iterator went through all non-ignored nodes and points at the end.
		@return Returns Nodes::INVALID_INDEX if isAtTheEnd is true or the current node index (index is relative to the entered nodes). */
		inline uint32 getNodeIndex() const;

		/** Returns the scope of the node the iterator currently points at.
		@return The returnded scope identifies and describes the extent of the node at which this iterator currently points.  */
		inline Scope getScope() const;

		/** Returns true if the iterator currently points at a leaf node.
		@return Returns true if the iterator is currently at a leaf node without children.*/
		inline bool isAtLeaf() const;

		/** Returns true if there are no nodes to be traversed anymore.
		@return Returns false if the iterator has not gone over all nodes yet. */
		inline bool isAtTheEnd() const;

		/** Goes through the nodes in depth first order whereas the iterator stops / goes over the parent node first.
			In particular, the order of node traversal is: 0.: current node; 1.: for all children: traverseNodes(child i).\n
			For example, for a tree with only 3 layers (root node, child nodes & grand child nodes):\n
			node,\n
			child 0, grand child 0/0, ..., grand child 0/Nodes::CHILD_COUNT - 1,\n
			child 1, grand child 1/0, ..., grand child 1/Nodes::CHILD_COUNT - 1,\n
			child 2, grand child 2/0, ..., grand child 2/Nodes::CHILD_COUNT - 1, ...\n
		*/
		void goToNext();
		
		/** Goes through the nodes as defined by goToNext().
		@see See goToNext().*/
		inline void operator ++();

	private:
		bool goDown();
		void popStack();

	protected:
		// stack for nodes traversal
		std::vector<uint32> mChildIndices;	/// Stores for each stack layer / tree level the current relative child which is currently traversed.
		std::vector<Scope> mScopes;			/// Stores for each stack layer / tree level the current scope of the node which is currently traversed.
		
		// nodes which are traversed
		const Nodes &mNodes;				/// These nodes are traversed by this iterator.
		const NodesChecker *mChecker;		/// Test whether nodes are ignored during traversal.
	};

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///   inline function definitions   ////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	inline uint32 NodesIterator::getDepth() const
	{
		return (uint32) (mChildIndices.size() - 1);
	}

	inline uint32 NodesIterator::getNodeIndex() const
	{
		if (isAtTheEnd())
			return Nodes::INVALID_INDEX;

		return mScopes.back().getNodeIndex();
	}

	inline Scope NodesIterator::getScope() const
	{
		if (isAtTheEnd())
			return Scope();

		return mScopes.back();
	}
	
	inline bool NodesIterator::isAtLeaf() const
	{
		return mNodes.isLeaf(getNodeIndex());
	}

	inline bool NodesIterator::isAtTheEnd() const
	{
		return mScopes.empty();
	}

	inline void NodesIterator::operator ++()
	{
		goToNext();
	}
}

#endif // _SCENE_TREE_NODES_ITERATOR_H_