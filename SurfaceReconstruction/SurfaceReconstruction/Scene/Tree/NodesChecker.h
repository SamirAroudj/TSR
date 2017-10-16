/*
 * Copyright (C) 2017 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */
#ifndef _SCENE_TREE_I_NODES_CHECKER_H_
#define _SCENE_TREE_I_NODES_CHECKER_H_

namespace SurfaceReconstruction
{
	// forward declarations
	class Scope;

	/// Interface for node ignorance checks.
	/** This is an interface to define the fucntiosn for testing whether a node should be ignored or not.
		For example, this interface is used by node iterators during traversal to check whether to go over a specific node or not. */
	class NodesChecker
	{
	public:
		inline NodesChecker(const uint32 maxDepth);

		/** Virtual destructor for safe programming. */
		inline virtual ~NodesChecker();

		inline uint32 getMaxDepth() const;
		
		/** Tests whether a node should be ignored or not, for example, during traversal of nodes.
		@param scope Identifies and describes the extent of the node which is tested for ignorance, for example, during nodes traversal.
		@param isLeaf This should be set to true if scope is the scope of a leaf.
		@return Should return true if the node shall be ignored.
			Should return false if the node shall not be ignored but processed normally. */
		virtual bool isIgnored(const Scope &scope, const bool isLeaf, const uint32 depth) const = 0;

	protected:
		uint32 mMaxDepth;
	};

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///   inline function definitions   ////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	inline NodesChecker::NodesChecker(const uint32 maxDepth) :
		mMaxDepth(maxDepth)
	{
		
	}
	
	inline NodesChecker::~NodesChecker()
	{
		
	}

	inline uint32 NodesChecker::getMaxDepth() const
	{
		return mMaxDepth;
	}
}

#endif // _SCENE_TREE_I_NODES_CHECKER_H_
