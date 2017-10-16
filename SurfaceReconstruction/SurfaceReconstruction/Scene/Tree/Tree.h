/*
 * Copyright (C) 2017 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */
#ifndef _SCENE_TREE_H_
#define _SCENE_TREE_H_

#include "Math/MathHelper.h"
#include "Platform/Storage/Path.h"

// todo comments

namespace SurfaceReconstruction
{
	// Forward declarations
	class DualCells;
	class Leaves;
	class Nodes;
	class Samples;
	class Scope;

	/// This tree data structure subdivides the scene spatially and recursively.
	class Tree
	{
	public:
		Tree(Samples *&reorderedCopys);
		Tree(const Storage::Path &filesBeginning);

		~Tree();

		/** Removes all nodes and destroys the complete tree structure.
			The tree is completely empty after a call to this function.
			Previously used memory for nodes is not released. */
		void clear();

		void eraseSamples(const uint32 *sampleOffsets);
		bool eraseSamplesInNodes(uint32 *sampleOffsets, const uint32 *nodeStates, const uint32 oldSampleCount, const uint8 FLAG_OF_DOOM);
		
		/** todo */
		Math::Vector3 getCenter() const;

		/** todo */
		inline const uint32 getDualCellCount() const;

		/** todo */
		inline const DualCells &getDualCells() const;
	
		inline const Leaves &getLeaves() const;

		void getLeaves(std::vector<uint32> &leaves, const Math::Vector3 &queryPosition, const Real queryRadius) const;

		uint32 getLeafIndex(const Math::Vector3 &queryPosWS) const;
		
		uint32 getNodeDepth(const uint32 nodeIdx) const;

		/** todo */
		inline const Nodes &getNodes() const;

		/** Returns the side length of the root node which is a cube.
		@return Returns the side length of the root node which is a cube. */
		Real getRootNodeSize() const;

		Scope getRootScope() const;

		/** Returns the tree level for a sample, 0 meaning the sample should be inserted into the root, 1 meaning the sample should be inserted into a node which is a direct child of the root, etc.
		@param sampleIdx Identifies the sample for which the proper tree level is returned.
		@return Returns the tree level for a sample, 0 meaning the sample should be inserted into the root, 1 meaning the sample should be inserted into a node which is a direct child of the root, etc. */
		uint32 getSampleDepth(const uint32 sampleIdx) const;

		///** Gathers all samples in sampleSet which have a relative distance d_ij = sample.getRelativeDistance(i, j) <= relativeradius.
		//@param todo*/
		//void gatherSamples(std::vector<uint32> &sampleSet, const uint32 sampleIdx, const Real relativeRadius) const;

		/** todo */
		void saveToFiles(const Storage::Path &fileName) const;

	private:
		Tree();

		/** Copy constructor is forbidden. Don't use it. */
		inline Tree(const Tree &other);

		/** Assignment operator is forbidden. Don't use it.*/
		inline Tree &operator =(const Tree &rhs);

		/** todo */
		void loadFromFile(const Storage::Path &filesBeginning);

	private:
		Leaves *mLeaves;		/// Leaf nodes data.
		Nodes *mNodes;			/// Defines the tree structure.
		DualCells *mDualCells;	/// Cells for dual marching cubes inferred from mNodes.
	};

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///   inline function definitions   ////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		
	inline Tree::Tree(const Tree &other) : Tree()
	{
		assert(false);
	}

	inline Tree &Tree::operator =(const Tree &rhs)
	{
		assert(false);
		return *this;
	}

	inline const DualCells &Tree::getDualCells() const
	{
		return *mDualCells;
	}

	inline const Leaves &Tree::getLeaves() const
	{
		return *mLeaves;
	}

	inline const Nodes &Tree::getNodes() const
	{
		return *mNodes;
	}
}

#endif // _SCENE_TREE_H_
