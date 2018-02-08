/*
 * Copyright (C) 2018 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */
#ifndef _SCENE_TREE_NODES_H_
#define _SCENE_TREE_NODES_H_

#include <vector>
#include "Math/Vector3.h"
#include "Platform/DataTypes.h"
#include "Platform/Storage/Path.h"
#include "SurfaceReconstruction/Scene/Tree/Scope.h"

// todo comments

namespace SurfaceReconstruction
{
	// forward declarations
	class Occupancy;
	class Samples;
	struct ViewCone;

	/// This class models objects which are single parts / nodes of Tree instances. 
	class Nodes
	{
	public:
		/** only checks the position
		todo */
		static inline bool contains(const Math::Vector3 &positionWS, const Math::Vector3 &nodePosWS, const Real size, const Real tolerance = 0.0f);
		static uint32 getChildIndexForOffset(const Math::Vector3 &offset);

		/** todo
		@param stepSize If stepSize is the child node size then the child node coords of the node at nodeCoordsWS are returned.
			If stepSize is the nodesize itself then the coordinates of the corners/vertices of the node at nodeCoordsWS are returned. */
		static Math::Vector3 getCoords(const Math::Vector3 &nodeCoordsWS, const Real stepSize, uint32 index);

		static Math::Vector3 getSideCenterPositionWS(const Scope &scope, const uint32 sideIdx, const Real stepSize);
		static bool isContainer(const Scope &scope, const uint32 sampleIdx);
		static inline bool isInvalidNodeIdx(const uint32 nodeIdx);

	public:
		Nodes(Samples *&reorderdSamples, const Scope &rootScope);
		Nodes(const Storage::Path &fileName);

		/** todo */
		void checkSamplesOrder(const Scope &rootScope) const;

		/** todo */
		void checkSampleIndicesOfNodes() const;

		void clear();

		/** todo */
		void eraseSamples(const uint32 *sampleOffsets);
		
		/** todo */
		bool eraseSamplesInNodes(uint32 *sampleOffsets, const uint32 *nodeStates, const uint32 oldSampleCount, const uint8 FLAG_OF_DOOM);

		/** todo scope -> nodeIdx, nodeCoordsWS, nodeSize are set to the values of the lastly processed node which is either the closest tree end on failure or the containing node on search success.
		@param scope todo */
		bool findContainer(Scope &scope, const uint32 sampleIdx) const;
		
		inline uint32 getChildBlock(const uint32 nodeIdx) const;
		
		inline Scope getChildScope(const Scope &scope, const uint32 childIdx) const;
		inline Scope getChildScope(const Scope &scope, const uint32 childIdx, const Real childSize) const;

		inline uint32 getCount() const;

		uint32 getDepth(const uint32 nodeIdx) const;

		void getLeafNodes(std::vector<uint32> &leafNodes,
			const Math::Vector3 &queryPosition, const Real queryRadius, const Scope &scope) const;

		uint32 getNode(Scope &scope, const Math::Vector3 &containedPositionWS, const uint32 maxDepth) const;

		inline uint32 getSampleCount(const uint32 nodeIdx) const;
		
		uint32 getSamples(uint32 &sampleCount, const uint32 nodeIdx) const;

		bool intersect(const Scope &scope, bool positiveSide, const uint32 sampleIdx) const;

		inline bool isLeaf(const uint32 nodeIdx) const;
	
		void loadFromFile(const Storage::Path &fileName);
		void saveToFile(const Storage::Path &fileName) const;

	private:

		/** Balances the empty tree structure in order to have at maximum 1 depth level difference for each pair of adjacent leaf nodes. 
		@param rootScope todo*/
		void balance(const Scope &rootScope);

		uint32 computeReorderedAddresses(std::vector<uint32> &newOrder,
			const uint32 nodeIdx, const uint32 nextFreeAddress) const;

		/** todo */
		void createNodes(const Scope &rootScope);

		bool createNodesForBalancing(const uint32 depth, const Scope &scope, const Scope &rootScope);

		/** todo scope -> nodeIdx, nodeCoordsWS, nodeSize are set to the values of the lastly processed node which is the newly created or found node which contains the entered sample.
		@param scope todo */
		void createNodesForContainment(Scope &scope, const uint32 sampleIdx);
		uint32 createNodesForSampling(Scope scope, const bool positiveSide, const uint32 sampleIdx);
		void createChildren(const uint32 currentNodeIdx);

		/** todo does not allocate additional memory but is slower. */
		void eraseSamplesInNodesInSitu(Samples &samples, const uint32 *nodeStates, const uint8 FLAG_OF_DOOM);

		uint32 findReliableSamplingNode(const Scope &rootScope, const bool positiveSide, const uint32 sampleIdx) const;

		bool fittingSamplingNodeCenter(const Math::Vector3 &nodeCoordsWS, const Real childSize, const bool positiveSide, const uint32 sampleIdx) const;

		void replaceChildBlock(uint32 **targets, const uint32 targetsOffsetIdx, const uint32 *const *sources, const uint32 sourcesOffset, const uint32 parentIdx);
		
		/** Reorders nodes in memory for higher cache coherence.
			Also Sets sample start and end indices (sample area in memory) for each node according to the NEW ORDER. */
		void reorder();

		void reorder(const std::vector<uint32> &newOrder);

		/** todo *
		@return Returns a copy of the scene samples, but reordered in memory according to tree structure. */
		Samples *reorderSamplesAndFillNodes(const Scope &rootScope);

		/** Reserves memory for nodeCount nodes.
		@param nodeCount Memory for nodeCount nodes is reserved. */
		void reserve(const uint32 nodeCount);

		/** Resizes arrays which contain nodes data.
		@param newCount Set this to the new number of nodes. */
		void resize(const uint32 newNodeCount);

	public:
		static const uint32	CHILD_COUNT = 8;
		static const uint32 FILE_VERSION;
		static const uint32 INVALID_INDEX;
		static const uint32 SIDE_COUNT = 6;

	private:
		// data for nodes themselves, nodeIdx = 0 is always the root
		std::vector<uint32> mChildren;				/// For each node: link to its block of Tree::CHILD_COUNT children / link to its first child, all other indices directly follow in the same memory block.
		std::vector<uint32> mParents;				/// For each node: link to its parent node or Nodes::INVALID_NODE_IDX for the root node with index 0
		std::vector<uint32> mSamplesPerNodes;		/// For each node: number of its contained samples, see mSampleStartIndices
		std::vector<uint32> mSampleStartIndices;	/// For each node: start index of its contained samples w.r.t. class Samples, see mSamplesPerNodes
	};

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///   inline function definitions   ////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	inline bool Nodes::contains(const Math::Vector3 &positionWS, const Math::Vector3 &nodePosWS, const Real size, const Real tolerance)
	{
		return	(positionWS.x >= nodePosWS.x  - tolerance &&
				 positionWS.x < nodePosWS.x + size + tolerance &&
				 positionWS.y >= nodePosWS.y  - tolerance &&
				 positionWS.y < nodePosWS.y + size + tolerance &&
				 positionWS.z >= nodePosWS.z  - tolerance &&
				 positionWS.z < nodePosWS.z + size + tolerance);
	}

	inline bool Nodes::isInvalidNodeIdx(const uint32 nodeIdx)
	{
		return (INVALID_INDEX == nodeIdx);
	}

	inline uint32 Nodes::getChildBlock(const uint32 nodeIdx) const
	{
		return mChildren[nodeIdx];
	}

	inline Scope Nodes::getChildScope(const Scope &scope, const uint32 childIdx) const
	{
		const Real childSize = scope.getSize() * 0.5f;
		const Math::Vector3 childCoords = getCoords(scope.getMinimumCoordinates(), childSize, childIdx);
		const uint32 globalChildIdx = childIdx + mChildren[scope.getNodeIndex()];
	
		assert(globalChildIdx < getCount());
		return Scope(childCoords, childSize, globalChildIdx);
	}

	inline Scope Nodes::getChildScope(const Scope &scope, const uint32 childIdx, const Real childSize) const
	{
		const Math::Vector3 childCoords = getCoords(scope.getMinimumCoordinates(), childSize, childIdx);
		const uint32 globalChildIdx = childIdx + mChildren[scope.getNodeIndex()];
	
		assert(globalChildIdx < getCount());
		return Scope(childCoords, childSize, globalChildIdx);
	}

	inline uint32 Nodes::getCount() const
	{
		return (uint32) mChildren.size();
	}
	
	inline uint32 Nodes::getSampleCount(const uint32 nodeIdx) const
	{
		return mSamplesPerNodes[nodeIdx];
	}

	inline bool Nodes::isLeaf(const uint32 nodeIdx) const
	{
		if (Nodes::isInvalidNodeIdx(nodeIdx))
			return false;
		return Nodes::isInvalidNodeIdx(mChildren[nodeIdx]);
	}
}

#endif // _SCENE_TREE_NODES_H_
