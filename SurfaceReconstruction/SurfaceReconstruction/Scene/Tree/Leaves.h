/*
 * Copyright (C) 2017 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */
#ifndef _SCENE_TREE_LEAVES_H_
#define _SCENE_TREE_LEAVES_H_

#include <string>
#include <vector>
#include "Platform/Storage/Path.h"
#include "SurfaceReconstruction/Scene/Tree/Scope.h"

// todo comments

namespace SurfaceReconstruction
{
	// forward declarations
	class Nodes;

	class Leaves
	{
	public:
		static inline bool isInvalidLeafIndex(const uint32 index);

	public:
		Leaves(const Nodes &nodes, const Scope &rootScope);
		Leaves(const Nodes &nodes, const Storage::Path &fileName);
		~Leaves();

		void clear();
		
		inline uint32 getCount() const;

		inline uint32 getLeafIndex(const uint32 nodeIndex) const;

		/** Returns the index of the leaf node which contains queryPosWS. 
		@param queryPosWS todo.
		@return Returns the index of the leaf node which contains queryPosWS.  */
		uint32 getLeafIndex(const Scope &rootScope, const Math::Vector3 &queryPosWS) const;

		void getLeaves(std::vector<uint32> &leaves,
			const Math::Vector3 &queryPosition, const Real queryRadius, const Scope &scope) const;

		/** todo */
		inline const uint32 *getNeighbors(uint32 &size, const uint32 leafIdx) const;

		/** todo */
		inline const uint32 *getNeighborsOffsets() const;

		/** todo */
		inline const uint32 *getNeighbors() const;

		inline const std::vector<uint32> &getNodeToLeafLinks() const;
		
		/** todo */
		inline void getScopeCenterPositions(Math::Vector3 leafCenterPositions[2], const uint32 leafIdx0, const uint32 leafIdx1) const;

		inline const Scope &getScope(const uint32 leafIdx) const;
		inline const std::vector<Scope> &getScopes() const;

		void getScopes(std::vector<uint32> &foundLeafScopes,
			const Math::Vector3 &queryPosition, const Real queryRadius,
			const Scope &scope) const;

		void loadFromFile(const Storage::Path &fileName);
		void saveToFile(const Storage::Path &fileName) const;

	private:
		Leaves(const Nodes &nodes);
		void createScopes(const Scope &rootScope);
		void gatherScopes(const Scope &scope);

		// leaf neighbors
		void createNeighborsEdges(const Scope &rootScope);
		uint32 getUnresponsibleEdgeNeighbor(const Scope &root, const Scope &leaf, const uint32 sideIdx) const;
		bool insertNeighborsEdge(const uint32 leafIdx, const uint32 leafNeighborIdx);

	public:
		static const uint32 FILE_VERSION;
		static const uint32 INVALID_INDEX;

	private:
		const Nodes &mNodes;

		// links: standard tree nodes -> mLeaves
		std::vector<uint32> mNodeToLeafLinks;	/// One index for each node as link to its corresponding leaf node in mLeaves or Nodes::INVALID_NODE_IDX if the node is not a leaf.

		// leaves
		std::vector<Scope> mScopes;				/// Contains the scopes (spatial extents & memory indices) of all tree leaves.

		// leaf neighbors
		uint32 *mNeighborsOffsets;			/// Each leaf i has n_i neighbors described by the edges in mNeighbors[offsets[i]] to mNeighbors[offsets[i+1]-1].
												/// (offsets[i+1] - offsets[i]) = n_i). Contains #leaves + 1 offsets.
		uint32 *mNeighbors;					/// Contains for each leaf mLeaves[n_i] its neighbors which is a set of edges starting at it and going to its adjacent leaves mLeaves[{n_ij}].
												/// The indices of the adjacent leaves {n_j} are stored for each leaf n_i  (Order: neighbors of leaf 0 (n_00, n_01, ..., n_0k), neighbors of leaf 1 (n_10, ...
	};

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///   inline function definitions   ////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	inline bool Leaves::isInvalidLeafIndex(const uint32 index)
	{
		return (INVALID_INDEX == index);
	}

	inline uint32 Leaves::getCount() const
	{
		return (uint32) mScopes.size();
	}

	inline uint32 Leaves::getLeafIndex(const uint32 nodeIndex) const
	{
		if (INVALID_INDEX == nodeIndex)
			return INVALID_INDEX;
		return mNodeToLeafLinks[nodeIndex];
	}

	inline void Leaves::getScopeCenterPositions(Math::Vector3 leafCenterPositions[2], const uint32 leafIdx0, const uint32 leafIdx1) const
	{
		const Scope &leaf0 = mScopes[leafIdx0];
		const Scope &leaf1 = mScopes[leafIdx1];

		leafCenterPositions[0] = leaf0.getCenterPosition();
		leafCenterPositions[1] = leaf1.getCenterPosition();
	}

	inline const uint32 *Leaves::getNeighbors(uint32 &size, const uint32 leafIdx) const
	{
		assert(leafIdx < getCount());
		const uint32 start	= mNeighborsOffsets[leafIdx];
		const uint32 end	= mNeighborsOffsets[leafIdx + 1];

		size = end - start;
		return mNeighbors + start;
	}

	inline const uint32 *Leaves::getNeighborsOffsets() const
	{
		return mNeighborsOffsets;
	}

	inline const uint32 *Leaves::getNeighbors() const
	{
		return mNeighbors;
	}

	inline const std::vector<uint32> &Leaves::getNodeToLeafLinks() const
	{
		return mNodeToLeafLinks;
	}
	
	inline const Scope &Leaves::getScope(const uint32 leafIdx) const
	{
		return mScopes[leafIdx];
	}

	inline const std::vector<Scope> &Leaves::getScopes() const
	{
		return mScopes;
	}
}

#endif // _SCENE_TREE_LEAVES_H_