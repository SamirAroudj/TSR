/*
 * Copyright (C) 2017 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */
#ifndef _SCENE_TREE_DUAL_CELLS_H_
#define _SCENE_TREE_DUAL_CELLS_H_

#include <string>
#include <vector>
#include "Platform/DataTypes.h"

// todo comments

namespace SurfaceReconstruction
{
	// forward declarations
	class Leaves;
	class Nodes;

	class DualCells
	{
	public:
		DualCells(const Nodes &nodes, const Leaves &leaves);
		DualCells(const Nodes &nodes, const Leaves &leaves, const Storage::Path &fileName);
		~DualCells();

		void clear();

		inline uint32 getCellCount() const;
		inline uint32 getIndexCount() const;
		inline const uint32 *getIndices() const;

		void loadFromFile(const Storage::Path &fileName);
		void saveToFile(const Storage::Path &fileName) const;

	protected:
		void createDualCellsByNode(const uint32 nodeIdx);

		void createDualCellsBy2AlongX(const uint32 nodeIdx0, const uint32 nodeIdx1);
		void createDualCellsBy2AlongY(const uint32 nodeIdx0, const uint32 nodeIdx1);
		void createDualCellsBy2AlongZ(const uint32 nodeIdx0, const uint32 nodeIdx1);

		void processFacesInYZ(const uint32 children[8]);
		void processFacesInXZ(const uint32 children[8]);
		void processFacesInXY(const uint32 children[8]);

		void createDualCellsBy4InXY(const uint32 nodeIdx0, const uint32 nodeIdx1, const uint32 nodeIdx2, const uint32 nodeIdx3);
		void createDualCellsBy4InXZ(const uint32 nodeIdx0, const uint32 nodeIdx1, const uint32 nodeIdx2, const uint32 nodeIdx3);
		void createDualCellsBy4InYZ(const uint32 nodeIdx0, const uint32 nodeIdx1, const uint32 nodeIdx2, const uint32 nodeIdx3);
		
		void processEdgesAlongZ(const uint32 children[8]);
		void processEdgesAlongY(const uint32 children[8]);
		void processEdgesAlongX(const uint32 children[8]);

		void createDualCellsByVertex(const uint32 nodeIndices[8]);

	public:
		static const uint32 FILE_VERSION;

	private:
		const Leaves &mLeaves;			/// Dual cells are created using these leaves.
		const Nodes &mNodes;			/// Dual cells are created using these nodes.
		std::vector<uint32> mIndices;	/// Each cell is a cube consisting of 8 corners which are represented by leaf node indices.
	};

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///   inline function definitions   ////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	inline uint32 DualCells::getCellCount() const
	{
		return getIndexCount() / 8;
	}

	inline uint32 DualCells::getIndexCount() const
	{
		return (uint32) mIndices.size();
	}

	inline const uint32 *DualCells::getIndices() const
	{
		return mIndices.data();
	}
}

#endif // _SCENE_TREE_DUAL_CELLS_H_