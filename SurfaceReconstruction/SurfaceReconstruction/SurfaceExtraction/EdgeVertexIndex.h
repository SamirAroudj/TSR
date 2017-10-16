/*
 * Copyright (C) 2017 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */

#ifndef _EDGE_VERTEX_INDEX_H_
#define _EDGE_VERTEX_INDEX_H_

// todo comments for class

#include "Math/Vector3.h"
#include "Platform/DataTypes.h"

namespace SurfaceReconstruction
{
	/// Represents a directed Dual Marching Cell edge (between two Octree leaf cell centers) used to uniquely identify DMC vertices of an extracted triangle mesh.
	/** todo
		The edges are directed in order to support up to two vertices per undirected edge which may occur due to thin structures. */
	class EdgeVertexIndex
	{
	public:
		/** Creates an edge starting at the lower leaf index and ending at the greater leaf index.
			This is used for surface extraction without thin structures and up to 1 vertex per edge.
		@param todo */
		EdgeVertexIndex(const uint32 leafIdx0, const uint32 leafIdx1);
		
		bool operator ==(const EdgeVertexIndex &other) const;
		bool operator <(const EdgeVertexIndex &other) const;

		inline const uint32 *getLeafIndices() const;

	private:
		uint32 mLeafIndices[2];
	};
	
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///   inline function definitions   ////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	inline const uint32 *EdgeVertexIndex::getLeafIndices() const
	{
		return mLeafIndices;
	}
}

#endif // _EDGE_VERTEX_INDEX_H_