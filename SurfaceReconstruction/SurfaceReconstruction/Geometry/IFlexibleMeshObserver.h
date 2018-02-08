/*
 * Copyright (C) 2018 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */

#ifndef _I_FLEXIBLE_MESH_OBSERVER_H_
#define _I_FLEXIBLE_MESH_OBSERVER_H_

namespace SurfaceReconstruction
{
	class IFlexibleMeshObserver
	{
	public:
		virtual void onEdgeMerging(const uint32 targetVertex, const uint32 edgeVertex0, const uint32 edgeVertex1) = 0;
		virtual void onEdgeSplitVertex(const uint32 newVertexIdx, const uint32 edgeVertex0, const uint32 edgeVertex1) = 0;

		virtual void onFilterData(
			const uint32 *vertexOffsets, const uint32 vertexOffsetCount,
			const uint32 *edgeOffsets, const uint32 edgeOffsetCount,
			const uint32 *triangleOffsets, const uint32 triangleOffsetCount) = 0;

		virtual void onNewElements(
			const uint32 firstNewVertex, const uint32 newVertexCount,
			const uint32 firstNewEdge, const uint32 newEdgeCount,
			const uint32 firstNewTriangle, const uint32 newTriangleCount) = 0;

		virtual void onReserveMemory(const uint32 vertexCapacity, const uint32 edgeCapacity, const uint32 indexCapacity) = 0;
	};
}

#endif // _I_FLEXIBLE_MESH_OBSERVER_H_