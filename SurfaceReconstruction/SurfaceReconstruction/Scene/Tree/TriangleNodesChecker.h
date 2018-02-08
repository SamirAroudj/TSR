/*
 * Copyright (C) 2018 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */
#ifndef _SCENE_TREE_TRIANGLE_NODES_CHECKER_H_
#define _SCENE_TREE_TRIANGLE_NODES_CHECKER_H_

#include "Math/Vector3.h"
#include "SurfaceReconstruction/Scene/Tree/NodesChecker.h"

namespace SurfaceReconstruction
{
	/// Nodes checker to test tree nodes for intersection with a specific triangle.
	class TriangleNodesChecker : public NodesChecker
	{
	public:
		/** Defines the triangle which is later used for intersection tests against scene tree node AABBs.
		@param v0 Set this to the coordinates of the first triangle vertex / corner.
		@param v0 Set this to the coordinates of the second triangle vertex / corner.
		@param v0 Set this to the coordinates of the third triangle vertex / corner. */
		TriangleNodesChecker(const Math::Vector3 &v0, const Math::Vector3 &v1, const Math::Vector3 &v2, const uint32 maxDepth);

		/** Frees resources. */
		virtual ~TriangleNodesChecker();
			
		/** Tests whether a node intersects with the triangle of this NodesChecker object.
		@param scope Identifies and describes the extent of the node which is tested for intersection within this triangle intersection-based nodes checker.
		@param isLeaf This should be set to true if scope is the scope of a leaf.
		@return Returns true if the triangle of this NodesChecker objcet intersects with the AABB of scope otherwise false is returned. */
		virtual bool isIgnored(const Scope &scope, const bool isLeaf, const uint32 depth) const;

	private:
		Math::Vector3 mV0;
		Math::Vector3 mV1;
		Math::Vector3 mV2;
	};
}

#endif // _SCENE_TREE_TRIANGLE_NODES_CHECEKR_H_