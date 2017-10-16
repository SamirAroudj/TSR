/*
 * Copyright (C) 2017 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */
#ifndef _SURFACE_EXTRACTION_EMPTY_LINE_CHECKER_H_
#define _SURFACE_EXTRACTION_EMPTY_LINE_CHECKER_H_

#include "Math/Vector3.h"
#include "SurfaceReconstruction/Scene/Tree/NodesChecker.h"

namespace SurfaceReconstruction
{
	/// Nodes checker to test tree nodes for being empty and overlaping with some line segment
	class LineChecker : public NodesChecker
	{
	public:
		LineChecker(const Math::Vector3 &segmentP0, const Math::Vector3 &segmentP1, const uint32 maxDepth,
			const uint32 *nodeStates = NULL, const uint32 requiredFlag = 0);

		/** Frees resources. */
		virtual ~LineChecker();
			
		/** Tests whether a node overlaps with the entered line segment and if the node has the state empty.
		@param scope Identifies and describes the extent of the node which is tested for intersection within this spherical nodes checker.
		@param isLeaf This should be set to true if scope is the scope of a leaf.
		@return Returns true if scope intersects with the entered line segment and if the node is considered to have the state empty.
			Otherwise false is returned. */
		virtual bool isIgnored(const Scope &scope, const bool isLeaf, const uint32 depth) const;

	private:
		const uint32 *mNodeStates;
		const Math::Vector3 mP0;
		const Math::Vector3 mP1;
		const uint32 mRequiredFlag;
	};
}

#endif // _SURFACE_EXTRACTION_EMPTY_LINE_CHECKER_H_