/*
 * Copyright (C) 2018 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */
#ifndef _SCENE_TREE_SPHERE_NODES_CHECKER_H_
#define _SCENE_TREE_SPHERE_NODES_CHECKER_H_

#include "Math/Vector3.h"
#include "SurfaceReconstruction/Scene/Tree/NodesChecker.h"

namespace SurfaceReconstruction
{
	/// Nodes checker to test tree nodes for overlap with a specific 3D sphere.
	class SphereNodesChecker : public NodesChecker
	{
	public:
		/** Defines the sphere which is then used for overlap tests against scene tree node AABBs.
		@param Defines the center of the sphere to test for overlap with nodes.
		@param Defines the radius of the sphere to test for overlap with nodes. */
		SphereNodesChecker(const Math::Vector3 &sphereCenter, const Real sphereRadius, const uint32 maxDepth);

		/** Frees resources. */
		virtual ~SphereNodesChecker();

		/** Returns the center of the sphere to test for overlap with nodes. 
		@return Returns the center of the sphere to test for overlap with nodes. */
		inline const Math::Vector3 &getCenter() const;
			
		/** Returns the radius of the sphere to test for overlap with nodes.
		@return Returns the radius of the sphere to test for overlap with nodes.*/
		inline Real getRadius() const;

		/** Tests whether a node overlaps with the spherical radius of this nodes checker.
		@param scope Identifies and describes the extent of the node which is tested for intersection within this spherical nodes checker.
		@param isLeaf This should be set to true if scope is the scope of a leaf.
		@return Returns true when the spherical support volume of this nodes checker overlaps with the AABB of scope otherwise false is returned. */
		virtual bool isIgnored(const Scope &scope, const bool isLeaf, const uint32 depth) const;

	protected:
		const Math::Vector3 &mCenter;	/// Defines the center of the sphere to test for overlap with nodes.
		const Real mRadius;				/// Defines the radius of the sphere to test for overlap with nodes.
	};
	
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///   inline function definitions   ////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	inline const Math::Vector3 &SphereNodesChecker::getCenter() const
	{
		return mCenter;
	}

	Real SphereNodesChecker::getRadius() const
	{
		return mRadius;
	}
}

#endif // _SCENE_TREE_SPHERE_NODES_CHECEKR_H_