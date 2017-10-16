/*
 * Copyright (C) 2017 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */
#ifndef _SCENE_TREE_NODE_SCOPE_H_
#define _SCENE_TREE_NODE_SCOPE_H_

// todo comments

#include "Math/Vector3.h"
#include "Platform/DataTypes.h"

namespace SurfaceReconstruction
{
	/// Represents the spatial position and extent of a Nodes w.r.t. the scene and memory space
	class Scope
	{
	public:
		friend class Nodes;

	public:
		Scope();
		inline Scope(const Math::Vector3 &positionWS, const Real size, const uint32 idx);

		inline Math::Vector3 getCenterPosition() const;
		inline const Math::Vector3 &getMinimumCoordinates() const;
		inline Math::Vector3 getMaximumCoordinates() const;
		inline uint32 getNodeIndex() const;
		inline Real getSize() const;

	private:
		Math::Vector3 mPositionWS;
		Real mSize;
		uint32 mIdx;
	};
	
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///   inline function definitions   ////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	inline Scope::Scope(const Math::Vector3 &positionWS, const Real size, const uint32 idx) :
		mPositionWS(positionWS), mSize(size), mIdx(idx)
	{

	}

	inline Math::Vector3 Scope::getCenterPosition() const
	{
		const Real halfSize = 0.5f * mSize;
		return Math::Vector3(mPositionWS.x + halfSize,
							 mPositionWS.y + halfSize,
							 mPositionWS.z + halfSize);
	}
	
	inline Math::Vector3 Scope::getMaximumCoordinates() const
	{
		return Math::Vector3(mPositionWS.x + mSize,
							 mPositionWS.y + mSize,
							 mPositionWS.z + mSize);
	}

	inline const Math::Vector3 &Scope::getMinimumCoordinates() const
	{
		return mPositionWS;
	}

	inline uint32 Scope::getNodeIndex() const
	{
		return mIdx;
	}

	inline Real Scope::getSize() const
	{
		return mSize;
	}
}

#endif // _SCENE_TREE_NODE_SCOPE_H_