/*
 * Copyright (C) 2017 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */
#ifndef _VERTEX_H_
#define _VERTEX_H_

#include "Math/Vector3.h"

// todo comments
namespace SurfaceReconstruction
{
	/// Represents corners of surfaces / meshes.
	class Vertex
	{
	public:
		inline Vertex(const Math::Vector3 &color, const Math::Vector3 &normal, const Math::Vector3 &position, const Real scale);
		inline bool operator <(const Vertex &rhs) const;

	public:
		static const uint32 INVALID_IDX;

	public:
		Math::Vector3 mColor;
		Math::Vector3 mNormal;
		Math::Vector3 mPosition;
		Real mScale;
	};	

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///   inline function definitions   ////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	inline Vertex::Vertex(const Math::Vector3 &color, const Math::Vector3 &normal, const Math::Vector3 &position, const Real scale) :
		mColor(color), mNormal(normal), mPosition(position), mScale(scale)
	{

	}
	
	inline bool Vertex::operator <(const Vertex &rhs) const
	{
		int n = memcmp(this, &rhs, sizeof(Vertex));
		return (n < 0);
	}
}

#endif // _VERTEX_H_