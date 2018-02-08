/*
 * Copyright (C) 2018 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */
#ifndef _SURFEL_H_
#define _SURFEL_H_

#include <vector>
#include "Math/Vector3.h"

namespace SurfaceReconstruction
{
	/** Represents a point which was reprojected from a pixel a view onto some mesh. */
	struct Surfel
	{
	public:
		Surfel &operator =(const Surfel &rhs);

	public:
		Math::Vector3 mNormal;		/// normal of triangle mTriangleIdx
		Math::Vector3 mPosition;	/// 3D world space position
		Real mBaryCoords[2];		/// 3D point mPosition's barycentric coordinates w.r.t. triangle mTriangleIdx
		uint32 mTriangleIdx;		/// Identifies the mesh triangle onto which this point was reprojected.
	};

	typedef std::vector<Surfel> GeometryMap;
}

#endif // _SURFEL_H_