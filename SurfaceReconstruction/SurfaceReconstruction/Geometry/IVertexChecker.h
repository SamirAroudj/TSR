/*
 * Copyright (C) 2018 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */
#ifndef _I_VERTEX_CHECKER_H_
#define _I_VERTEX_CHECKER_H_

#include "Platform/DataTypes.h"

// todo comments

namespace SurfaceReconstruction
{
	class IVertexChecker
	{
	public:
		virtual bool isBad(const uint32 vertexIdx) const = 0;
	};
}

#endif // _I_VERTEX_CHECKER_H_