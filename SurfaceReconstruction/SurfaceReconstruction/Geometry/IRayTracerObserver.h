/*
 * Copyright (C) 2017 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */
#ifndef _IRAY_TRACER_OBSERVER_H_
#define _IRAY_TRACER_OBSERVER_H_

#include "Math/Vector3.h"
#include "Platform/DataTypes.h"

// todo comments

namespace SurfaceReconstruction
{
	class IRayTracerObserver
	{
	public:
		inline virtual ~IRayTracerObserver();

		virtual void onPotentialRayHit(const Math::Vector3 &hitPosition, const uint32 primitiveIdx, const uint32 localRayIdx, const Real t) = 0;
	};

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///   inline function definitions   ////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	inline IRayTracerObserver::~IRayTracerObserver()
	{
	
	}
}

#endif // _IRAY_TRACER_OBSERVER_H_