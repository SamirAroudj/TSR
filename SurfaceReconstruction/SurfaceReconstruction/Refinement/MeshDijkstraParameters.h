/*
 * Copyright (C) 2018 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */

#ifndef _MESH_DIJKSTRA_PARAMETERS_H_
#define _MESH_DIJKSTRA_PARAMETERS_H_

#include <algorithm>
#include <map>
#include <vector>
#include "SurfaceReconstruction/Refinement/RangedVertexIdx.h"

// todo comments

namespace SurfaceReconstruction
{
	class FlexibleMesh;

	class MeshDijkstraParameters
	{
	public:
		MeshDijkstraParameters();
		
		inline Real getAngularCostsFactor() const;
		inline Real getBandwidthFactor() const;
		inline Real getMaxAngleDifference() const;

	private:
		// search configuration data
		Real mAngularCostsFactor;			/// Defines angular costs increase strength.
		Real mBandwidthFactor;
		Real mMaxAngleDifference;
	};

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///   inline function definitions   ////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	inline Real MeshDijkstraParameters::getAngularCostsFactor() const
	{
		return mAngularCostsFactor;
	}

	inline Real MeshDijkstraParameters::getBandwidthFactor() const
	{
		return mBandwidthFactor;
	}

	inline Real MeshDijkstraParameters::getMaxAngleDifference() const
	{
		return mMaxAngleDifference;
	}
}

#endif // _MESH_DIJKSTRA_PARAMETERS_H_