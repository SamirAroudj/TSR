/*
 * Copyright (C) 2018 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */

#ifndef _DUAL_MARCHING_CELL_H_
#define _DUAL_MARCHING_CELL_H_

#include <map>
#include <vector>
#include "Math/Vector3.h"
#include "Platform/DataTypes.h"
#include "SurfaceReconstruction/Scene/Tree/Nodes.h"

// todo comments for class

namespace SurfaceReconstruction
{
	// forward declarations
	class Occupancy;

	/// Contains data for a single cell stored for dual contouring.
	class DualMarchingCell
	{
	public:
		DualMarchingCell();

		/** todo */
		void clear();

		uint32 getCuttingsIdx() const;

		/** todo 
		@see Real getInterpolationFactor(const Math::Vector3 &p0, const Math::Vector3 &p1, const Real distance0, const Real distance1) const; */
		Real getInterpolationFactor(const uint32 edgeIdx) const;

		/** Computes the isosurface intersection point between p0 and p1 at their connecting line according to their attached distances distance0 and distance1.
			The function returns a corresponding interpolation factor that defines how much data of sample 0 at p0 and sample 1 at p1 should be used for interpolation of the data at the iso intersection point p_interpolated.
		@return Returns a factor f that should be used for interpolation of sample data at the two point p0 and p1 to get a properly interpolated representation at the corresponding intersection point p_interpolated.
			In math terms: interpolatedData = dataAtP0 + (dataAtP1 - dataAtP0) * getInterpolationFactor(p0, p1, distance0, distance1); */
		Real getInterpolationFactor(const Real distance0, const Real distance1) const;

		/** todo
			A triangle is degenerated as soon as one of its edges is collapsed (= a single point = its start and end leaf indices are equal). */
		bool isDegeneratedTriangle(const int32 edges[3]) const;

	public:
		static const uint32 CELL_EDGE_COUNT = 12;
		static const uint32 CELL_EDGE_END_COUNT = 2;
		static const uint32 CUTTINGS_COUNT = 256;

		static const uint32 MC_CORNER_REORDERING[Nodes::CHILD_COUNT]; /// Corners are enumerated in an order that fits to the tables MC_EDGE_CUTTINGS_TABLE and MC_TRIANGULATION_TABLE. (paulbourke order to internal order of Tree implementation)
		static const uint32	MC_EDGE_CUTTINGS_TABLE[CUTTINGS_COUNT];
		static const uint32 MC_EDGE_ENDS[DualMarchingCell::CELL_EDGE_COUNT][CELL_EDGE_END_COUNT];
		static const int32	MC_TRIANGULATION_TABLE[CUTTINGS_COUNT][16];

	public:
		// data for a single surface within this cell
		Math::Vector3 mCornerPositions[Nodes::CHILD_COUNT];
		Real mCornerDistances[Nodes::CHILD_COUNT];
		Real mCornerScales[Nodes::CHILD_COUNT];
		uint32 mCornerLeafIndices[Nodes::CHILD_COUNT];

		// data of ISO-vertices defining a single surface within this cell
		Math::Vector3 mInterPositions[CELL_EDGE_COUNT];
		Real mInterScales[CELL_EDGE_COUNT];
	};
}

#endif // _DUAL_MARCHING_CELL_H_
