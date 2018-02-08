/*
 * Copyright (C) 2018 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */

#ifndef _DUAL_MARCHING_CELLS_H_
#define _DUAL_MARCHING_CELLS_H_

#include <map>
#include <vector>
#include "Graphics/Color.h"
#include "SurfaceReconstruction/Geometry/FlexibleMesh.h"
#include "SurfaceReconstruction/SurfaceExtraction/DualMarchingCell.h"
#include "SurfaceReconstruction/SurfaceExtraction/EdgeVertexIndex.h"

// todo comments for class

namespace SurfaceReconstruction
{
	class Occupancy;
	class Leaves;

	/** Base class for surface mesh extraction for a Dual Octree which stores implicit function values.
		This implementation is based on the descriptions of the following sources:
		https://www.volume-gfx.com/volume-rendering/dual-marching-cubes/
		http://paulbourke.net/geometry/polygonise/marchingsource.cpp */
	class DualMarchingCells
	{
	public:
		DualMarchingCells(const Occupancy &occupancy, const uint32 minTriangleIsleSize);
		~DualMarchingCells();
		
		inline const FlexibleMesh &getSurface() const;

	protected:
		/** todo 
		@return Returns the vertex index of the triangle corner. A vertex might be added if it is new otherwise an index to an already existing vertex is returned. */
		uint32 addTriangleCorner(const Math::Vector3 &position, const Real scale, const EdgeVertexIndex &edgeVertexIdx);

		void clear();
		void extractMesh(const Occupancy &occupancy);
		void extractSurfaces(const Occupancy &occupancy, const Leaves &leaves, const uint32 *corners, const uint32 cellCount);
		void fillCellCorners(const Occupancy &occupancy, const Leaves &leaves, const uint32 *dualCells);
		void findCellIntersections(const uint32 cutEdges);

	private:
		inline DualMarchingCells(const DualMarchingCells &other);
		inline DualMarchingCells &operator =(const DualMarchingCells &rhs);

	public:
		static const Graphics::Color SURFACE_COLOR;

	private:
		DualMarchingCell mCell;
		FlexibleMesh mSurface;
		std::map<EdgeVertexIndex, uint32> mEvittvi;	/// Maps Dual Marching Cell edge (E) vertex (v) indices (i) to (t) triangle (t) vertex (v) indices (i) and is used to avoid duplicate vertices.
	};

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///   inline function definitions   ////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	inline const FlexibleMesh &DualMarchingCells::getSurface() const
	{
		return mSurface;
	}

	inline DualMarchingCells::DualMarchingCells(const DualMarchingCells &other)
	{
		assert(false);
	}

	inline DualMarchingCells &DualMarchingCells::operator =(const DualMarchingCells &rhs)
	{
		assert(false);
		return *this;
	}
}

#endif // _DUAL_MARCHING_CELLS_H_
