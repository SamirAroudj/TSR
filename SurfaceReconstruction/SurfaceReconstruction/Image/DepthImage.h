/*
* Copyright (C) 2017 by Author: Aroudj, Samir
* TU Darmstadt - Graphics, Capture and Massively Parallel Computing
* All rights reserved.
*
* This software may be modified and distributed under the terms
* of the BSD 3-Clause license. See the License.txt file for details.
*/

#ifndef _DEPTH_IMAGE_H_
#define _DEPTH_IMAGE_H_

#include "Math/Vector2.h"
#include "SurfaceReconstruction/Image/Image.h"
#include "Utilities/Size2.h"

// todo comments

namespace SurfaceReconstruction
{
	class ColorImage;
	class Filter;
	class FlexibleMesh;

	class DepthImage : public Image
	{
	public:
		static bool isDepthDiscontinuity(const Real footprints[4], const Real blockDepths[4], const uint32 i1, const uint32 i2);
		static DepthImage *request(const std::string &resourceName, const Storage::Path &imageFileName);

	public:
		/** todo */
		FlexibleMesh *triangulate(std::vector<std::vector<uint32>> &vertexNeighbors, std::vector<uint32> &indices, std::vector<uint32> &pixelToVertexIndices,
			const std::vector<Math::Vector3> &positionsWSMap, const Math::Matrix3x3 &pixelToViewSpace,
			const ColorImage *image = NULL) const;	
		void saveAsMVEFloatImage(const Storage::Path &fileName, 
			const bool invertX = false, const bool invertY = true, float *temporaryStorage = NULL);

	protected:
		DepthImage(const std::string &resourceName, const Storage::Path &imageFileName);
		DepthImage(Real *depths, const Utilities::ImgSize &size, const std::string &resourceName);
		virtual ~DepthImage();

		virtual void clear();
		
		FlexibleMesh *createFlexibleMesh(const std::vector<std::vector<uint32>> &vertexNeighbors, const std::vector<uint32> &indices,
			const std::vector<uint32> &pixelToVertexIndices, const std::vector<Math::Vector3> &positionsWSMap, const uint32 vertexCount,
			const ColorImage *image = NULL) const;

		uint32 triangulateBlock(std::vector<uint32> &indices, std::vector<uint32> &pixelToVertexIndices, uint32 vertexCount,
			const uint32 x, const uint32 y, const Math::Matrix3x3 &pixelToViewSpace) const;	
	
	public:
		static const Real DEPTH_DIFFERENCE_FACTOR;

	private:
		Real *mDepths;
	};
	
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///   inline function definitions   ////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

}

#endif // _DEPTH_IMAGE_