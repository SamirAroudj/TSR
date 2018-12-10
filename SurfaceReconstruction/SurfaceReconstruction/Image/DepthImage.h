/*
* Copyright (C) 2018 by Author: Aroudj, Samir
* TU Darmstadt - Graphics, Capture and Massively Parallel Computing
* All rights reserved.
*
* This software may be modified and distributed under the terms
* of the BSD 3-Clause license. See the License.txt file for details.
*/

#ifndef _DEPTH_IMAGE_H_
#define _DEPTH_IMAGE_H_

#include "Graphics/PinholeCamera.h"
#include "Math/Vector2.h"
#include "SurfaceReconstruction/Image/Image.h"
#include "Platform/Utilities/Size2.h"

// todo comments

namespace SurfaceReconstruction
{
	class ColorImage;
	class Filter;
	class FlexibleMesh;

	class DepthImage : public Image
	{
	public:
		enum DepthConvention
		{
			DEPTH_ALONG_RAY,
			DEPTH_ALONG_Z_AXIS,
			CONVENTION_COUNT
		};

	public:
		static void convertDepthsToColor(uint8 *pixels, const Real *depths,
			const uint32 elementCount, const uint32 channelCount, const Real minDepth, const Real maxDepth);
		static void findExtrema(Real &minimum, Real &maximum, const Real *depths, const uint32 elementCount);
		static bool isDepthDiscontinuity(const Real footprints[4], const Real blockDepths[4], const uint32 i1, const uint32 i2);
		static DepthImage *request(const std::string &resourceName, const Storage::Path &imageFileName);
		static DepthImage *createFromDisneyData(const std::string &resourceName, Utilities::ImgSize size, Real* depthValues);

	public:
		/** Standard image erosion applied to depth images.
			Removes estimated depths (invalidates them) at the border between pixels with valid and invalid values.
			(Pixels with valid values are shrinked from outside towards inside by a border of width pixels.)
		@param borderWidth Defines the number of outermost border rings of valid depth value pixels which are removed.
			E.g. width = 1 -> remove a one pixel thick border. */
		void erode(const uint32 &borderWidth);

		inline DepthConvention getDepthConvention() const;

		bool hasInvalidNeighbor(const uint32 &x, const uint32 &y) const;

		/** todo */
		FlexibleMesh *triangulate(std::vector<uint32> &tempPixelToVertexIndices, std::vector<std::vector<uint32>> &tempVertexNeighbors, std::vector<uint32> &tempIndices,
			const Graphics::PinholeCamera &camera, const ColorImage *image = NULL) const;	

		void saveAsMVEFloatImage(const Storage::Path &fileName, 
			const bool invertX = false, const bool invertY = true, float *temporaryStorage = NULL);
		
		void setDepthConvention(const Graphics::PinholeCamera &camera, const DepthConvention &targetConvention);

	protected:
		static Real *loadDepths(MVEIHeader &header, const Storage::Path &relativeImageFileName);

	protected:
		DepthImage(Real *&depths, const Utilities::ImgSize &size, const std::string &resourceName, const DepthConvention &depthConvention = DEPTH_ALONG_RAY);
		virtual ~DepthImage();

		virtual void clear();
		
		FlexibleMesh *createFlexibleMesh(const std::vector<uint32> &pixelToVertexIndices, const std::vector<std::vector<uint32>> &vertexNeighbors,
			const Math::Matrix3x3 &hPSToNNRayDirWS, const Math::Vector3 &centerOfProjection,
			const std::vector<uint32> &indices, const ColorImage *image = NULL) const;

		void setVertexColors(FlexibleMesh &mesh, const std::vector<uint32> &pixelToVertexIndices, const uint32 &vertexCount,
			const ColorImage *image) const;

		void setVertexPositions(FlexibleMesh &mesh, const std::vector<uint32> &pixelToVertexIndices,
			const Math::Matrix3x3 &hPSToNNRayDirWS, const Math::Vector3 &centerOfProjection) const;

		uint32 triangulateBlock(std::vector<uint32> &indices, std::vector<uint32> &pixelToVertexIndices, uint32 vertexCount,
			const uint32 x, const uint32 y, const Math::Matrix3x3 &hPSToNNRayDirVS) const;	
	
	public:
		static const Real DEPTH_DIFFERENCE_FACTOR;
		static const uint8 SPECIAL_PINK_COLOR[4]; /// Special color value which can be used to for example identify actually unset/invalid pixels.

	private:
		Real *mDepths;
		DepthConvention mDepthConvention;
	};
	
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///   inline function definitions   ////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	inline DepthImage::DepthConvention DepthImage::getDepthConvention() const
	{
		return mDepthConvention;
	}
}

#endif // _DEPTH_IMAGE_
