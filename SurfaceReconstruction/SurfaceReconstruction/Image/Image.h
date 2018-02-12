/*
* Copyright (C) 2018 by Author: Aroudj, Samir
* TU Darmstadt - Graphics, Capture and Massively Parallel Computing
* All rights reserved.
*
* This software may be modified and distributed under the terms
* of the BSD 3-Clause license. See the License.txt file for details.
*/

#ifndef _IMAGE_H_
#define _IMAGE_H_

#include "Math/Vector2.h"
#include "Platform/ResourceManagement/VolatileResource.h"
#include "Platform/Storage/Path.h"
#include "Platform/Utilities/Size2.h"

// todo comments

namespace SurfaceReconstruction
{
	class Filter;
	struct MVEIHeader;

	class Image : public ResourceManagement::VolatileResource<Image>
	{
	public:
		friend class ResourceManagement::VolatileResource<Image>;

	public:
		/** todo
		@param trianglePS Set this to the triangle coordinates which are relative to this image's pixels. (Must be relative to this image's pixel space (PS)).
		@return Returns true if trianglePS is completely within the image. (Each vertex is within the rectangle [0, width) x [0, height). */
		static bool contains(const Math::Vector2 trianglePS[3], const Utilities::ImgSize &size);

		static void freeMemory();
		
		static void *loadMVEI(MVEIHeader &header, const Storage::Path &fileName, const bool relativePath);

		static void saveAsMVEFloatImage(const Storage::Path &fileName, const bool relativePath,
			const Utilities::ImgSize &size, const Real *data,
			const bool invertX = false, const bool invertY = true, float *temporaryStorage = NULL);
		static void saveAsMVEI(const Storage::Path &fileName, const bool relativePath,
			const MVEIHeader &header, const void *data);

		static inline void setPathToImages(const Storage::Path &path);

	public:
		/** todo
		@param trianglePS Set this to the triangle coordinates which are relative to this image's pixels. (Must be relative to this image's pixel space (PS)).
		@return Returns true if trianglePS is completely within the image. (Each vertex is within the rectangle [0, width) x [0, height). */
		inline bool contains(const Math::Vector2 trianglePS[3]) const;

		inline const Utilities::ImgSize &getSize() const;

	protected:
		Image(const Utilities::ImgSize &size, const std::string &resourceName);
		virtual ~Image();

		virtual void clear() = 0;

		static Image *request(const std::string &resourceName);

	private:
		Image(const Image &copy);
		Image &operator =(const Image &rhs);


	protected:
		Utilities::ImgSize mSize;
	};

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///   inline function definitions   ////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	inline void Image::setPathToImages(const Storage::Path &path)
	{
		msResourcePath = path;
	}

	bool Image::contains(const Math::Vector2 trianglePS[3]) const
	{
		return Image::contains(trianglePS, mSize);
	}

	inline const Utilities::ImgSize &Image::getSize() const
	{
		return mSize;
	}
}
#endif // _COLOR_IMAGE_H_
