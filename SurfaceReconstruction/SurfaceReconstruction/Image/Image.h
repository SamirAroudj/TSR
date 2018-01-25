/*
* Copyright (C) 2017 by Author: Aroudj, Samir
* TU Darmstadt - Graphics, Capture and Massively Parallel Computing
* All rights reserved.
*
* This software may be modified and distributed under the terms
* of the BSD 3-Clause license. See the License.txt file for details.
*/

#ifndef _IMAGE_H_
#define _IMAGE_H_

#include "Math/Vector2.h"
#include "Platform/Storage/Path.h"
#include "Utilities/Size2.h"

// todo comments

namespace SurfaceReconstruction
{
	class Filter;

	class Image
	{
	public:
		enum MVEType
		{
			MVE_UNKNOWN,

			MVE_UINT8,	/// uint8_t, unsigned char
			MVE_UINT16,	/// uint16_t
			MVE_UINT32,	/// uint32_t, unsigned int
			MVE_UINT64,	/// uint64_t

			MVE_SINT8,	/// int8_t, char, signed char
			MVE_SINT16,	/// int16_t
			MVE_SINT32,	/// int32_t, int
			MVE_SINT64,	/// int64_t

			MVE_FLOAT,	/// float
			MVE_DOUBLE	/// double
		};

	public:
		/** todo
		@param trianglePS Set this to the triangle coordinates which are relative to this image's pixels. (Must be relative to this image's pixel space (PS)).
		@return Returns true if trianglePS is completely within the image. (Each vertex is within the rectangle [0, width) x [0, height). */
		static bool contains(const Math::Vector2 trianglePS[3], const Utilities::ImgSize &size);

		static void saveAsMVEFloatImage(const Storage::Path &fileName, const Utilities::Size2<uint32> &resolution, const Real *data,
			const bool invertX = false, const bool invertY = true, float *temporaryStorage = NULL);
		static void saveAsMVEI(const Storage::Path &fileName, const Utilities::Size2<uint32> &resolution, const uint32 channelCount,
			const uint32 type, const void *data, const uint32 elementSize, const uint64 elementCount);

	public:
		/** todo
		@param trianglePS Set this to the triangle coordinates which are relative to this image's pixels. (Must be relative to this image's pixel space (PS)).
		@return Returns true if trianglePS is completely within the image. (Each vertex is within the rectangle [0, width) x [0, height). */
		inline bool contains(const Math::Vector2 trianglePS[3]) const;

		inline const Utilities::ImgSize &getSize() const;

	protected:
		Image(const Utilities::ImgSize &size);
		Image(const Image &copy);
		virtual ~Image();

		virtual void clear() = 0;

		Image &operator =(const Image &rhs);

	protected:
		Utilities::ImgSize mSize;
	};

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///   inline function definitions   ////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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
