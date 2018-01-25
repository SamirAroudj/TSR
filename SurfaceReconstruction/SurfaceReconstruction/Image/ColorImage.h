/*
 * Copyright (C) 2017 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */

#ifndef _COLOR_IMAGE_H_
#define _COLOR_IMAGE_H_

#include "Graphics/Texture.h"
#include "Math/Vector2.h"
#include "Math/Vector3.h"
#include "Math/Vector4.h"
#include "Platform/ResourceManagement/VolatileResource.h"
#include "Platform/Storage/Path.h"
#include "Utilities/Size2.h"

// todo comments

namespace SurfaceReconstruction
{
	class Filter;

	class ColorImage : public ResourceManagement::VolatileResource<ColorImage>
	{
	public:
		friend class ResourceManagement::VolatileResource<ColorImage>;

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
		static void freeMemory();
		static ColorImage *request(const std::string &resourceName, const Storage::Path &imageFileName);
		static void setPathToImages(const Storage::Path &path);

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

		/** todo
		zero / black border padding*/
		void filter(uint8 *targetPixels, const Filter &filter) const;
		
		inline uint8 get(const uint32 x, const uint32 y, const uint32 channel) const;
		
		inline void get(Math::Vector3 &color, const uint32 x, const uint32 y) const;

		/** todo - For real value access to RGB color values.
		so far bilinear interpolation
		@param color Is filled with interpolated red, green and blue values. */
		inline void get(Math::Vector3 &color,
			const Math::Vector2 &x, const Graphics::Texture::Wrapping wrapping = Graphics::Texture::WRAPPING_CLAMP) const;

		inline void get(Math::Vector4 &color,
			const Math::Vector2 &x, const Graphics::Texture::Wrapping wrapping = Graphics::Texture::WRAPPING_CLAMP) const;

		void get(Real *color, const uint32 channelCount,
			const Math::Vector2 coords, const Graphics::Texture::Wrapping wrapping = Graphics::Texture::WRAPPING_CLAMP) const;

		inline uint32 getChannelCount() const;

		inline Graphics::Texture::Format getFormat() const;
		inline const uint8 *getPixels() const;
		inline const Utilities::ImgSize &getSize() const;

	protected:
		ColorImage(uint8 *pixels, const Utilities::ImgSize &size, const Graphics::Texture::Format format, const std::string &resourceName);
		virtual ~ColorImage();

		void clear();
		
		static ColorImage *request(const std::string &resourceName);

	private:
		uint8 *mPixels;

		Utilities::ImgSize mSize;
		Graphics::Texture::Format mFormat;
	};

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///   inline function definitions   ////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	bool ColorImage::contains(const Math::Vector2 trianglePS[3]) const
	{
		return ColorImage::contains(trianglePS, mSize);
	}

	inline uint8 ColorImage::get(const uint32 x, const uint32 y, const uint32 channel) const
	{
		const uint32 channelCount = getChannelCount();
		const uint32 rowInBytes = mSize[0] * channelCount;

		return mPixels[y * rowInBytes + x * channelCount + channel];
	}

	inline void ColorImage::get(Math::Vector3 &color, const uint32 x, const uint32 y) const
	{
		const uint32 channelCount = getChannelCount();
		const uint32 rowInBytes = mSize[0] * channelCount;
		const uint8 *c = mPixels + y * rowInBytes + x * channelCount;

		color.set(c[0] / 255.0f, c[1] / 255.0f, c[2] / 255.0f);
	}

	inline void ColorImage::get(Math::Vector3 &c, const Math::Vector2 &coords, const Graphics::Texture::Wrapping wrapping) const
	{
		get((Real *) &c, 3, coords, wrapping);
	}

	inline void ColorImage::get(Math::Vector4 &c, const Math::Vector2 &coords, const Graphics::Texture::Wrapping wrapping) const
	{
		get((Real *) &c, 4, coords, wrapping);
	}

	inline uint32 ColorImage::getChannelCount() const
	{
		switch (mFormat)
		{
		case Graphics::Texture::FORMAT_RGB:
			return 3;

		case Graphics::Texture::FORMAT_RGBA:
			return 4;

		default:
			assert(false);
			return 0;
		}

		return 0;
	}

	inline Graphics::Texture::Format ColorImage::getFormat() const
	{
		return mFormat;
	}

	inline const uint8 *ColorImage::getPixels() const
	{
		return mPixels;
	}
	
	inline const Utilities::ImgSize &ColorImage::getSize() const
	{
		return mSize;
	}
}
#endif // _COLOR_IMAGE_H_
