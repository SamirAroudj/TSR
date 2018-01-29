/*
* Copyright (C) 2017 by Author: Aroudj, Samir
* TU Darmstadt - Graphics, Capture and Massively Parallel Computing
* All rights reserved.
*
* This software may be modified and distributed under the terms
* of the BSD 3-Clause license. See the License.txt file for details.
*/

#ifndef _MVEI_HEADER_H_
#define _MVEI_HEADER_H_

#include "Platform/Storage/File.h"
#include "Utilities/Size2.h"

namespace SurfaceReconstruction
{
	struct MVEIHeader
	{
	public:
		/** Defines all supported types for the content of MVE image pixels. */
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
		static uint32 getTypeSize(const MVEType type);

	public:
		MVEIHeader();

		/** Constructor for defining and later saving the values of an MVE image header to a file.
		@param size Defines the width and height of the image in pixels.
		@param channelCount Defines the number of channels the image has, e.g. gray image <-> 1, RGBA image <-> 4, etc.
		@param type Defines the type of each channel element and each pixel, e.g., MVE_FLOAT for depth images, MVE_UINT8 for color images. SEE MVEType. */
		MVEIHeader(const Utilities::ImgSize &size, const uint32 channelCount, const MVEType type);

		MVEIHeader(Storage::File &file);

		inline uint64 getBodySize() const;
		inline uint32 getPixelSize() const;
		inline uint32 getTypeSize() const;

		void loadFromFile(Storage::File &file);

		/** Returns true when the header was successfully written to file.
		@param file The header is written (appended) to the file represented by file.
		@return Returns false when the header could not be written entirely and returns true when everything went fine. */
		void saveToFile(Storage::File &file) const;

	public:
		static const char *SIGNATURE;
		static const uint32 PARAMETER_COUNT = 4;
		static const uint32 SIGNATURE_LENGTH = 11;

	public:
		char mSignature[SIGNATURE_LENGTH];
		Utilities::ImgSize mSize;
		uint32 mChannelCount;
		uint32 mType;
	};

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///   inline function definitions   ////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	inline uint64 MVEIHeader::getBodySize() const
	{
		return getPixelSize() * mSize.getElementCount();
	}

	inline uint32 MVEIHeader::getPixelSize() const
	{
		return getTypeSize() * mChannelCount;
	}

	inline uint32 MVEIHeader::getTypeSize() const
	{
		return MVEIHeader::getTypeSize((MVEType) mType);
	}
}

#endif // _MVEI_HEADER_H_