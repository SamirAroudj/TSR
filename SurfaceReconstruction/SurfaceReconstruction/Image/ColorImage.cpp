/*
 * Copyright (C) 2018 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */
#include "Graphics/ImageManager.h"
#include "Math/MathHelper.h"
#include "Platform/FailureHandling/FileCorruptionException.h"
#include "Platform/Storage/Path.h"
#include "SurfaceReconstruction/Image/Filter.h"
#include "SurfaceReconstruction/Image/ColorImage.h"

using namespace FailureHandling;
using namespace Graphics;
using namespace Math;
using namespace ResourceManagement;
using namespace std;
using namespace Storage;
using namespace SurfaceReconstruction;
using namespace Utilities;

void ColorImage::filter(uint8 *targetPixels, const Filter &filter) const
{
	uint8 *target = targetPixels;

	const uint32 channelCount = getChannelCount();
	for (uint32 y = 0; y < mSize[1]; ++y)
		for (uint32 x = 0; x < mSize[0]; ++x)
			for (uint32 channel = 0; channel < channelCount; ++channel, ++target)
				*target = filter.getConvolution(x, y, channel, *this);
}

void ColorImage::get(Real *color, const uint32 channelCount, const Vector2 coords, const Texture::Wrapping wrapping) const
{
	// get coordinates of the 4 pixels around x & corresponding interpolation factors
	const uint32 PIXEL_COUNT = 4;
	const uint32 MAX_CHANNEL_COUNT = 4;
	int32 pixelBB[PIXEL_COUNT];					// minX, minY, maxX, maxY
	Real factors[2];							// relative interpolation factors \in [0, 1]^2
	Real values[MAX_CHANNEL_COUNT][PIXEL_COUNT];	// (up to) 4 channels for 4 source pixels

													// get pixel bounding box around coords and corresponding interpolation factors
	Texture::getSurroundingPixelCoords(pixelBB[0], pixelBB[1], pixelBB[2], pixelBB[3], factors[0], factors[1],
		coords, mSize, wrapping);

	// get values for each pixel and channel
	for (uint32 pixelIdx = 0; pixelIdx < PIXEL_COUNT; ++pixelIdx)
	{
		// coords order: left, bottom -> right, bottom -> left, top, -> right, top
		for (uint32 channelIdx = 0; channelIdx < channelCount; ++channelIdx)
		{
			const uint32 x = pixelBB[2 * (pixelIdx & 0x1)];
			const uint32 y = pixelBB[1 + (pixelIdx & 0x2)];
			values[channelIdx][pixelIdx] = get(x, y, channelIdx) / 255.0f;
		}
	}

	// interpolate values bilinearly per channel
	for (uint32 channelIdx = 0; channelIdx < channelCount; ++channelIdx)
		color[(Axis)channelIdx] = Math::interpolateBilinearly<Real>(values[channelIdx], factors);
}

ColorImage *ColorImage::request(const string &resourceName, const Path &imageFileName)
{
	// is the image alread in main memory?
	Image *image = Image::request(resourceName);
	if (image)
	{
		ColorImage *colorImage = dynamic_cast<ColorImage *>(image);
		if (colorImage)
			return colorImage;
		throw FileException("An image of another type than ColorImage but with the same resource name already exists!", imageFileName);
	}

	return new ColorImage(resourceName, imageFileName);
}

ColorImage::ColorImage(const string &resourceName, const Path &imageFileName) :
	Image(ImgSize(0, 0), 0, resourceName), mPixels(NULL), mFormat(Texture::FORMAT_NUM_OF)
{
	// get complete file name
	const Path &folder = VolatileResource<Image>::getPathToResources();
	const Path fileName = Path::appendChild(folder, imageFileName);
	
	// load data
	mPixels = ImageManager::getSingleton().loadPNG(mSize, mFormat, fileName, false);
	if (!mPixels)
		throw FileCorruptionException("Could not load pixel RGB values from PNG file!", imageFileName);
	mChannelCount = Texture::getChannelCount(mFormat);

	checkFormat();
}

ColorImage::ColorImage(uint8 *pixels, const ImgSize &size, const Graphics::Texture::Format format, const string &resourceName) :
	Image(size, Texture::getChannelCount(format), resourceName), mPixels(pixels), mFormat(format)
{
	checkFormat();
}

void ColorImage::checkFormat()
{
	// suported format?
	if (mFormat == Texture::FORMAT_RGB)
		return;

	// unsupported format -> throw exception
	delete [] mPixels;
	mPixels = NULL;
	throw Exception("Unsupported image format. Only RGB images are supported.");
}

ColorImage::~ColorImage()
{
	clear();
}

void ColorImage::clear()
{
	delete [] mPixels;
	mPixels = NULL;
}
