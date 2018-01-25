/*
 * Copyright (C) 2017 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */
#include "Platform/FailureHandling/FileAccessException.h"
#include "Graphics/ImageManager.h"
#include "Math/MathHelper.h"
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

// constants
template <>
const char *ColorImage::Resource<ColorImage>::msResourcePath = NULL;

template <>
vector<ColorImage *> ColorImage::Resource<ColorImage>::msResources(0);

template <>
uint32 VolatileResource<ColorImage>::msMaximumNumber = 0x1 << 10;

void ColorImage::freeMemory()
{
	delete [] msResourcePath;
	msResourcePath = NULL;

	VolatileResource<ColorImage>::freeMemory();
}

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
	ColorImage *image = request(resourceName);
	if (image)
		return image;

	// get complete file name
	const char *path = VolatileResource<ColorImage>::getPathToResources();
	const Path folder(path);
	const Path fileName = Path::appendChild(folder, imageFileName);

	// load the image
	uint8 *pixels = NULL;
	ImgSize size;
	Texture::Format format = Texture::FORMAT_NUM_OF;
	
	// load data?
	pixels = ImageManager::getSingleton().loadPNG(size, format, fileName);
	if (!pixels)
		return NULL;

	// suported format?
	if (format != Texture::FORMAT_RGB)
	{
		delete [] pixels;
		pixels = NULL;
		throw Exception("Unsupported image format. Only RGB images are supported.");
	}

	// create the image & return it
	image = new ColorImage(pixels, size, format, resourceName);
	return image;
}

ColorImage *ColorImage::request(const string &resourceName)
{
	// resource path and memory for loading available?
	assert(msResourcePath);
	if (!msResourcePath)
		throw FileAccessException("Cannot load resource since resource path is not set.", resourceName, -1);

	// is the image available in main memory?
	ColorImage *image = VolatileResource<ColorImage>::request(resourceName);
	return image;
}

void ColorImage::setPathToImages(const Path &path)
{
	// copy path to memory
	const uint32 length = (uint32) path.getString().length();
	char *pathMemory = new char[length + 1];
	memcpy(pathMemory, path.getCString(), length);
	pathMemory[length] = '\0';

	msResourcePath = pathMemory;
}

ColorImage::ColorImage(uint8 *pixels, const ImgSize &size, const Graphics::Texture::Format format, const string &resourceName) :
	VolatileResource<ColorImage>(resourceName), Image(size), mPixels(pixels), mFormat(format)
{

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
