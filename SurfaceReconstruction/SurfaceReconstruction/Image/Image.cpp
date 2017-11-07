/*
 * Copyright (C) 2017 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */
#include "Graphics/ImageManager.h"
#include "Math/MathHelper.h"
#include "Platform/FailureHandling/FileAccessException.h"
#include "SurfaceReconstruction/Image/Filter.h"
#include "SurfaceReconstruction/Image/Image.h"

using namespace FailureHandling;
using namespace Graphics;
using namespace Math;
using namespace ResourceManagement;
using namespace std;
using namespace Platform;
using namespace Storage;
using namespace SurfaceReconstruction;
using namespace Utilities;

// constants
template <>
const char *Image::Resource<Image>::msResourcePath = NULL;

template <>
vector<Image *> Image::Resource<Image>::msResources(0);

template <>
uint32 VolatileResource<Image>::msMaximumNumber = 0x1 << 10;

void Image::freeMemory()
{
	delete [] msResourcePath;
	msResourcePath = NULL;

	VolatileResource<Image>::freeMemory();
}

void Image::saveAsMVEFloatImage(const Path &fileName, const Utilities::Size2<uint32> &resolution, const Real *realData,
	const bool invertX, const bool invertY, float *temporaryStorage)
{
	const MVEType type = MVE_FLOAT;
	const uint32 channelCount = 1;
	const uint32 eleCount = resolution.getElementCount();
	const uint32 eleSize = sizeof(float);

	// is the data already in the right format?
	if (sizeof(Real) == eleSize && !invertY)
	{
		Image::saveAsMVEI(fileName, resolution, channelCount, type, realData, eleSize, eleCount);
		return;
	}

	// use temporaryStorage or own array for conversion?
	float *floatData = temporaryStorage;
	bool ownAllocation = false;
	if (!floatData)
	{
		floatData = new float[eleCount];
		ownAllocation = true;
	}

	// convert the data
	for (uint32 y = 0; y < resolution[1]; ++y)
	{
		for (uint32 x = 0; x < resolution[0]; ++x)
		{
			const uint32 targetX = (invertX ? resolution[0] - (x + 1) : x);
			const uint32 targetY = (invertY ? resolution[1] - (y + 1) : y);
			const uint32 targetIdx = resolution[0] * targetY + targetX;
			const uint32 sourceIdx = resolution[0] * y + x;
			floatData[targetIdx] = (float) realData[sourceIdx];
		}
	}

	// save it
	Image::saveAsMVEI(fileName, resolution, channelCount, type, floatData, eleSize, eleCount);

	// free resources
	if (ownAllocation)
		delete [] floatData;
	floatData = NULL;
}

void Image::saveAsMVEI(const Path &fileName, const Size2<uint32> &resolution, const uint32 channelCount, const uint32 type,
	const void *data, const uint32 elementSize, const uint64 elementCount)
{
	if (!data)
		return;

	// create file
	File file(fileName, File::CREATE_WRITING, true);
	
	// write MVE image header
	const char *MVEI_FILE_SIGNATURE = "\211MVE_IMAGE\n";
	const uint32  MVEI_FILE_SIGNATURE_LEN = 11;

	file.write(MVEI_FILE_SIGNATURE, sizeof(char), MVEI_FILE_SIGNATURE_LEN);
	file.write(&resolution[0], sizeof(uint32), 1);
	file.write(&resolution[1], sizeof(uint32), 1);
	file.write(&channelCount, sizeof(uint32), 1);
	file.write(&type, sizeof(uint32), 1);

	// write MVE image body
	file.write(data, elementSize, elementCount);
}

//Image *Image::requestMeanImage(const Image &source, const Filter &gaussian)
//{
//	// is the image alread in main memory?
//	const string resourceName = source.getName() + "Mean";
//	Image *image = request(resourceName);
//	if (image)
//		return image;
//
//	// get image meta data
//	const uint32 channelCount = source.getChannelCount();
//	const uint32 width = source.getWidth();
//	const uint32 height = source.getHeight();
//	const Texture::Format format = source.getFormat();
//
//	// request memory and compute filtered image
//	uint8 *pixels = new uint8[width * height * channelCount];
//	source.filter(pixels, gaussian);
//
//	return new Image(pixels, width, height, format, resourceName);
//}

bool Image::contains(const Vector2 trianglePS[3]) const
{
	return Image::contains(trianglePS, mSize);
}

bool Image::contains(const Vector2 trianglePS[3], const ImgSize &size)
{
	// check each vertex against this image's pixel rectangle
	for (uint32 i = 0; i < 3; ++i)
	{
		const Vector2 &v = trianglePS[i];

		if (v.x < 0.0f || v.x >= size[0] ||
			v.y < 0.0f || v.y >= size[1])
				return false;
	}

	return true;
}

void Image::filter(uint8 *targetPixels, const Filter &filter) const
{
	uint8 *target = targetPixels;

	const uint32 channelCount = getChannelCount();
	for (uint32 y = 0; y < mSize[1]; ++y)
		for (uint32 x = 0; x < mSize[0]; ++x)
			for (uint32 channel = 0; channel < channelCount; ++channel, ++target)
				*target = filter.getConvolution(x, y, channel, *this);
}

void Image::get(Vector3 &c, const Vector2 &coords, const Texture::Wrapping wrapping) const
{
	// get coordinates of the 4 pixels around x & corresponding interpolation factors
	int32 pixels[4];
	Real factors[2];
	Texture::getSurroundingPixelCoords(pixels[0], pixels[1], pixels[2], pixels[3], factors[0], factors[1],
		coords, mSize, wrapping);

	// interpolate bilinearly for each channel
	Real values[3][4];	// 3 channels x 4 source values

	for (uint32 channel = 0; channel < 3; ++channel)
	{
		values[channel][0] = get(pixels[0], pixels[1], channel) / 255.0f; // left, bottom
		values[channel][1] = get(pixels[2], pixels[1], channel) / 255.0f; // right, bottom
		values[channel][2] = get(pixels[0], pixels[3], channel) / 255.0f; // left, top
		values[channel][3] = get(pixels[2], pixels[3], channel) / 255.0f; // right, top

		c[(Axis) channel] = Math::interpolateBilinearly<Real>(values[channel], factors);
	}
}

Image *Image::request(const string &resourceName, const Path &imageFileName)
{
	// is the image alread in main memory?
	Image *image = request(resourceName);
	if (image)
		return image;

	// get complete file name
	const char *path = VolatileResource<Image>::getPathToResources();
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
	image = new Image(pixels, size, format, resourceName);
	return image;
}

Image *Image::request(const string &resourceName)
{
	// resource path and memory for loading available?
	assert(msResourcePath);
	if (!msResourcePath)
		throw FileAccessException("Cannot load resource since resource path is not set.", resourceName, -1);

	// is the image available in main memory?
	Image *image = VolatileResource<Image>::request(resourceName);
	return image;
}

void Image::setPathToImages(const Path &path)
{
	// copy path to memory
	const uint32 length = (uint32) path.getString().length();
	char *pathMemory = new char[length + 1];
	memcpy(pathMemory, path.getCString(), length);
	pathMemory[length] = '\0';

	msResourcePath = pathMemory;
}

Image::Image(uint8 *pixels, const ImgSize &size, const Graphics::Texture::Format format, const string &resourceName) :
	VolatileResource<Image>(resourceName), mPixels(pixels), mSize(size), mFormat(format)
{

}

Image::~Image()
{
	clear();
}

void Image::clear()
{
	delete [] mPixels;
}
