/*
* Copyright (C) 2018 by Author: Aroudj, Samir
* TU Darmstadt - Graphics, Capture and Massively Parallel Computing
* All rights reserved.
*
* This software may be modified and distributed under the terms
* of the BSD 3-Clause license. See the License.txt file for details.
*/

#include "Platform/FailureHandling/FileAccessException.h"
#include "Platform/FailureHandling/FileCorruptionException.h"
#include "Platform/Storage/File.h"
#include "SurfaceReconstruction/Image/Image.h"
#include "SurfaceReconstruction/Image/MVEIHeader.h"

using namespace FailureHandling;
using namespace Math;
using namespace std;
using namespace Storage;
using namespace ResourceManagement;
using namespace SurfaceReconstruction;
using namespace Utilities;

// constants for resource management
template <>
Path Image::Resource<Image>::msResourcePath;

template <>
vector<Image *> Image::Resource<Image>::msResources(0);

template <>
uint32 VolatileResource<Image>::msMaximumNumber = 0x1 << 10;

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

void Image::freeMemory()
{
	VolatileResource<Image>::freeMemory();
}

Image *Image::request(const string &resourceName)
{
	// resource path and memory for loading available?
	assert(!msResourcePath.getString().empty());
	if (msResourcePath.getString().empty())
		throw FileAccessException("Cannot get color image since resource path is not set.", resourceName, -1);

	// is the image available in main memory?
	return VolatileResource<Image>::request(resourceName);
}

void Image::saveAsMVEFloatImage(const Path &fileName, const bool relativePath,
	const Utilities::ImgSize &size, const Real *realData,
	const bool invertX, const bool invertY, float *temporaryStorage)
{
	const MVEIHeader header(size, 1, MVEIHeader::MVE_FLOAT);
	const uint32 eleCount = size.getElementCount();

	// is the data already in the right format?
	if (sizeof(Real) == header.getTypeSize() && !invertY && !invertX)
	{
		Image::saveAsMVEI(fileName, relativePath, header, realData);
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
	for (uint32 y = 0; y < size[1]; ++y)
	{
		for (uint32 x = 0; x < size[0]; ++x)
		{
			const uint32 targetX = (invertX ? size[0] - (x + 1) : x);
			const uint32 targetY = (invertY ? size[1] - (y + 1) : y);
			const uint32 targetIdx = size[0] * targetY + targetX;
			const uint32 sourceIdx = size[0] * y + x;
			floatData[targetIdx] = (float) realData[sourceIdx];
		}
	}

	// save it
	Image::saveAsMVEI(fileName, relativePath, header, floatData);

	// free resources
	if (ownAllocation)
		delete[] floatData;
	floatData = NULL;
}


 void *Image::loadMVEI(MVEIHeader &header, const Path &fileName, const bool relativePath)
{
	// load MVE image
	const Path targetFileName = (relativePath ? Path::appendChild(msResourcePath, fileName) : fileName);
	File file(targetFileName, File::OPEN_READING, true);

	// load header
	header.loadFromFile(file);

	// allocate memory for the actual image data
	const uint32 pixelCount = header.mSize.getElementCount();
	const uint64 contentSize = header.getBodySize();
	uint8 *data = new uint8[contentSize];
	
	// load content
	const uint32 readCount = file.read(data, contentSize, header.getPixelSize(), pixelCount);
	if (pixelCount != readCount)
		throw FileCorruptionException("Could not load all pixels of an MVE image!", file.getName());
	return data;
}

void Image::saveAsMVEI(const Path &fileName, const bool relativePath,
	const MVEIHeader &header, const void *data)
{
	if (!data)
		return;

	// create file
	const Path targetFileName = (relativePath ? Path::appendChild(msResourcePath, fileName) : fileName);
	File file(targetFileName, File::CREATE_WRITING, true);

	// save image header
	header.saveToFile(file);

	// save image body
	const uint32 pixelCount = header.mSize.getElementCount();
	const uint32 writtenCount = file.write(data, header.getPixelSize(), pixelCount);
	if (writtenCount != pixelCount)
		throw FileException("Could not write MVE image content to file.", targetFileName);
}

Image::Image(const ImgSize &size, const string &resourceName) :
	VolatileResource<Image>(resourceName), mSize(size)
{

}

Image::Image(const Image &copy) : VolatileResource<Image>(""), mSize(copy.mSize)
{
	assert(false);
}

Image::~Image()
{

}


Image &Image::operator =(const Image &rhs)
{
	assert(false);
	return *this;

	//if (this == &rhs)
	//	return *this;

	//this->mSize = rhs.mSize;

	//return *this;
}
