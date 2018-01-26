/*
* Copyright (C) 2017 by Author: Aroudj, Samir
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

using namespace FailureHandling;
using namespace Math;
using namespace std;
using namespace Storage;
using namespace ResourceManagement;
using namespace SurfaceReconstruction;
using namespace Utilities;

// constants for MVE
const char *Image::MVEI_FILE_SIGNATURE = "\211MVE_IMAGE\n";
const uint32 Image::MVEI_FILE_SIGNATURE_LENGTH = 11;

// constants for resource management
template <>
const char *Image::Resource<Image>::msResourcePath = NULL;

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
	assert(msResourcePath);
	if (!msResourcePath)
		throw FileAccessException("Cannot get color image since resource path is not set.", resourceName, -1);

	// is the image available in main memory?
	return VolatileResource<Image>::request(resourceName);
}

void Image::saveAsMVEFloatImage(const Path &fileName, const bool relativePath,
	const Utilities::ImgSize &size, const Real *realData,
	const bool invertX, const bool invertY, float *temporaryStorage)
{
	const MVEType type = MVE_FLOAT;
	const uint32 channelCount = 1;
	const uint32 eleCount = size.getElementCount();
	const uint32 eleSize = sizeof(float);

	// is the data already in the right format?
	if (sizeof(Real) == eleSize && !invertY)
	{
		Image::saveAsMVEI(fileName, relativePath, size, channelCount, type, realData, eleSize);
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
			floatData[targetIdx] = (float)realData[sourceIdx];
		}
	}

	// save it
	Image::saveAsMVEI(fileName, relativePath, size, channelCount, type, floatData, eleSize);

	// free resources
	if (ownAllocation)
		delete[] floatData;
	floatData = NULL;
}


void Image::loadMVEI(void *&data, ImgSize &size, uint32 &channelCount, uint32 &type,
	const Path &fileName, const bool relativePath)
{
	const Path targetFileName = (relativePath ? Path::appendChild(msResourcePath, fileName) : fileName);
	File file(targetFileName, File::OPEN_READING, true);

	load and save header via struct;
	// load & check signature
	char buffer[MVEI_FILE_SIGNATURE_LENGTH];
	file.read(buffer, MVEI_FILE_SIGNATURE_LENGTH, sizeof(char), MVEI_FILE_SIGNATURE_LENGTH);
	if (strncmp(buffer, MVEI_FILE_SIGNATURE, MVEI_FILE_SIGNATURE_LENGTH))
		throw FileCorruptionException("Could not load MVEI file! The MVE header is broken!", targetFileName);
}

void Image::saveAsMVEI(const Path &fileName, const bool relativePath,
	const ImgSize &size, const uint32 channelCount, const uint32 type, const void *data, const uint32 elementSize)
{
	if (!data)
		return;

	// create file
	const Path targetFileName = (relativePath ? Path::appendChild(msResourcePath, fileName) : fileName);
	File file(targetFileName, File::CREATE_WRITING, true);

	// write MVE image header
	file.write(MVEI_FILE_SIGNATURE, sizeof(char), MVEI_FILE_SIGNATURE_LENGTH);
	file.write(&size[0], sizeof(uint32), 1);
	file.write(&size[1], sizeof(uint32), 1);
	file.write(&channelCount, sizeof(uint32), 1);
	file.write(&type, sizeof(uint32), 1);

	// write MVE image body
	file.write(data, elementSize, size.getElementCount());
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
