/*
* Copyright (C) 2017 by Author: Aroudj, Samir
* TU Darmstadt - Graphics, Capture and Massively Parallel Computing
* All rights reserved.
*
* This software may be modified and distributed under the terms
* of the BSD 3-Clause license. See the License.txt file for details.
*/
#include "Platform/Storage/File.h"
#include "SurfaceReconstruction/Image/Image.h"

using namespace Math;
using namespace std;
using namespace Storage;
using namespace SurfaceReconstruction;
using namespace Utilities;

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
			floatData[targetIdx] = (float)realData[sourceIdx];
		}
	}

	// save it
	Image::saveAsMVEI(fileName, resolution, channelCount, type, floatData, eleSize, eleCount);

	// free resources
	if (ownAllocation)
		delete[] floatData;
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

Image::Image(const ImgSize &size) : mSize(size)
{

}

Image::Image(const Image &copy) : mSize(copy.mSize)
{

}

Image::~Image()
{

}


Image &Image::operator =(const Image &rhs)
{
	if (this == &rhs)
		return *this;

	this->mSize = rhs.mSize;

	return *this;
}
