/*
* Copyright (C) 2018 by Author: Aroudj, Samir
* TU Darmstadt - Graphics, Capture and Massively Parallel Computing
* All rights reserved.
*
* This software may be modified and distributed under the terms
* of the BSD 3-Clause license. See the License.txt file for details.
*/

#include <cstring>
#include "Platform/FailureHandling/FileCorruptionException.h"
#include "SurfaceReconstruction/Image/MVEIHeader.h"

using namespace FailureHandling;
using namespace Storage;
using namespace SurfaceReconstruction;
using namespace Utilities;

// constants for MVE
const char *MVEIHeader::SIGNATURE = "\211MVE_IMAGE\n";

MVEIHeader::MVEIHeader() :
	mSize((uint32) -1, (uint32) -1), mChannelCount((uint32) -1), mType(MVE_UNKNOWN)
{ 

}

uint32 SurfaceReconstruction::MVEIHeader::getTypeSize(const MVEType type)
{
	switch (type)
	{
		case MVE_UNKNOWN:
		{
			throw Exception("Cannot return the size for an unknown type.");
			break;
		}

		case MVE_UINT8:		return sizeof(uint8);
		case MVE_UINT16:	return sizeof(uint16);
		case MVE_UINT32:	return sizeof(uint32);
		case MVE_UINT64:	return sizeof(uint64);

		case MVE_SINT8:		return sizeof(int8);
		case MVE_SINT16:	return sizeof(int16);
		case MVE_SINT32:	return sizeof(int32);
		case MVE_SINT64:	return sizeof(int64);

		case MVE_FLOAT:		return sizeof(float);
		case MVE_DOUBLE:	return sizeof(double);

		default:
		{
			throw Exception("Missing case for getTypeSize for and MVE type!");
			break;
		}
	}

	return (uint32) -1;
}

MVEIHeader::MVEIHeader(const ImgSize & size, const uint32 channelCount, const MVEType type) :
	mSize(size), mChannelCount(channelCount), mType(type)
{

}

SurfaceReconstruction::MVEIHeader::MVEIHeader(File & file)
{
	loadFromFile(file);
}

void MVEIHeader::loadFromFile(Storage::File &file)
{
	// load & check signature
	file.read(mSignature, SIGNATURE_LENGTH, sizeof(uint8), SIGNATURE_LENGTH);
	if (strncmp(mSignature, SIGNATURE, SIGNATURE_LENGTH))
		throw FileCorruptionException("Could not load MVEI file! The MVE header is broken!", file.getName());

	// load parameters
	const uint32 bufferSize = PARAMETER_COUNT * sizeof(uint32);
	const uint32 readCount = file.read((void *) &mSize[0], bufferSize, sizeof(uint32), PARAMETER_COUNT);
	if (readCount != PARAMETER_COUNT)
		throw FileCorruptionException("Could not load MVE image header parameters!. The MVE header is broken!", file.getName());

	//// load parameters
	//mSize[0] = file.readUInt32(ENCODING_BINARY_LITTLE_ENDIAN);
	//mSize[1] = file.readUInt32(ENCODING_BINARY_LITTLE_ENDIAN);
	//mChannelCount = file.readUInt32(ENCODING_BINARY_LITTLE_ENDIAN);
	//mType = (MVEType) file.readUInt32(ENCODING_BINARY_LITTLE_ENDIAN);
}

void SurfaceReconstruction::MVEIHeader::saveToFile(File & file) const
{
	// save header to file	
	uint32 count = file.write(SIGNATURE, sizeof(char), SIGNATURE_LENGTH);
	count += file.write(&mSize[0], sizeof(uint32), PARAMETER_COUNT);

	if (count != (SIGNATURE_LENGTH + PARAMETER_COUNT))
		throw FileCorruptionException("Could not save MVE image header to file!", file.getName());

	//// save header to file
	//file.write(SIGNATURE, sizeof(char), SIGNATURE_LENGTH);
	//file.write(&mSize[0], sizeof(uint32), 1);
	//file.write(&mSize[1], sizeof(uint32), 1);
	//file.write(&mChannelCount, sizeof(uint32), 1);
	//file.write(&mType, sizeof(uint32), 1);
}
