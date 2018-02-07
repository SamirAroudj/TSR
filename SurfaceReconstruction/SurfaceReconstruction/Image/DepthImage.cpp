/*
* Copyright (C) 2017 by Author: Aroudj, Samir
* TU Darmstadt - Graphics, Capture and Massively Parallel Computing
* All rights reserved.
*
* This software may be modified and distributed under the terms
* of the BSD 3-Clause license. See the License.txt file for details.
*/

#include "Platform/FailureHandling/FileCorruptionException.h"
#include "SurfaceReconstruction/Geometry/FlexibleMesh.h"
#include "SurfaceReconstruction/Image/ColorImage.h"
#include "SurfaceReconstruction/Image/DepthImage.h"
#include "SurfaceReconstruction/Image/MVEIHeader.h"
#include "Utilities/HelperFunctions.h"

using namespace FailureHandling;
using namespace Math;
using namespace Storage;
using namespace std;
using namespace SurfaceReconstruction;
using namespace Utilities;

const Real DepthImage::DEPTH_DIFFERENCE_FACTOR = 5.0f;
const uint8 DepthImage::SPECIAL_PINK_COLOR[4] = { 255, 51, 255, 0 };

void DepthImage::convertDepthsToColor(uint8 *pixels, const Real *depths, const uint32 elementCount, const uint32 channelCount, const Real minDepth, const Real maxDepth)
{
	// convert depths into color values
	const Real depthRange = maxDepth - minDepth;
	const Real colorScaleFactor = 255 / depthRange;

	for (int64 eleIdx = 0; eleIdx < elementCount; ++eleIdx)
	{
		uint8 *target = pixels + eleIdx * channelCount;

		// invalid value?
		for (uint32 channelIdx = 0; channelIdx < channelCount; ++channelIdx)
			target[channelIdx] = (3 == channelCount ? DepthImage::SPECIAL_PINK_COLOR[channelIdx] : 0);
		if (0 > depths[eleIdx])
			continue;

		// gray according to depth
		const uint8 value = static_cast<uint8>(colorScaleFactor * (depths[eleIdx] - minDepth));
		for (uint32 channelIdx = 0; channelIdx < channelCount; ++channelIdx)
			target[channelIdx] = value;
	}
}

void DepthImage::findExtrema(Real &minimum, Real &maximum, const Real *depths, const uint32 elementCount)
{
	minimum = REAL_MAX;
	maximum = -REAL_MAX;

	for (uint32 eleIdx = 0; eleIdx < elementCount; ++eleIdx)
	{
		// valid depth?
		const Real depth = depths[eleIdx];
		if (0 >= depth || REAL_MAX == depth)
			continue;

		// update min & max
		if (depth > maximum)
			maximum = depth;
		if (depth < minimum)
			minimum = depth;
	}
}

DepthImage *DepthImage::request(const string &resourceName, const Path &imageFileName)
{
	Image *image = Image::request(resourceName);
	if (image)
	{
		DepthImage *depthImage = dynamic_cast<DepthImage *>(image);
		if (depthImage)
			return depthImage;
		throw FileException("An image of another type than DepthImage but with the same resource name already exists!", imageFileName);
	}

	// not existing -> create it
	return new DepthImage(resourceName, imageFileName);
}

DepthImage::DepthImage(const string &resourceName, const Path &imageFileName) :
	Image(ImgSize(0, 0), resourceName), mDepths(NULL)
{
	// get complete file name
	const Path viewsFolder(VolatileResource<Image>::getPathToResources());
	const Path fileName = Path::appendChild(viewsFolder, imageFileName);

	// load image header and body
	MVEIHeader header;
	void *data = Image::loadMVEI(header, imageFileName, true);
	mSize = header.mSize;

	// check header
	if (header.mChannelCount != 1)
		throw FileException("Unsupported channel count for a depth map!", imageFileName);
	if (MVEIHeader::MVE_FLOAT != header.mType && MVEIHeader::MVE_DOUBLE != header.mType)
		throw FileException("Unsupported depth type. Only real numbers are supported!", imageFileName);

	// simply keep or convert the loaded data?
	if (sizeof(Real) == header.getTypeSize())
	{
		mDepths = (Real *) data;
		return;
	}

	// convert the data
	const uint32 pixelCount = header.mSize.getElementCount();
	if (MVEIHeader::MVE_FLOAT == header.mType)
		mDepths = Utilities::convert<Real, float>(data, pixelCount);
	else if (MVEIHeader::MVE_DOUBLE == header.mType)
		mDepths = Utilities::convert<Real, double>(data, pixelCount);
	else
		throw FileCorruptionException("MVE image for depth image creation! Only supported data types are float and double.", imageFileName);
}

void DepthImage::saveAsMVEFloatImage(const Path &fileName, const bool invertX, const bool invertY, float *temporaryStorage)
{
	Image::saveAsMVEFloatImage(fileName, true, mSize, mDepths, invertX, invertY, temporaryStorage);
}

FlexibleMesh *DepthImage::triangulate(vector<vector<uint32>> &tempVertexNeighbors, vector<uint32> &tempIndices, vector<uint32> &tempPixelToVertexIndices,
	const vector<Vector3> &positionsWSMap, const Matrix3x3 &pixelToViewSpace, const ColorImage *image) const
{
	// reserve memory & clear buffers
	const uint32 pixelCount = mSize.getElementCount();
	tempPixelToVertexIndices.resize(pixelCount);
	memset(tempPixelToVertexIndices.data(), -1, sizeof(uint32) * pixelCount);
	tempIndices.clear();

	// compute vertex & index count & index buffer
	uint32 vertexCount = 0;
	for (uint32 y = 0; y < mSize[1] - 1; ++y)
		for (uint32 x = 0; x < mSize[0] - 1; ++x)
			vertexCount = triangulateBlock(tempIndices, tempPixelToVertexIndices, vertexCount, x, y, pixelToViewSpace);
	const uint32 indexCount = (uint32) tempIndices.size();

	// recompute vertex neighbors
	tempVertexNeighbors.resize(vertexCount);
	for (uint32 vertexIdx = 0; vertexIdx < vertexCount; ++vertexIdx)
		tempVertexNeighbors[vertexIdx].clear();
	FlexibleMesh::findVertexNeighbors(tempVertexNeighbors.data(), tempIndices.data(), indexCount);

	return createFlexibleMesh(tempVertexNeighbors, tempIndices, tempPixelToVertexIndices, positionsWSMap, vertexCount, image);
}

uint32 DepthImage::triangulateBlock(vector<uint32> &indices, vector<uint32> &pixelToVertexIndices, uint32 vertexCount,
	const uint32 x, const uint32 y, const Matrix3x3 &pixelToViewSpace) const
{
	// indices of 4 different, reasonable triangles within 2x2 rectangle of vertices
	const uint32 blockTriangles[4][3] =
	{
		{ 0, 2, 1 },
		{ 0, 3, 1 },
		{ 0, 2, 3 },
		{ 1, 2, 3 }
	};

	//const uint32 blockCoords[4][2] = 
	//{
	//	{ x, y },
	//	{ x + 1, y},
	//	{ x, y + 1},
	//	{ x + 1, y + 1}
	//};
	
	const uint32 &width = mSize[0];
	const uint32 &height = mSize[1];
	const uint32 pixelIdx = width * y + x;
	const bool inImage[4] =
	{
		(x < width && y < height),
		(x + 1 < width && y < height),
		(x < width && y + 1 < height),
		(x + 1 < width && y + 1 < height)
	};

	// get depths
	const Real blockDepths[4] =
	{
		(inImage[0] ? mDepths[pixelIdx] : -REAL_MAX),
		(inImage[1] ? mDepths[pixelIdx + 1] : -REAL_MAX),
		(inImage[2] ? mDepths[pixelIdx + width] : -REAL_MAX),
		(inImage[3] ? mDepths[pixelIdx + width + 1] : -REAL_MAX)
	};

	// mask-based encoding of available depths
	uint32 mask = 0;
	uint32 availablePixels = 0;
	for (uint32 localDepthIdx = 0; localDepthIdx < 4; ++localDepthIdx)
	{
		if (0.0f >= blockDepths[localDepthIdx])
			continue;

		mask |= 1 << localDepthIdx;
		++availablePixels;
	}

	// 3 or more vertices / at least 1 triangle?
	if (availablePixels < 3)
		return vertexCount;

	// find proper triangles to be created within the block
	uint32 triangleIndices[2] = { Triangle::INVALID_IDX, Triangle::INVALID_IDX };
	switch (mask)
	{
		case 7:  triangleIndices[0] = 0; break;
		case 11: triangleIndices[0] = 1; break;
		case 13: triangleIndices[0] = 2; break;
		case 14: triangleIndices[0] = 3; break;
		case 15:
		{
			// triangle with smaller depth
			const Real diagonalLength0 = fabsr(blockDepths[0] - blockDepths[3]);
			const Real diagonalLength1 = fabsr(blockDepths[1] - blockDepths[2]);

			if (diagonalLength0 < diagonalLength1)
			{
				triangleIndices[0] = 1;
				triangleIndices[1] = 2;
			}
			else
			{
				triangleIndices[0] = 0;
				triangleIndices[1] = 3;
			}
			break;
		}

		default:
			return vertexCount;
	}

	// compute 3D (world space) pixel footprints
	Real footprints[4];
	for (uint32 localDepthIdx = 0; localDepthIdx < 4; ++localDepthIdx)
	{
		if (0.0f >= blockDepths[localDepthIdx])
			continue;

		const uint32 neighborX = x + (localDepthIdx % 2);
		const uint32 neighborY = y + (localDepthIdx / 2);
		const Real &depth = blockDepths[localDepthIdx];
		const Vector3 vVS = Vector3((Real) neighborX, (Real) neighborY, 1.0f) * pixelToViewSpace;

		footprints[localDepthIdx] = pixelToViewSpace.m00 * depth / vVS.getLength();
	}

	// try to avoid triangulating depth discontinuities
	for (uint32 triangleIdxIdx = 0; triangleIdxIdx < 2; ++triangleIdxIdx)
	{
		// valid triangle?
		uint32 &triangleIdx = triangleIndices[triangleIdxIdx];
		if (Triangle::isInvalidIndex(triangleIdx))
			continue;

		// invalidate triangle if it looks like going over a depth discontinuity
		const uint32 *triangle = blockTriangles[triangleIdx];
		for (uint32 edgeIdx = 0; edgeIdx < 3 && !Triangle::isInvalidIndex(triangleIdx); ++edgeIdx)
		{
			const uint32 edgeEnd0 = edgeIdx;
			const uint32 edgeEnd1 = (edgeIdx + 1) % 3;
			if (isDepthDiscontinuity(footprints, blockDepths, triangle[edgeEnd0], triangle[edgeEnd1]))
				triangleIdx = Triangle::INVALID_IDX;
		}
	}

	// finally, create the triangles!
	for (uint32 triangleIdxIdx = 0; triangleIdxIdx < 2; ++triangleIdxIdx)
	{
		// valid triangle?
		const uint32 triangleIdx = triangleIndices[triangleIdxIdx];
		if (Triangle::isInvalidIndex(triangleIdx))// || /*0 == triangleIdx ||*/ 1 == triangleIdx || 2 == triangleIdx || 3 == triangleIdx)
			continue;

		// add indices for the triangle
		const uint32 *triangle = blockTriangles[triangleIdx];
		for (uint32 cornerIdx = 0; cornerIdx < 3; ++cornerIdx)
		{
			const uint32 dX = triangle[cornerIdx] % 2;
			const uint32 dY = triangle[cornerIdx] / 2;
			const uint32 neighborIdx = pixelIdx + dY * width + dX;

			// get vertex index for position (only keep triangulated projected depth values)
			uint32 &vertexIdx = pixelToVertexIndices[neighborIdx];
			if (-1 == vertexIdx)
			{
				vertexIdx = vertexCount;
				++vertexCount;
			}

			indices.push_back(vertexIdx);
		}
	}

	return vertexCount;
}

bool DepthImage::isDepthDiscontinuity(const Real footprints[4], const Real blockDepths[4], const uint32 v0, const uint32 v1)
{
	// index of vertex closer (min) and further away from camera (max)
	const uint32 closerVertex = (blockDepths[v0] < blockDepths[v1] ? v0 : v1);
	const uint32 furtherVertex = (blockDepths[v0] < blockDepths[v1] ? v1 : v0);

	// allow larger depth difference for diagonal edge
	Real ddFactor = DEPTH_DIFFERENCE_FACTOR;
	if (3 == v0 + v1)
		ddFactor *= Math::SQRT_TWO;

	// discontinuity depending on whether the edge reaches far away from the camera (relative to the footprint size of the closer pixel / vertex)
	const Real depthDifference = blockDepths[furtherVertex] - blockDepths[closerVertex];
	const bool discontinuity = (depthDifference > footprints[closerVertex] * ddFactor);
	return discontinuity;
}

FlexibleMesh *DepthImage::createFlexibleMesh(const vector<vector<uint32>> &vertexNeighbors, const vector<uint32> &indices,
	const vector<uint32> &pixelToVertexIndices,	const vector<Vector3> &positionsWSMap, const uint32 vertexCount, const ColorImage *image) const
{
	// create triangulation & set indices
	const uint32 indexCount = (uint32) indices.size();
	FlexibleMesh *triangulation = new FlexibleMesh(vertexCount, indexCount);
	triangulation->setIndices(indices.data(), indexCount);

	// vertex positions
	const uint32 pixelCount = mSize.getElementCount();
	for (uint32 pixelIdx = 0; pixelIdx < pixelCount; ++pixelIdx)
	{
		// triangulated back projected depth value?
		const uint32 vertexIdx = pixelToVertexIndices[pixelIdx];
		if (Vertex::INVALID_IDX != vertexIdx)
			triangulation->setPosition(positionsWSMap[pixelIdx], vertexIdx);
	}

	// vertex normals & scales
	triangulation->computeNormalsWeightedByAngles();
	FlexibleMesh::computeVertexScales(triangulation->getScales(), vertexNeighbors.data(), triangulation->getPositions(), triangulation->getVertexCount());

	// vertex colors via default color?
	if (!image)
	{
		// no image? -> set vertex colors to some default value
		const Vector3 defaultColor(0.5f, 0.5f, 0.5f);
		for (uint32 vertexIdx = 0; vertexIdx < vertexCount; ++vertexIdx)
			triangulation->setColor(defaultColor, vertexIdx);
		return triangulation;
	}

	// set vertex colors via image
	for (uint32 pixelIdx = 0; pixelIdx < pixelCount; ++pixelIdx)
	{
		const uint32 vertexIdx = pixelToVertexIndices[pixelIdx];
		if (Vertex::INVALID_IDX == vertexIdx)
			continue;

		// copy color
		const uint32 &width = image->getSize()[0];
		Vector3 color;

		image->get(color, pixelIdx % width, pixelIdx / width);
		triangulation->setColor(color, vertexIdx);
	}
	
	return triangulation;
}

DepthImage::DepthImage(Real *depths, const ImgSize &size, const string &resourceName) :
	Image(size, resourceName), mDepths(NULL)
{
	// copy depths
	const uint32 elementCount = size.getElementCount();

	mDepths = new Real[elementCount];
	memcpy(mDepths, depths, sizeof(Real) * elementCount);
}

DepthImage::~DepthImage()
{
	clear();
}

void DepthImage::clear()
{
	delete [] mDepths;
	mDepths = NULL;
}
