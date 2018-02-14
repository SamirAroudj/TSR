/*
* Copyright (C) 2018 by Author: Aroudj, Samir
* TU Darmstadt - Graphics, Capture and Massively Parallel Computing
* All rights reserved.
*
* This software may be modified and distributed under the terms
* of the BSD 3-Clause license. See the License.txt file for details.
*/

#include "Platform/FailureHandling/FileCorruptionException.h"
#include "Platform/Utilities/HelperFunctions.h"
#include "SurfaceReconstruction/Geometry/FlexibleMesh.h"
#include "SurfaceReconstruction/Image/ColorImage.h"
#include "SurfaceReconstruction/Image/DepthImage.h"
#include "SurfaceReconstruction/Image/MVEIHeader.h"

using namespace FailureHandling;
using namespace Graphics;
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
	Image(ImgSize(0, 0), resourceName), mDepths(NULL), mDepthConvention(DEPTH_CONVENTION_ALONG_RAY)
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
		throw FileException("Unsupported depth type. Only floating point numbers (IEEE 754, single or double precision) are supported!", imageFileName);

	// simply keep the data or do we need to convert the depths?
	const uint32 pixelCount = header.mSize.getElementCount();
	mDepths = reinterpret_cast<Real *>(data);

	#ifdef DOUBLE_PRECISION
		if (MVEIHeader::MVE_FLOAT == header.mType)
			mDepths = Converter::toFirstFromSecond<double, float>(data, pixelCount);
	#else
		if (MVEIHeader::MVE_DOUBLE == header.mType)
			mDepths = Converter::toFirstFromSecond<float, double>(data, pixelCount);
	#endif // DOUBLE_PRECISION
}

void DepthImage::erode(const uint32 &borderWidth)
{
	// memory for border pixel identification
	const uint32 pixelCount = mSize.getElementCount();
	const uint32 byteCount = sizeof(bool) * pixelCount;
	bool *borderPixels = new bool[pixelCount];

	// invalidate all borderWidth pixel borders
	for (uint32 borderIdx = 0; borderIdx < borderWidth; ++borderIdx)
	{
		memset(borderPixels, false, byteCount);

		// find 1-pixel border
		for (uint32 y = 0, xOffset = 0; y < mSize[1]; ++y, xOffset += mSize[0])
		{
			for (uint32 x = 0; x < mSize[0]; ++x)
			{
				// border pixel?
				const uint32 pixelIdx = x + xOffset;
				const Real &depth = mDepths[pixelIdx];
				if (depth > 0.0f)
					borderPixels[pixelIdx] = hasInvalidNeighbor(x, y);			
			}
		}

		// invalidate border
		for (uint32 pixelIdx = 0; pixelIdx < pixelCount; ++pixelIdx)
			if (borderPixels[pixelIdx])
				mDepths[pixelIdx] = -REAL_MAX;
	}

	// free resources
	delete [] borderPixels;
	borderPixels = NULL;
}

bool DepthImage::hasInvalidNeighbor(const uint32 &x, const uint32 &y) const
{
	// go over the 8-neighborhood
	for (int32 dY = -1; dY <= 1; ++dY)
	{
		// neighbor y within image?
		const uint32 neighborY = y + dY;
		if (neighborY >= mSize[1])
			continue;

		for (int32 dX = -1; dX <= 1; ++dX)
		{
			// no neighbor?
			if (dX == 0 && dY == 0)
				continue;

			// neighbor x within image?
			const uint32 neighborX = x + dX;
			if (neighborX >= mSize[0])
				continue;

			// invalid neighbor
			const uint32 neighborIdx = neighborY * mSize[0] + neighborX;
			if (mDepths[neighborIdx] < 0.0f)
				return true;
		}
	}

	return false;
}

FlexibleMesh *DepthImage::triangulate( vector<uint32> &tempPixelToVertexIndices, vector<vector<uint32>> &tempVertexNeighbors, vector<uint32> &tempIndices,
	const PinholeCamera &camera, const ColorImage *image) const
{
	// get necessary camera data
	const Matrix3x3 invViewPort = camera.computeInverseViewportMatrix(mSize, true);
	const Matrix3x3 invProj = camera.computeInverseProjectionMatrix();
	const Matrix3x3 invRot = camera.computeInverseRotationMatrix();
	const Matrix3x3 hPSToNNRayDirVS = invViewPort * invProj;
	const Matrix3x3 hPSToNNRayDirWS = hPSToNNRayDirVS * invRot; //camera.computeHPSToNNRayDirWS(mSize, true);
	const Vector4 &camPosHWS = camera.getPosition();
	const Vector3 camPosWS(camPosHWS.x, camPosHWS.y, camPosHWS.z);

	// reserve memory & clear buffers
	const uint32 pixelCount = mSize.getElementCount();
	tempPixelToVertexIndices.resize(pixelCount);
	memset(tempPixelToVertexIndices.data(), -1, sizeof(uint32) * pixelCount);
	tempIndices.clear();

	// compute vertex & index count & index buffer
	uint32 vertexCount = 0;
	for (uint32 y = 0; y < mSize[1] - 1; ++y)
		for (uint32 x = 0; x < mSize[0] - 1; ++x)
			vertexCount = triangulateBlock(tempIndices, tempPixelToVertexIndices, vertexCount, x, y, hPSToNNRayDirVS);
	const uint32 indexCount = (uint32) tempIndices.size();

	// recompute vertex neighbors
	tempVertexNeighbors.resize(vertexCount);
	for (uint32 vertexIdx = 0; vertexIdx < vertexCount; ++vertexIdx)
		tempVertexNeighbors[vertexIdx].clear();
	FlexibleMesh::findVertexNeighbors(tempVertexNeighbors.data(), tempIndices.data(), indexCount);

	return createFlexibleMesh(tempPixelToVertexIndices, tempVertexNeighbors, hPSToNNRayDirWS, camPosWS, tempIndices, image);
}

uint32 DepthImage::triangulateBlock(vector<uint32> &indices, vector<uint32> &pixelToVertexIndices, uint32 vertexCount,
	const uint32 x, const uint32 y, const Matrix3x3 &hPSToNNRayDirVS) const
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
		const Vector3 vVS = Vector3((Real) neighborX, (Real) neighborY, 1.0f) * hPSToNNRayDirVS;

		footprints[localDepthIdx] = hPSToNNRayDirVS.m00 * depth / vVS.getLength();
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

FlexibleMesh *DepthImage::createFlexibleMesh(const vector<uint32> &pixelToVertexIndices, const vector<vector<uint32>> &vertexNeighbors,
	const Matrix3x3 &hPSToNNRayDirWS, const Vector3 &centerOfProjection,
	const vector<uint32> &indices, const ColorImage *image) const
{
	// index & vertex count
	const uint32 indexCount = (uint32) indices.size();
	const uint32 vertexCount = (uint32) vertexNeighbors.size();

	// create mesh & set indices
	FlexibleMesh *viewMesh = new FlexibleMesh(vertexCount, indexCount);
	viewMesh->setIndices(indices.data(), indexCount);

	// set vertices: position, color, normal & scale for each vertex
	setVertexPositions(*viewMesh, pixelToVertexIndices, hPSToNNRayDirWS, centerOfProjection);
	setVertexColors(*viewMesh, pixelToVertexIndices, vertexCount, image);
	viewMesh->computeNormalsWeightedByAngles();
	FlexibleMesh::computeVertexScales(viewMesh->getScales(), vertexNeighbors.data(), viewMesh->getPositions(), viewMesh->getVertexCount());
	
	return viewMesh;
}

void DepthImage::setVertexPositions(FlexibleMesh &viewMesh, const vector<uint32> &pixelToVertexIndices,
	const Matrix3x3 &hPSToNNRayDirWS, const Vector3 &centerOfProjection) const
{
	// vertex positions
	const uint32 pixelCount = mSize.getElementCount();
	for (uint32 pixelIdx = 0; pixelIdx < pixelCount; ++pixelIdx)
	{
		// triangulated back projected depth value?
		const uint32 vertexIdx = pixelToVertexIndices[pixelIdx];
		if (Vertex::INVALID_IDX == vertexIdx)
			continue;
		
		// compute direction of ray through the current pixel (relative to world space)
		uint32 coords[2] = { pixelIdx % mSize[0], pixelIdx / mSize[0] };
		const Vector3 posHPS((Real) coords[0], (Real) coords[1], 1.0f);
		Vector3 directionWS = posHPS * hPSToNNRayDirWS;
		directionWS.normalize();

		// compute world space position using ray direction, center of projection and depth
		const Real &depth = mDepths[pixelIdx];
		const Vector3 posWS = centerOfProjection + (directionWS * depth);
		viewMesh.setPosition(posWS, vertexIdx);
	}
}

void DepthImage::setVertexColors(FlexibleMesh &viewMesh,
	const vector<uint32> &pixelToVertexIndices, const uint32 &vertexCount, const ColorImage *image) const
{
	// vertex colors via default color?
	if (!image)
	{
		// no image? -> set vertex colors to some default value
		const Vector3 defaultColor(0.5f, 0.5f, 0.5f);
		for (uint32 vertexIdx = 0; vertexIdx < vertexCount; ++vertexIdx)
			viewMesh.setColor(defaultColor, vertexIdx);
		return;
	}

	// set vertex colors via image
	const uint32 pixelCount = mSize.getElementCount();
	for (uint32 pixelIdx = 0; pixelIdx < pixelCount; ++pixelIdx)
	{
		const uint32 vertexIdx = pixelToVertexIndices[pixelIdx];
		if (Vertex::INVALID_IDX == vertexIdx)
			continue;

		// copy color
		const uint32 &width = image->getSize()[0];
		Vector3 color;

		image->get(color, pixelIdx % width, pixelIdx / width);
		viewMesh.setColor(color, vertexIdx);
	}
}

void DepthImage::saveAsMVEFloatImage(const Path &fileName, const bool invertX, const bool invertY, float *temporaryStorage)
{
	Image::saveAsMVEFloatImage(fileName, true, mSize, mDepths, invertX, invertY, temporaryStorage);
}

void DepthImage::setDepthConvention(const PinholeCamera &camera, const DepthConvention &targetConvention)
{
	// already wanted convention?
	if (targetConvention == mDepthConvention)
		return;

	// homogenous pixel to non-normalized ray dir view space matrix
	const Matrix3x3 invViewPort = camera.computeInverseViewportMatrix(mSize, true);
	const Matrix3x3 invProjection = camera.computeInverseProjectionMatrix();
	const Matrix3x3 hPSToNNRayDirVS = invViewPort * invProjection;

	// convert all valid depths
	for (uint32 y = 0, rowOffset = 0; y < mSize[1]; ++y, rowOffset += mSize[0])
	{
		for (uint32 x = 0; x < mSize[0]; ++x)
		{
			// valid depth?
			Real &depth = mDepths[rowOffset + x];
			if (depth <= 0.0f)
				continue;

			// ray direction in view space
			const Vector3 hPS((Real) x, (Real) y, 1.0f);
			Vector3 rayVS = hPS * hPSToNNRayDirVS;
			rayVS.normalize();

			// convert from depth along view space z-axis to depth along view space ray
			const Real conversionFactor = (DEPTH_CONVENTION_ALONG_Z_AXIS == targetConvention ? fabsr(rayVS.z) : 1.0f / rayVS.z);
			depth = depth * conversionFactor;
		}
	}

	mDepthConvention = targetConvention;
}

DepthImage::DepthImage(Real *depths, const ImgSize &size, const string &resourceName) :
	Image(size, resourceName), mDepths(NULL), mDepthConvention(DEPTH_CONVENTION_ALONG_RAY)
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
