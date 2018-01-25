/*
 * Copyright (C) 2017 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */
#include "Math/MathHelper.h"
#include "Math/Vector3.h"
#include "Graphics/Viewport.h"
#include "Graphics/Camera3D.h"
#include "Platform/FailureHandling/FileException.h"
#include "Platform/Platform.h"
#include "SurfaceReconstruction/Image/ColorImage.h"
#include "SurfaceReconstruction/Scene/CapturedScene.h"
#include "SurfaceReconstruction/Scene/View.h"
#include "Utilities/RandomManager.h"

using namespace FailureHandling;
using namespace Graphics;
using namespace Math;
using namespace Platform;
using namespace std;
using namespace Storage;
using namespace SurfaceReconstruction;
using namespace Utilities;

const uint32 View::INVALID_ID = (uint32) -1;

View::View(const uint32 ID, const Vector3 AABB[2], const Real minSampleDistance, const Real maxSampleDistance,
	const Real meanFOV, const Real maxFOVDeviation, const Real aspectRatio) :
		mCamera(
			Quaternion(0, 0, 0, 1), Vector4(0, 0, 0, 1), 
			RandomManager::getSingleton().getUniform(meanFOV - maxFOVDeviation, meanFOV + maxFOVDeviation),
			Vector2(0.5f, 0.5f), aspectRatio),
		mID(ID)
{
	RandomManager &manager = RandomManager::getSingleton();

	// place the camera with some randomndess
	const Vector3 AABBSize		= (AABB[1] - AABB[0]);
	const Vector3 maxRndDiff	= AABBSize * 0.1f;
	const Vector3 target		= (AABB[0] + AABB[1]) * 0.5f + manager.getUniform(-maxRndDiff, maxRndDiff);
	const Vector3 position		= manager.getUniform(AABB[0], AABB[1]);

	mCamera.lookAt(position, target, Vector3(0.0f, 1.0f, 0.0f));
}

View::View(const uint32 ID, const Quaternion &orientation, const Vector4 &position,
	const Real focalLength, const Vector2 &principlePoint, const Real aspectRatio) :
	mCamera(orientation, position, focalLength, principlePoint, aspectRatio), mID(ID)
{

}

View::View(const uint32 ID, File &file, const Path &fileName) :
	mCamera(Quaternion(0, 0, 0, 1), Vector4(), 1.0f, Vector2(0.5f, 0.5f), 1.0f), mID(ID)
{
	loadFromFile(file, fileName);
}

void View::computeHWSToNNPS(Math::Matrix4x4 &WSToPS, const bool considerPixelCenterOffset) const
{
	// valid image?
	const ColorImage *image = getColorImage();
	if (!image)
	{
		WSToPS.setToZero();
		return;
	}

	// compute viewport transformation: device coordinates to pixel coordinates
	const ImgSize &resolution = image->getSize();
	WSToPS = mCamera.computeWorldSpaceToPixelSpaceMatrix(resolution, considerPixelCenterOffset);
}

string View::getIDString(const uint32 viewID)
{
	// viewID to at least 4 digits string
	char buffer[File::READING_BUFFER_SIZE];
	snprintf(buffer, File::READING_BUFFER_SIZE, "%.4u", viewID);

	return string(buffer);
}

const ColorImage *View::getColorImage() const
{
	return View::getColorImage(mID);
}

const ColorImage *View::getColorImage(const uint32 viewID)
{
	// return the image taken by this this view
	const Scene &scene = Scene::getSingleton();
	const string resourceName = View::getIDString(viewID);
	const Path relativeFileName = scene.getRelativeImageFileName(resourceName);
	
	const ColorImage *image = ColorImage::request(resourceName, relativeFileName);
	return image;
}

Vector3 View::getRay(const uint32 x, const uint32 y, const Matrix3x3 &pixelToRay)
{
	const Vector3 ray = Vector3((Real) x, (Real) y, 1.0f) * pixelToRay;
	return ray;
}

const Vector3 View::getViewDirection() const
{
	const Matrix4x4 &V = mCamera.getViewMatrix();
	return -Vector3(V.m02, V.m12, V.m22);
}

void View::loadFromFile(File &file, const Path &fileName)
{
	// load camera data
	Real		temp[3];
	Quaternion	ori;
	Vector3		pos;

	file.read(temp, sizeof(Real) * 3, sizeof(Real), 3);
	file.read(&ori, sizeof(Quaternion), sizeof(Quaternion), 1);
	file.read(&pos, sizeof(Vector3), sizeof(Vector3), 1);
	
	// get focalLenth & principle point
	const Vector2 principlePoint(temp[0], temp[1]);
	const Real focalLength = temp[2];

	// compute aspect ratio
	const ColorImage *image = getColorImage();
	//if (!image) todo
	//{
	//	string message = "Could not load the image for a view to determine its aspect ratio.";
	//	message += " View: " + getIDString(mID);
	//	throw FileException(message, fileName);
	//} todo 
	const Real aspectRatio = (image ? (Real) image->getSize()[0] / (Real) image->getSize()[1] : 1.0f);

	// update camera
	// extrinsic camera parameters
	mCamera.setOrientation(ori);
	mCamera.setPosition(pos.x, pos.y, pos.z, 1.0f);

	// intrinsic camera parameters
	mCamera.setProjectionProperties(focalLength, aspectRatio);
	mCamera.setPrinciplePoint(principlePoint);
	
//	cout << "Loaded a view with (focal length, principle point, aspectRatio, position, orientation) = (" <<
//		"(" << focalLength << "), " <<
//		"(" << principlePoint << "), " <<
//		"(" << aspectRatio << "), " <<
//		"(" << pos << "), " <<
//		"(" << ori << ")" <<
//		")" << endl;
}

void View::saveToFile(File &file) const
{
	// save camera data
	const Real temp[3] =
	{
		mCamera.getPrinciplePoint().x,
		mCamera.getPrinciplePoint().y,
		mCamera.getFocalLength()
	};

	const Vector4		&camWS	= mCamera.getPosition();
	const Quaternion	&ori	= mCamera.getOrientation();
	const Vector3		&pos	= Vector3(camWS.x, camWS.y, camWS.z);

	file.write(temp, sizeof(Real), 3);
	file.write(&ori, sizeof(Quaternion), 1);
	file.write(&pos, sizeof(Vector3), 1);
}
