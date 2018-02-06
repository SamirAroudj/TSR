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

View::View(const CameraData &data) :
	View(data.mViewID,
		 data.mOrientation, Vector4(data.mPosition.x, data.mPosition.y, data.mPosition.z, 1.0f),
		 data.mFocalLength, data.mPrincipalPoint, data.mPixelAspectRatio)
{

}

View::View(const uint32 ID, const Vector3 AABB[2], const Real minSampleDistance, const Real maxSampleDistance,
	const Real meanFOV, const Real maxFOVDeviation, const Real pixelAspectRatio) :
		mCamera(
			Quaternion(0, 0, 0, 1), Vector4(0, 0, 0, 1), 
			RandomManager::getSingleton().getUniform(meanFOV - maxFOVDeviation, meanFOV + maxFOVDeviation),
			Vector2(0.5f, 0.5f), pixelAspectRatio),
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
	const Real focalLength, const Vector2 &principalPoint, const Real pixelAspectRatio) :
	mCamera(orientation, position, focalLength, principalPoint, pixelAspectRatio), mID(ID)
{

}

View::View(const uint32 ID, File &file, const Path &fileName) :
	mCamera(Quaternion(0, 0, 0, 1), Vector4(), 1.0f, Vector2(0.5f, 0.5f), 1.0f), mID(ID)
{
	loadFromFile(file, fileName);
}

void View::computeHWSToNNPS(Math::Matrix4x4 &WSToPS, const ImgSize &resolution, const bool considerPixelCenterOffset) const
{
	// compute viewport transformation: device coordinates to pixel coordinates
	WSToPS = mCamera.computeWorldSpaceToPixelSpaceMatrix(resolution, considerPixelCenterOffset);
}

const ColorImage *View::getColorImage(const uint32 viewID, const string &tag, const uint32 scale)
{
	// return the image taken by this this view
	const Path relativeFileName = Scene::getRelativeImageFileName(viewID, tag, scale, true);
	return ColorImage::request(relativeFileName.getString(), relativeFileName);
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
	Real temp[4];
	Quaternion ori;
	Vector3 pos;

	file.read(temp, sizeof(Real) * 4, sizeof(Real), 4);
	file.read(&ori, sizeof(Quaternion), sizeof(Quaternion), 1);
	file.read(&pos, sizeof(Vector3), sizeof(Vector3), 1);
	
	// get focalLenth & principal point
	const Vector2 principalPoint(temp[0], temp[1]);
	const Real focalLength = temp[2];
	const Real pixelAspectRatio = temp[3];

	// update camera
	// extrinsic camera parameters
	mCamera.setOrientation(ori);
	mCamera.setPosition(pos.x, pos.y, pos.z, 1.0f);

	// intrinsic camera parameters
	mCamera.setProjectionProperties(focalLength, pixelAspectRatio);
	mCamera.setPrincipalPoint(principalPoint);
	
//	cout << "Loaded a view with (focal length, principal point, aspectRatio, position, orientation) = (" <<
//		"(" << focalLength << "), " <<
//		"(" << principalPoint << "), " <<
//		"(" << aspectRatio << "), " <<
//		"(" << pos << "), " <<
//		"(" << ori << ")" <<
//		")" << endl;
}

void View::saveToFile(File &file) const
{
	// save camera data
	const Real temp[4] =
	{
		mCamera.getPrincipalPoint().x,
		mCamera.getPrincipalPoint().y,
		mCamera.getFocalLength(),
		mCamera.getAspectRatio()
	};

	const Vector4 &camWS = mCamera.getPosition();
	const Quaternion &ori = mCamera.getOrientation();
	const Vector3 &pos = Vector3(camWS.x, camWS.y, camWS.z);

	file.write(temp, sizeof(Real), 4);
	file.write(&ori, sizeof(Quaternion), 1);
	file.write(&pos, sizeof(Vector3), 1);
}
