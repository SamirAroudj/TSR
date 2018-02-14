/*
* Copyright (C) 2018 by Author: Aroudj, Samir
* TU Darmstadt - Graphics, Capture and Massively Parallel Computing
* All rights reserved.
*
* This software may be modified and distributed under the terms
* of the BSD 3-Clause license. See the License.txt file for details.
*/
#include "SurfaceReconstruction/Scene/Camera/CameraData.h"

using namespace Graphics;
using namespace Math;
using namespace std;
using namespace SurfaceReconstruction;

CameraData::CameraData() :
	mOrientation(), mPosition(),
	mDistortion(),
	mPrincipalPoint(0.5f, 0.5f),
	mFocalLength(-REAL_MAX),
	mPixelAspectRatio(1.0f),
	mViewID(-1)
{

}

void CameraData::set(const uint32 &viewID, const PinholeCamera &camera)
{
	// extrinsics
	mOrientation = camera.getOrientation();
	mPosition.set(camera.getPosition().x, camera.getPosition().y, camera.getPosition().z);

	// intrinsics
	mDistortion = camera.getDistortion();
	mPrincipalPoint = camera.getPrincipalPoint();

	mFocalLength = camera.getFocalLength();
	mPixelAspectRatio = camera.getAspectRatio();

	// link to view
	mViewID = viewID;
}