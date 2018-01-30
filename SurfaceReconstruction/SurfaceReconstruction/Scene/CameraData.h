/*
* Copyright (C) 2017 by Author: Aroudj, Samir
* TU Darmstadt - Graphics, Capture and Massively Parallel Computing
* All rights reserved.
*
* This software may be modified and distributed under the terms
* of the BSD 3-Clause license. See the License.txt file for details.
*/
#ifndef _CAMERA_DATA_H_
#define _CAMERA_DATA_H_

#include <string>
#include "Math/Quaternion.h"
#include "Math/Vector3.h"
#include "Math/Vector2.h"
#include "Platform/DataTypes.h"

namespace SurfaceReconstruction
{
	struct CameraData
	{
	public:
		CameraData();

	public:
		std::string mName;
		Math::Quaternion mOrientation;
		Math::Vector3 mPosition;
		Math::Vector2 mDistortion;
		Math::Vector2 mPrinciplePoint;
		Real mFocalLength;
		Real mPixelAspectRatio;
		uint32 mViewID;
	};
}

#endif // _CAMERA_DATA_H_