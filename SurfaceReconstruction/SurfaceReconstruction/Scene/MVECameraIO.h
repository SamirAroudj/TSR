/*
* Copyright (C) 2017 by Author: Aroudj, Samir
* TU Darmstadt - Graphics, Capture and Massively Parallel Computing
* All rights reserved.
*
* This software may be modified and distributed under the terms
* of the BSD 3-Clause license. See the License.txt file for details.
*/
#ifndef _MVE_CAMERA_IO_H_
#define _MVE_CAMERA_IO_H_

#include <map>
#include <vector>
#include "Platform/Storage/File.h"
#include "SurfaceReconstruction/Scene/CameraData.h"

// todo comments

namespace SurfaceReconstruction
{
	class View;

	class MVECameraIO
	{
	public:
		MVECameraIO(const Storage::Path &path);

		/** todo */
		void loadFromCamerasFile(std::vector<View *> &views, std::map<uint32, uint32> &oldToNewViewIDs,
			const Math::Matrix3x3 &inverseInputRotation, const Math::Vector3 &inverseInputTranslation);

		/** todo */
		void loadFromMetaIniFiles(std::vector<View *> &views, std::map<uint32, uint32> &oldToNewViewIDs,
			const Math::Matrix3x3 &inverseInputRotation, const Math::Vector3 &inverseInputTranslation);

	private:
		void clear(std::vector<View*>& views, std::map<uint32, uint32>& oldToNewViewIDs);

		/** todo */
		void loadFromMetaIniFile(std::vector<View *> &views, std::map<uint32, uint32> &oldToNewViewIDs, Storage::File &file,
			const Math::Matrix3x3 &inverseInputRotation, const Math::Vector3 &inverseInputTranslation);

		void readCamerasFileHeader(Storage::File &file);
		void readDistortion(CameraData &data, Storage::File &file);
		void readExtrinsics(CameraData &data, Storage::File &file,
			const Math::Matrix3x3 &inverseInputRotation, const Math::Vector3 &inverseInputTranslation);
		void readFocalLength(CameraData &data, Storage::File &file);
		void readPixelAspectRatio(CameraData &data, Storage::File& file);
		void MVECameraIO::readPrinciplePoint(CameraData &data, Storage::File &file);
		void readViewData(std::map<uint32, uint32> &oldToNewViewIDs, CameraData &data, Storage::File &file,
			const uint32 newViewID);

	public:
		static const char *ASPECT_RATIO_FORMAT;
		static const char *CAMERA_COUNT_FORMAT;
		static const char *CAMERA_NAME_FORMAT_0;
		static const char *CAMERA_NAME_FORMAT_1;
		static const char *DISTORTION_FORMAT;
		static const char *FOCAL_LENGTH_FORMAT;
		static const char *HEADER_SIGNATURE;
		static const char *PRINCIPLE_POINT_FORMAT;
		static const char *ROTATION_FORMAT;
		static const char *TRANSLATION_FORMAT;
		static const char *VIEW_ID_FORMAT;

	private:
		const Storage::Path mPath;
		uint32 mCameraCount;
	};

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///   inline function definitions   ////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

}

#endif // _MVE_CAMERA_IO_H_
