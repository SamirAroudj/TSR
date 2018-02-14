/*
* Copyright (C) 2018 by Author: Aroudj, Samir
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
#include "SurfaceReconstruction/Scene/View/CameraData.h"

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

		void saveCamerasToFile(const std::vector<View *> &views) const;

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
		void readPrincipalPoint(CameraData &data, Storage::File &file);
		void readViewData(std::map<uint32, uint32> &oldToNewViewIDs, CameraData &data, Storage::File &file,
			const uint32 newViewID);

		int64 saveViewToMVECamerasFile(char *buffer, const int64 bufferSize, const View &view) const;

	private:
		const Storage::Path mPath;
		uint32 mCameraCount;
	};

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///   inline function definitions   ////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

}

#endif // _MVE_CAMERA_IO_H_
