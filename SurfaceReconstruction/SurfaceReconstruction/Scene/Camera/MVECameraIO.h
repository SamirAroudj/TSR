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
#include "SurfaceReconstruction/Scene/Camera/CameraData.h"

// todo comments

namespace SurfaceReconstruction
{
	class Cameras;

	class MVECameraIO
	{
	public:
		MVECameraIO(const Storage::Path &path);

		/** todo */
		void loadFromCamerasFile(Cameras &cameras, std::vector<uint32> &viewToCameraIndices,
			const Math::Matrix3x3 &inverseInputRotation, const Math::Vector3 &inverseInputTranslation);

		/** todo */
		void loadFromMetaIniFiles(Cameras &cameras, std::vector<uint32> &viewToCameraIndices,
			const Math::Matrix3x3 &inverseInputRotation, const Math::Vector3 &inverseInputTranslation);

		void saveCamerasToFile(const Cameras &cameras) const;

	private:
		void clear(Cameras &cameras, std::vector<uint32>& viewToCameraIndices);

		/** todo */
		void loadFromMetaIniFile(Cameras &cameras, std::vector<uint32> &viewToCameraIndices,
			const Storage::Path &metaFileName, const Math::Matrix3x3 &inverseInputRotation, const Math::Vector3 &inverseInputTranslation);

		void readCamerasFileHeader(Storage::File &file);
		void readDistortion(CameraData &data, Storage::File &file);
		void readExtrinsics(CameraData &data, Storage::File &file,
			const Math::Matrix3x3 &inverseInputRotation, const Math::Vector3 &inverseInputTranslation);
		Real readFocalLength(Storage::File &file);
		Real readImageAspectRatio(Storage::File &file);
		Real readPixelAspectRatio(Storage::File& file);
		void readPrincipalPoint(CameraData &data, Storage::File &file);
		void readViewData(CameraData &data, Storage::File &file,
			std::vector<uint32> &viewToCameraIndices, const uint32 cameraIdx);

		int64 saveCameraToCamerasFile(char *buffer, const int64 bufferSize, const Cameras &cameras, const uint32 cameraIdx) const;

	private:
		const Storage::Path mPath;
		uint32 mCameraCount;
	};

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///   inline function definitions   ////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

}

#endif // _MVE_CAMERA_IO_H_
