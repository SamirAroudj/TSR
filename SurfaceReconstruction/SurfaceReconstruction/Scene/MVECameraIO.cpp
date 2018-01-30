/*
* Copyright (C) 2017 by Author: Aroudj, Samir
* TU Darmstadt - Graphics, Capture and Massively Parallel Computing
* All rights reserved.
*
* This software may be modified and distributed under the terms
* of the BSD 3-Clause license. See the License.txt File for details.
*/
#include "Math/MathHelper.h"
#include "Math/Matrix3x3.h"
#include "Math/Vector2.h"
#include "Platform/FailureHandling/FileCorruptionException.h"
#include "Platform/Storage/Directory.h"
#include "Platform/Storage/Path.h"
#include "SurfaceReconstruction/Scene/MVECameraIO.h"
#include "SurfaceReconstruction/Scene/Scene.h"
#include "SurfaceReconstruction/Scene/View.h"

using namespace FailureHandling;
using namespace Math;
using namespace SurfaceReconstruction;
using namespace std;
using namespace Storage;

const char *MVECameraIO::ASPECT_RATIO_FORMAT = "pixel_aspect_ratio = " REAL_IT "\n";
const char *MVECameraIO::CAMERA_COUNT_FORMAT = "camera_count = %d\n";
const char *MVECameraIO::CAMERA_NAME_FORMAT_0 = "name = ";
const char *MVECameraIO::CAMERA_NAME_FORMAT_1 = "\n";
const char *MVECameraIO::DISTORTION_FORMAT = "camera_distortion = " REAL_IT " " REAL_IT "\n";
const char *MVECameraIO::FOCAL_LENGTH_FORMAT = "focal_length = " REAL_IT "\n";
const char *MVECameraIO::HEADER_SIGNATURE = "MVE camera infos 1.0\n";
const char *MVECameraIO::PRINCIPLE_POINT_FORMAT = "principle_point = " REAL_IT " " REAL_IT "\n";
const char *MVECameraIO::ROTATION_FORMAT = "rotation = " REAL_IT " " REAL_IT " " REAL_IT " " REAL_IT " " REAL_IT " " REAL_IT " " REAL_IT " " REAL_IT " " REAL_IT "\n";
const char *MVECameraIO::TRANSLATION_FORMAT = "translation = " REAL_IT " " REAL_IT " " REAL_IT "\n";
const char *MVECameraIO::VIEW_ID_FORMAT = "id = %d\n";

MVECameraIO::MVECameraIO(const Path &path) :
	mPath(path)
{

}

void MVECameraIO::loadFromCamerasFile(vector<View *> &views, map<uint32, uint32> &oldToNewViewIDs,
	const Matrix3x3 &inverseInputRotation, const Vector3 &inverseInputTranslation)
{
	clear(views, oldToNewViewIDs);

	// read header & reserve memory
	File file(mPath, File::OPEN_READING, false);
	readCamerasFileHeader(file);
	views.reserve(mCameraCount);

	// read each camera
	uint32 newViewID = 0;
	while (file.hasLeftData())
	{
		CameraData cameraData;
		readViewData(oldToNewViewIDs, cameraData, file, newViewID);
		readFocalLength(cameraData, file);
		readPrinciplePoint(cameraData, file);
		readPixelAspectRatio(cameraData, file);
		readDistortion(cameraData, file);
		readExtrinsics(cameraData, file, inverseInputRotation, inverseInputTranslation);

		// check parameters
		if (EPSILON >= cameraData.mFocalLength)
			continue;

		// create view
		views.push_back(new View(cameraData));
		++newViewID;

		// debug output
		#ifdef _DEBUG
			cout << "\nRead & created view " << cameraData.mName << ", " << cameraData.mViewID;
		#endif // _DEBUG
	}

	#ifdef _DEBUG
		cout << endl;
	#endif // _DEBUG
}

void MVECameraIO::readCamerasFileHeader(File &file)
{
	string line;
	file.readTextLine(line);
	if (string::npos == line.find(HEADER_SIGNATURE))
		throw FileCorruptionException("First line of MVE cameras File is not in correct format.", file.getName());

	if (1 != file.scanf(CAMERA_COUNT_FORMAT, &mCameraCount))
		throw FileCorruptionException("Camera count isn't provided in correct MVE cameras File format.", file.getName());
}

void MVECameraIO::loadFromMetaIniFiles(vector<View *> &views, map<uint32, uint32> &oldToNewViewIDs,
	const Matrix3x3 &inverseInputRotation, const Vector3 &inverseInputTranslation)
{
	clear(views, oldToNewViewIDs);

	// assume MVE file structure -> one folder per view and each folder contains a meta.ini file describing the camera for that view
	// does the views folder exist?
	if (!Directory::exists(mPath))
		throw FileException("Cannot open views folder for loading cameras!", mPath);

	// find its children - each should contain a camera file called meta.ini
	vector<string> children;
	Directory::findChildren(children, mPath);
	
	// process each camera file
	const uint32 fileCount = 0;
	for (uint32 fileIdx = 0; fileIdx < fileCount; ++fileIdx)
	{
		// is it a directory?
		const string &fileName = children[fileIdx];
		if (!Directory::exists(fileName))
			continue;
	
		const Path metaFileName = Path::appendChild(fileName, "meta.ini");
		try
		{
			File cameraFile(metaFileName, File::OPEN_READING, false);
			loadFromMetaIniFile(views, oldToNewViewIDs, cameraFile, inverseInputRotation, inverseInputTranslation);
		}
		catch (Exception &exception)
		{
			// todo log this: "Could not open file: " + metaFileName
		}
	}
}

void MVECameraIO::loadFromMetaIniFile(vector<View *> &views, map<uint32, uint32> &oldToNewViewIDs, File &file,
	const Matrix3x3 &inverseInputRotation, const Vector3 &inverseInputTranslation)
{
	const uint32 newViewID = (uint32) views.size();
	CameraData cameraData;
	string line;
	
	// read data into cameraData
	while (file.hasLeftData())
	{
		file.readTextLine(line);

		// empty line?
		if (line.empty())
			continue;

		// comment?
		if ('#' == line[0])
			continue;

		// [camera] block start?
		if (string::npos != line.find("[camera]"))
		{
			readFocalLength(cameraData, file);
			readPixelAspectRatio(cameraData, file);
			readPrinciplePoint(cameraData, file);
			readExtrinsics(cameraData, file, inverseInputRotation, inverseInputTranslation);
			continue;
		}

		// [view] block start?
		if (string::npos != line.find("[view]"))
		{
			readViewData(oldToNewViewIDs, cameraData, file, newViewID);
			continue;
		}
	}

	// create view
	views.push_back(new View(cameraData));
}

void MVECameraIO::readDistortion(CameraData &data, File &file)
{
	if (2 != file.scanf(DISTORTION_FORMAT, &data.mDistortion.x, &data.mDistortion.y))
		throw FileCorruptionException("Camera distortion parameters aren't provided in correct MVE cameras File format.", file.getName());
}

void MVECameraIO::readExtrinsics(CameraData &data, File &file,
	const Matrix3x3 &inverseInputRotation, const Vector3 &inverseInputTranslation)
{
	Matrix3x3 rotation;
	Real *p = (Real *)rotation.values;
	Vector3 &pos = data.mPosition;

	// load translation vector (MVE conventions t = -R^t * cameraPos with R as row major matrix for rhs column vectors)
	if (3 != file.scanf(TRANSLATION_FORMAT, &pos.x, &pos.y, &pos.z))
		throw FileCorruptionException("Camera translation vector isn't provided in correct MVE cameras File format.", file.getName());

	// load rotation in matrix format
	if (9 != file.scanf(ROTATION_FORMAT, p, p + 1, p + 2, p + 3, p + 4, p + 5, p + 6, p + 7, p + 8, p + 9))
		throw FileCorruptionException("Camera rotation matrix isn't provided in correct MVE cameras File format.", file.getName());

	// adapt camera data to this project's conventions
	{
		// translation = -R^-1 * cam position -> position = -R * translation (loaded rotation is R as it is (R^-1)^t = R)
		const Vector3 temp = -pos * rotation;
		pos = temp;

		// here a camera looks along negative z-direction in order to have a local camera frame which is right handed and
		// the camera has x pointing right and y up
		rotation.m10 = -rotation.m10;
		rotation.m11 = -rotation.m11;
		rotation.m12 = -rotation.m12;
		rotation.m20 = -rotation.m20;
		rotation.m21 = -rotation.m21;
		rotation.m22 = -rotation.m22;
	}

	// transform camera into the coordinate system dataBasis
	data.mOrientation = Math::createQuaternionFromMatrix(rotation * inverseInputRotation);
	pos = pos * inverseInputRotation - inverseInputTranslation;
}

void MVECameraIO::readFocalLength(CameraData &data, File &file)
{
	if (1 != file.scanf(FOCAL_LENGTH_FORMAT, &data.mFocalLength))
		throw FileCorruptionException("Camera focal length isn't provided in correct MVE cameras File format.", file.getName());
}

void MVECameraIO::readPixelAspectRatio(CameraData &data, File &file)
{
	if (1 != file.scanf(ASPECT_RATIO_FORMAT, &data.mPixelAspectRatio))
		throw FileCorruptionException("Camera focal length isn't provided in correct MVE cameras File format.", file.getName());
}

void MVECameraIO::readPrinciplePoint(CameraData &data, File &file)
{
	if (2 != file.scanf(PRINCIPLE_POINT_FORMAT, &data.mPrinciplePoint.x, &data.mPrinciplePoint.y))
		throw FileCorruptionException("Principle point isn't provided in correct MVE cameras File format.", file.getName());
}

void MVECameraIO::readViewData(map<uint32, uint32> &oldToNewViewIDs, CameraData &data, File &file, const uint32 newViewID)
{
	char nameBuffer[File::READING_BUFFER_SIZE];
	uint32 oldViewID;

	// get old view ID (MVE view IDs are not necessarily compact)
	if (1 != file.scanf(VIEW_ID_FORMAT, &oldViewID))
		throw FileCorruptionException("View ID isn't provided in correct MVE cameras File format.", file.getName());

	// link old to new view ID & set new view ID
	if (oldToNewViewIDs.end() != oldToNewViewIDs.find(oldViewID))
		throw FileCorruptionException("Duplicate view ID. All view IDs in MVE camera files must be unique!", file.getName());
	oldToNewViewIDs[oldViewID] = newViewID;
	data.mViewID = newViewID;

	// set view name
	if (1 != file.scanfString(CAMERA_NAME_FORMAT_0, CAMERA_NAME_FORMAT_1, nameBuffer, File::READING_BUFFER_SIZE))
		throw FileCorruptionException("Camera name isn't provided in correct MVE cameras File format.", file.getName());
	data.mName = nameBuffer;
}

void MVECameraIO::clear(vector<View *> &views, map<uint32, uint32> &oldToNewViewIDs)
{
	assert(views.empty());
	if (!views.empty())
		throw Exception("Views loading must always start from scratch!");
	oldToNewViewIDs.clear();
}