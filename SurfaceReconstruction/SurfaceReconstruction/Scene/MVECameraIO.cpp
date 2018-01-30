/*
* Copyright (C) 2017 by Author: Aroudj, Samir
* TU Darmstadt - Graphics, Capture and Massively Parallel Computing
* All rights reserved.
*
* This software may be modified and distributed under the terms
* of the BSD 3-Clause license. See the License.txt File for details.
*/
#include <fstream>
#include "Graphics/PinholeCamera.h"
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
using namespace Graphics;
using namespace Math;
using namespace SurfaceReconstruction;
using namespace std;
using namespace Storage;

const char *MVECameraIO::CAMERA_COUNT_FORMAT = "camera_count = %d\n";
const char *MVECameraIO::CAMERA_NAME_FORMAT_0 = "name = ";
const char *MVECameraIO::CAMERA_NAME_FORMAT_1 = "\n";
const char *MVECameraIO::DISTORTION_FORMAT = "camera_distortion = " REAL_IT " " REAL_IT "\n";
const char *MVECameraIO::FOCAL_LENGTH_FORMAT = "focal_length = " REAL_IT "\n";
const char *MVECameraIO::HEADER_SIGNATURE = "MVE camera infos 1.1\n";
const char *MVECameraIO::PIXEL_ASPECT_RATIO_FORMAT = "pixel_aspect = " REAL_IT "\n";
const char *MVECameraIO::PRINCIPAL_POINT_FORMAT = "principal_point = " REAL_IT " " REAL_IT "\n";
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
	CameraData cameraData;
	string line;

	while (file.hasLeftData())
	{
		readFocalLength(cameraData, file);
		readPixelAspectRatio(cameraData, file);
		readPrincipalPoint(cameraData, file);
		readDistortion(cameraData, file);
		readExtrinsics(cameraData, file, inverseInputRotation, inverseInputTranslation);
		readViewData(oldToNewViewIDs, cameraData, file, newViewID);

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

	// signature with version
	file.readTextLine(line);
	if (string::npos == line.find(HEADER_SIGNATURE))
		throw FileCorruptionException("First line of MVE cameras File is not in correct format.", file.getName());

	// camera count
	if (1 != file.scanf(CAMERA_COUNT_FORMAT, &mCameraCount))
		throw FileCorruptionException("Camera count isn't provided in correct camera file format.", file.getName());
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
	const uint32 fileCount = (uint32) children.size();
	for (uint32 fileIdx = 0; fileIdx < fileCount; ++fileIdx)
	{
		// is it a directory?
		const string &child = children[fileIdx];
		if (string::npos == child.find("view"))
			continue;

		const Path absFolder = Path::appendChild(mPath, child);
		if (!Directory::exists(absFolder))
			continue;
	
		const Path metaFileName = Path::appendChild(absFolder, "meta.ini");
		try
		{
			File cameraFile(metaFileName, File::OPEN_READING, false);
			loadFromMetaIniFile(views, oldToNewViewIDs, cameraFile, inverseInputRotation, inverseInputTranslation);
		}
		catch (Exception &exception)
		{
			// todo log this: "Could not open file: " + metaFileName
			exception;
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
		if (line.empty() || line[0] == '\n')
			continue;

		// comment?
		if ('#' == line[0])
			continue;

		// [camera] block start?
		if (string::npos != line.find("[camera]"))
		{
			readFocalLength(cameraData, file);
			readPixelAspectRatio(cameraData, file);
			readPrincipalPoint(cameraData, file);
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
		throw FileCorruptionException("Camera distortion parameters aren't provided in correct camera file format.", file.getName());
}

void MVECameraIO::readExtrinsics(CameraData &data, File &file,
	const Matrix3x3 &inverseInputRotation, const Vector3 &inverseInputTranslation)
{
	// read rotation matrix & translation vector
	Matrix3x3 rotation;
	Real *p = (Real *) rotation.values;
	Vector3 &pos = data.mPosition;

	// load rotation in matrix format
	if (9 != file.scanf(ROTATION_FORMAT, p, p + 1, p + 2, p + 3, p + 4, p + 5, p + 6, p + 7, p + 8, p + 9))
		throw FileCorruptionException("Camera rotation matrix isn't provided in correct camera file format.", file.getName());

	// load translation vector (MVE conventions t = -R^t * cameraPos with R as row major matrix for rhs column vectors)
	if (3 != file.scanf(TRANSLATION_FORMAT, &pos.x, &pos.y, &pos.z))
		throw FileCorruptionException("Camera translation vector isn't provided in correct camera file format.", file.getName());

	// adapt camera data to this project's conventions
	{
		// translation = -R^-1 * cam position -> position = -R * translation (loaded rotation is R as it is (R^-1)^t = R)
		pos = -pos * rotation;

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
		throw FileCorruptionException("Camera focal length isn't provided in correct camera file format.", file.getName());
}

void MVECameraIO::readPixelAspectRatio(CameraData &data, File &file)
{
	string line;
	file.readTextLine(line);

	if (1 != sscanf(line.c_str(), PIXEL_ASPECT_RATIO_FORMAT, &data.mPixelAspectRatio))
			throw FileCorruptionException("Camera focal length isn't provided in correct camera file format.", file.getName());
}

void MVECameraIO::readPrincipalPoint(CameraData &data, File &file)
{
	if (2 != file.scanf(PRINCIPAL_POINT_FORMAT, &data.mPrincipalPoint.x, &data.mPrincipalPoint.y))
		throw FileCorruptionException("Principal point isn't provided in correct camera file format.", file.getName());
}

void MVECameraIO::readViewData(map<uint32, uint32> &oldToNewViewIDs, CameraData &data, File &file, const uint32 newViewID)
{
	char nameBuffer[File::READING_BUFFER_SIZE];
	uint32 oldViewID;

	// get old view ID (MVE view IDs are not necessarily compact)
	if (1 != file.scanf(VIEW_ID_FORMAT, &oldViewID))
		throw FileCorruptionException("View ID isn't provided in correct camera file format.", file.getName());

	// link old to new view ID & set new view ID
	if (oldToNewViewIDs.end() != oldToNewViewIDs.find(oldViewID))
		throw FileCorruptionException("Duplicate view ID. All view IDs in MVE camera files must be unique!", file.getName());
	oldToNewViewIDs[oldViewID] = newViewID;
	data.mViewID = newViewID;

	// set view name
	if (1 != file.scanfString(CAMERA_NAME_FORMAT_0, CAMERA_NAME_FORMAT_1, nameBuffer, File::READING_BUFFER_SIZE))
		throw FileCorruptionException("Camera name isn't provided in correct camera file format.", file.getName());
	data.mName = nameBuffer;
}

void MVECameraIO::clear(vector<View *> &views, map<uint32, uint32> &oldToNewViewIDs)
{
	assert(views.empty());
	if (!views.empty())
		throw Exception("Views loading must always start from scratch!");
	oldToNewViewIDs.clear();
}

void MVECameraIO::saveCamerasToFile(const vector<View *> &views) const
{
	// view count
	const uint32 viewCount = (uint32) views.size();
	cout << "SyntheticScene:: Writing synthetic camera infos of " << viewCount << " cameras: " << mPath << "...\n";

	ofstream out(mPath.getString(), ios::binary);
	if (!out.good())
		throw FileException("Could not create a file to save data of synthetic cameras.", mPath);

	// file header
	out << HEADER_SIGNATURE;
	out << "camera_count = " << viewCount << "\n";
	out << "\n";

	// write all cameras infos to the file
	Matrix3x3 rotation;

	for (size_t viewIdx = 0; viewIdx < viewCount; ++viewIdx)
	{
		const View &view = *views[viewIdx];
		const PinholeCamera &camera = view.getCamera();
		const Vector4 pHWS = camera.getPosition();
		const Vector3 pWS(pHWS.x, pHWS.y, pHWS.z);
		const Vector2 &principalPoint = camera.getPrincipalPoint();

		// new camera with inverted looking direction for MVE convention
		const Vector3 viewDirection = view.getViewDirection();
		const Vector3 targetWS = pWS - viewDirection;

		// output intrinsics
		out << "focal_length = " << camera.getFocalLength() << "\n";
		out << "pixel_aspect = " << 1.0f << "\n";
		out << "principal_point = " << principalPoint.x << " " << principalPoint.y << "\n";
		out << "camera_distortion = " << 0.0f << " " << 0.0f << "\n";

		// convert extrinsics according to MVE conventions
		PinholeCamera mveCamera(camera);
		mveCamera.lookAt(pWS, targetWS, Vector3(0.0f, -1.0f, 0.0f)); // mve rotation conventions: flip y and z -> y down and z
		
		const Matrix4x4 viewMatrix = mveCamera.getViewMatrix();
		for (uint32 rowIdx = 0; rowIdx < 3; ++rowIdx)
			for (uint32 columnIdx = 0; columnIdx < 3; ++columnIdx)
				rotation(rowIdx, columnIdx) = viewMatrix(rowIdx, columnIdx);

		const Vector3 translation = -(pWS * rotation);// MVE directly uses this translation vector
		rotation.transpose(); // rotation in MVE format (they use column vectors and left matrices, e.g. R * t = rotatedT instead of this projects t * R = rotatedT)

		// output extrinsics
		out << "rotation = ";
		out << rotation(0, 0) << " " << rotation(0, 1) << " " << rotation(0, 2) << " ";
		out << rotation(1, 0) << " " << rotation(1, 1) << " " << rotation(1, 2) << " ";
		out << rotation(2, 0) << " " << rotation(2, 1) << " " << rotation(2, 2) << "\n";

		out << "translation = " << translation[AXIS_X] << " " << translation[AXIS_Y] << " " << translation[AXIS_Z] << "\n";

		// identifiers
		out << "id = " << viewIdx << "\n";
		out << "name = " << "v" << viewIdx << "\n\n";
	}
}
