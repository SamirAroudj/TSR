/*
 * Copyright (C) 2017 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */
#include <fstream>
#include <map>
#include <filesystem>

#include "Math/Vector3.h"
#include "Platform/Application.h"
#include "Platform/Storage/File.h"
#include "Platform/Utilities/HelperFunctions.h"
#include "Graphics/PinholeCamera.h"

#include "SurfaceReconstruction/Scene/Camera/CameraData.h"
#include "SurfaceReconstruction/Image/Image.h"
#include "SurfaceReconstruction/Image/MVEIHeader.h"
#include "SurfaceReconstruction/Image/DepthImage.h"

#include "Mvei.h"


using namespace Math;
using namespace Platform;
using namespace std;
using namespace Storage;


#define CCD_WIDTH_MM 36.0f //CCD width of the camera used in mm


bool findSourceData(vector<string> &images, vector<string> &depthMaps, const Path &undistortedDir, const Path &depthMapsDir);
bool getArguments(Path &undistortedDir, Path &depthMapsDir, Path &metaParamsFilePath, Path &targetDir,
	const char **unformattedArguments, const int32 argumentCount);
bool getMetaParams(Path metaParamsFilePath, float &focalLengthMM, uint32 &focalLengthPixels, float &cameraSeparationMM, int &rawWidth, int &rawHeight);
bool handleExistingTargetDirectory(const Path &targetDir);
void writeMetaData(MetaData metaData, Path dir);
void outputDescription();

using namespace SurfaceReconstruction;

int32 main(int32 argumentCount, const char **unformattedArguments)
{
	// data paths
	Path undistortedDir;
	Path depthMapsDir;
	Path targetDir;
	Path viewsDir;
	Path metaParamsFilePath;
	// camera calibration
	float focalLengthMM;
	uint32 focalLengthPixels;
	float cameraSeparationMM;
	int rawWidth;
	int rawHeight;

	// input file names
	vector<string> images;
	vector<string> depthMaps;

	outputDescription();
	
	// get input arguments and source data locations
	getArguments(undistortedDir, depthMapsDir, metaParamsFilePath, targetDir,
		unformattedArguments, argumentCount);
	if (!findSourceData(images, depthMaps, undistortedDir, depthMapsDir))
		return 1;
	getMetaParams(metaParamsFilePath, focalLengthMM, focalLengthPixels, cameraSeparationMM, rawWidth, rawHeight);

	// create scene & views directory
	if (!handleExistingTargetDirectory(targetDir))
		return 2;
	if (!Directory::createDirectory(targetDir))
	{
		cerr << "Could not create target directory: " << targetDir << endl;
		return 3;
	}
	viewsDir = Path::appendChild(targetDir, Path("views"));
	if (!Directory::createDirectory(viewsDir))
	{
		cerr << "Could not create views directory: " << viewsDir << endl;
		return 4;
	}

	cout << "creating scene directory and meta data ... ";

	for (string dm_fn : depthMaps) 
	{
		// create .mve folder
		string suffix(dm_fn.substr(dm_fn.rfind("_") + 1));
		int id = stoi(suffix);
		suffix = suffix.substr(0, suffix.find("."));
		Path viewDir = Path::appendChild(viewsDir, Path("view_" + suffix + ".mve"));
		if (!Directory::createDirectory(viewDir))
		{
			cerr << "Could not create target directory: " << viewDir << endl;
			return 3;
		}
		Image::setPathToImages(viewDir);
		cout << "Reading in depth map " << dm_fn << " ...";

		// read .dmaps
		Encoding enc = ENCODING_BINARY_LITTLE_ENDIAN;
		Path src = Path::appendChild(depthMapsDir, Path(dm_fn));
		File dmf(src, File::FileMode::OPEN_READING, true);

		// first two values denote the dimensions of the dmap
		const Utilities::ImgSize size = { dmf.readUInt32(enc), dmf.readUInt32(enc) };

		uint32 count = size.getElementCount();
		std::vector<float> disparities(count);
		std::vector<Real> depths(count);
		dmf.read(disparities.data(), sizeof(float) * count, sizeof(float), count);
		for (uint32 i = 0; i < count; ++i)
			depths[i] = focalLengthPixels * cameraSeparationMM / disparities[i];

		DepthImage *depthImage = DepthImage::createFromDisneyData(suffix, size, depths.data());

		CameraData camera = CameraData::CameraData();
		Real distortion[] = { 0.0f, 0.0f };
		float aspectRatio = ((float)rawWidth) / rawHeight;
		Math::Vector2 principalPoint(0.5f, 0.5f);
		Math::Vector4 posHWS(id * cameraSeparationMM, 0.0f, 0.0f, 1.0f);
		Math::Vector3 posWS(posHWS.x, posHWS.y, posHWS.z);

		Graphics::PinholeCamera pinholeCamera = Graphics::PinholeCamera(Math::Quaternion(), posHWS,
			(float)focalLengthPixels / 5616.0f, principalPoint, aspectRatio, distortion);
		pinholeCamera.lookAt(posWS, posWS + Vector3(0.0f, 0.0f, 1.0f), Vector3(0.0f, 1.0f, 0.0f));
		camera.set(id, pinholeCamera);
		

		depthImage->setDepthConvention(pinholeCamera, DepthImage::DepthConvention::DEPTH_ALONG_RAY);
		depthImage->saveAsMVEFloatImage(Path("depth-L1.mvei"), false, false);

		cout << " Saving as .mvei ...";
	
		// create views map with view id at every pixel
		std::vector<int32_t> vm_data;
		vm_data.resize(size[0] * size[1]);
		std::fill(vm_data.begin(), vm_data.end(), id);

		const MVEIHeader header(size, 1, MVEIHeader::MVE_SINT32);
		Image::saveAsMVEI(Path::appendChild(viewDir, Path("views-L1.mvei")), false, header, &vm_data[0]);

		cout << " done." << endl;

		MetaData meta;
		meta.data["view.id"] = to_string(id);
		meta.data["view.name"] = suffix;
		meta.data["camera.focal_length"] = to_string((float)focalLengthPixels / rawWidth);
		meta.data["camera.pixel_aspect"] = "1";
		meta.data["camera.principal_point"] = "0.5 0.5";
		meta.data["camera.rotation"] = "1 0 0 0 1 0 0 0 1";
		meta.data["camera.translation"] = to_string(id * cameraSeparationMM) + " 0 0";
		writeMetaData(meta, viewDir);


		// copy undistorted image into .mve folder
		ifstream source(Path::appendChild(undistortedDir, Path(images[id])).getCString(), ios::binary);
		ofstream dest(Path::appendChild(viewDir, Path("undist-L1.jpg")).getCString(), ios::binary);
		dest << source.rdbuf();
		source.close();
		dest.close();
	}


	return 0;
}

bool findSourceData(vector<string> &images, vector<string> &depthMaps, const Path &undistortedDir, const Path &depthMapsDir)
{
	Directory::findChildren(images, undistortedDir, ".jpg");
	Directory::findChildren(depthMaps, depthMapsDir, ".dmap");

	if (0 == images.size())
	{
		cerr << "Found zero source images. JPG format is required (.jpg)! Aborting.\n";
		return false;
	}

	return true;
}

bool getArguments(Path &undistortedDir, Path &depthMapsDir, Path &metaParamsFilePath, Path &targetDir,
	const char **unformattedArguments, const int32 argumentCount)
{
	// process command line arguments
	vector<string> arguments;
	Utilities::getCommandLineArguments(arguments, unformattedArguments, argumentCount);
	if (4 != arguments.size())
	{
		cout << "\nUsage:\n" << unformattedArguments[0] << " <undistorted images dir> <depth maps dir> <meta.txt path> <output dir = MVE scene dir>\n";
		cerr << "Error: invalid argument count.\n";
		return false;
	}

	uint32 currentArgument = 0;

	undistortedDir = arguments[currentArgument++];
	depthMapsDir = arguments[currentArgument++];
	metaParamsFilePath = arguments[currentArgument++];
	targetDir = arguments[currentArgument++];

	return true;
}

bool getMetaParams(Path metaParamsFilePath, float &focalLengthMM, uint32 &focalLengthPixels, float &cameraSeparationMM, int &rawWidth, int &rawHeight) {
	File metaFile(metaParamsFilePath, File::FileMode::OPEN_READING, true);

	while (!metaFile.endOfFileReached())
	{
		string line;
		metaFile.readTextLine(line);
		string key = line.substr(0, line.find("="));
		string val = line.substr(line.find("=")+1, line.size());
		if (key != "flen_mm" && key != "flen_px" && key != "baseline" && key != "raw_width" && key != "raw_height") {
			cerr << "Incorrect meta.txt file format. Only flen_mm, flen_pix and baseline are allowed parameters." << endl;
			return false;
		}
		
		if (key == "flen_mm")
			focalLengthMM = stof(val);
		else if (key == "flen_px")
			focalLengthPixels = stoi(val);
		else if (key == "baseline")
			cameraSeparationMM = stof(val);
		else if (key == "raw_width")
			rawWidth = stoi(val);
		else if (key == "raw_height")
			rawHeight = stoi(val);

	}

	return true;
}

bool handleExistingTargetDirectory(const Path &targetDir)
{
	// check for existance
	if (!Directory::exists(targetDir))
		return true;

	// ask user
	cout << "Directory \"" << targetDir << "\" already exists!\n";
	cout << "Continue? (y = yes / n = no)\n";
	char choice = 'n';
	cin >> choice;

	// continue?
	if (choice == 'y')
	{
		cout << "Continuing on user request.\n";
		return true;
	}
	
	// stop
	cout << "Stopping on user request.\n";
	return false;
}


void outputDescription()
{
	cout << "Converter for data from Kim's lightfield reconstruction to MVE.\n\n";

	cout << "Input data:\n";
	cout << "Scene Reconstruction from High Spatio-Angular Resolution Light Fields\n";
	cout << "C. Kim, H. Zimmer, Y. Pritch, A. Sorkine-Hornung, and M. Gross.\n";
	cout << "In: ACM Transactions on Graphics 32(4) (Proceedings of ACM SIGGRAPH 2013)\n";
	cout << "Please see the project website for more information:\n";
	cout << "http://www.disneyresearch.com/project/lightfields" << "\n";
	cout << "\n";

	cout << "Output data:\n";
	cout << "MVE - A Multi-View Reconstruction Environment\n";
	cout << "Simon Fuhrmann, Fabian Langguth and Michael Goesele\n"; 
	cout << "In: Proceedings of the Eurographics Workshop on Graphics and Cultural Heritage, Darmstadt, Germany, 2014.\n";
	cout << "Please see the project website for more information:\n";
	cout << "https://www.gcc.tu-darmstadt.de/home/proj/mve/" << "\n";
	cout << "\n";

	cout << "Please cite the above papers if you use any part of the datasets or software provided for the papers.\n";
}