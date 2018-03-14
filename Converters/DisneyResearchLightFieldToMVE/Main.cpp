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
#include "Mvei.h"
#include "Image/Image.h"

using namespace Math;
using namespace Platform;
using namespace std;
using namespace Storage;

#define CCD_WIDTH_MM 36.0f //CCD width of the camera used in mm

bool findSourceData(vector<string> &images, vector<string> &depthMaps, const Path &undistortedDir, const Path &depthMapsDir);
bool getArguments(float &focalLengthMM, uint32 &focalLengthPixels, float &cameraSeparationMM,
	Path &undistortedDir, Path &depthMapsDir, Path &targetDir,
	const char **unformattedArguments, const int32 argumentCount);
bool handleExistingTargetDirectory(const Path &targetDir);
void writeMetaData(MetaData metaData, Path dir);
void outputDescription();



int32 main(int32 argumentCount, const char **unformattedArguments)
{
	// data paths
	Path undistortedDir;
	Path depthMapsDir;
	Path targetDir;
	Path viewsDir;
	// camera calibration
	float focalLengthMM;
	uint32 focalLengthPixels;
	float cameraSeparationMM;

	// input file names
	vector<string> images;
	vector<string> depthMaps;

	outputDescription();
	
	// get input arguments and source data locations
	getArguments(focalLengthMM, focalLengthPixels, cameraSeparationMM,	undistortedDir, depthMapsDir, targetDir,
		unformattedArguments, argumentCount);
	if (!findSourceData(images, depthMaps, undistortedDir, depthMapsDir))
		return 1;
	
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

	cout << "starting images" << endl;

	// for each image:
	// create folder
	// output .ini
	// output undistorted image
	// output depth map

	for (string img : images)
	{
		// create .mve folder
		string suffix(img.substr(img.rfind("_")+1));
		int id = stoi(suffix);
		suffix = suffix.substr(0, suffix.find("."));
		Path viewDir = Path::appendChild(viewsDir, Path("view_" + suffix + ".mve"));
		if (!Directory::createDirectory(viewDir))
		{
			cerr << "Could not create target directory: " << viewDir << endl;
			return 3;
		}

		// create meta.ini
		// TODO missing params
		int width = 5616;
		int height = 3744;

		cout << "translation: " << (id * (width * cameraSeparationMM / CCD_WIDTH_MM)) / max(width, height) << endl;

		MetaData meta;
		meta.data["view.id"] = to_string(id);
		meta.data["view.name"] = suffix;
		meta.data["camera.focal_length"] = to_string((float)focalLengthPixels / max(width, height)); // TODO get dims of image 
		meta.data["camera.pixel_aspect"] = "1";
		meta.data["camera.principal_point"] = "0.5 0.5";
		meta.data["camera.rotation"] = "1 0 0 0 1 0 0 0 1";
		meta.data["camera.translation"] = to_string((id * (width * cameraSeparationMM / CCD_WIDTH_MM)) / max(width, height)) + " 0 0";
		writeMetaData(meta, viewDir);


		// copy undistorted image into .mve folder
		ifstream source(Path::appendChild(undistortedDir, Path(img)).getCString(), ios::binary);
		ofstream dest(Path::appendChild(viewDir, Path("undist-L1.jpg")).getCString(), ios::binary);
		dest << source.rdbuf();
		source.close();
		dest.close();

	}

	cout << "images done" << endl;
	cout << "starting depth maps" << endl;

	for (string dm_fn : depthMaps) 
	{
		// read .dmaps
		cout << "Reading in depth map " << dm_fn << " ...";

		Encoding enc = ENCODING_BINARY_LITTLE_ENDIAN;
		Path src = Path::appendChild(depthMapsDir, Path(dm_fn));
		File dmf(src, File::FileMode::OPEN_READING, true);

		unsigned int width = dmf.readUInt32(enc);
		unsigned int height = dmf.readUInt32(enc);

		TypedImageBase<float>* dm = new TypedImageBase<float>();
		
		while (!dmf.endOfFileReached()) 
		{
			float d = dmf.readFloat(enc);
			dm->get_data().push_back(focalLengthPixels * cameraSeparationMM / d);
		}
		dm->resize(width, height, 1);
		cout << " done." << endl;
		cout << "Saving as .mvei ...";

		string suffix(dm_fn.substr(dm_fn.rfind("_") + 1));
		suffix = suffix.substr(0, suffix.find("."));
		Path viewDir = Path::appendChild(viewsDir, Path("view_" + suffix + ".mve"));
		save_mvei_file(dm, Path::appendChild(viewDir, Path("depth-L1.mvei")).getString(), IMAGE_TYPE_FLOAT);

		TypedImageBase<int32_t>* vm = new TypedImageBase<int32_t>();
		vm->resize(width, height, 1);
		vm->fill(stoi(suffix));
		save_mvei_file(vm, Path::appendChild(viewDir, Path("view-L1.mvei")).getString(), IMAGE_TYPE_SINT32);

		cout << " done." << endl;
	}

	cout << "depth maps done" << endl;


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
	

	if (images.size() != depthMaps.size())
	{
		cerr << "Number of undistorted images is not equal to the number of depth maps.\n";
		cerr << "Only depth maps from author C. Kim in original format (.dmap) are supported! Aborting.\n";
		return false;
	}

	return true;
}

bool getArguments(float &focalLengthMM, uint32 &focalLengthPixels, float &cameraSeparationMM,
	Path &undistortedDir, Path &depthMapsDir, Path &targetDir,
	const char **unformattedArguments, const int32 argumentCount)
{
	// process command line arguments
	vector<string> arguments;
	Utilities::getCommandLineArguments(arguments, unformattedArguments, argumentCount);
	if (6 != arguments.size())
	{
		cout << "Format: <focal length in mm> <focal length in pixels> <camera separation in mm> <undistorted images dir> <depth maps dir> <output dir = MVE scene dir>";
		return false;
	}

	
	uint32 currentArgument = 0;
	sscanf(arguments[currentArgument++].c_str(), "%f", &focalLengthMM);
	sscanf(arguments[currentArgument++].c_str(), "%u", &focalLengthPixels);
	sscanf(arguments[currentArgument++].c_str(), "%f", &cameraSeparationMM);

	undistortedDir = arguments[currentArgument++];
	depthMapsDir = arguments[currentArgument++];
	targetDir = arguments[currentArgument++];

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
	cout << "MVE – A Multi-View Reconstruction Environment\n";
	cout << "Simon Fuhrmann, Fabian Langguth and Michael Goesele\n"; 
	cout << "In: Proceedings of the Eurographics Workshop on Graphics and Cultural Heritage, Darmstadt, Germany, 2014.\n";
	cout << "Please see the project website for more information:\n";
	cout << "https://www.gcc.tu-darmstadt.de/home/proj/mve/" << "\n";
	cout << "\n";

	cout << "Please cite the above papers if you use any part of the datasets or software provided for the papers.\n";
}