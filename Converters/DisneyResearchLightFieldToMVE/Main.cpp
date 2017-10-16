/*
 * Copyright (C) 2017 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */
#include "Math/Vector3.h"
#include "Platform/Storage/Directory.h"
#include "Platform/Storage/File.h"
#include "Utilities/HelperFunctions.h"

using namespace Math;
using namespace Platform;
using namespace std;
using namespace Storage;

bool findSourceData(vector<string> &images, vector<string> &depthMaps, const Path &undistortedDir, const Path &depthMapsDir);
bool getArguments(float &focalLengthMM, uint32 &focalLengthPixels, float &cameraSeparationMM,
	Path &undistortedDir, Path &depthMapsDir, Path &targetDir,
	const char **unformattedArguments, const int32 argumentCount);
bool handleExistingTargetDirectory(const Path &targetDir);
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
	viewsDir = targetDir + "views";
	if (!Directory::createDirectory(viewsDir))
	{
		cerr << "Could not create views directory: " << viewsDir << endl;
		return 4;
	}

	// for each image:
	// create folder
	// output .ini
	// output undistorted image
	// output depth map

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

	//if (images.size() != depthMaps.size())
	//{
	//	cerr << "Number of undistorted images is not equal to the number of depth maps.\n";
	//	cerr << "Only depth maps from author C. Kim in original format (.dmap) are supported! Aborting.\n";
	//	return false;
	//}

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