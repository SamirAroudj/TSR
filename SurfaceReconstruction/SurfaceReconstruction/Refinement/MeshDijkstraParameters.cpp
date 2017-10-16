/*
 * Copyright (C) 2017 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */

#include "Math/MathHelper.h"
#include "Platform/FailureHandling/Exception.h"
#include "Platform/ParametersManager.h"
#include "SurfaceReconstruction/Refinement/MeshDijkstraParameters.h"

using namespace FailureHandling;
using namespace Math;
using namespace Platform;
using namespace std;
using namespace SurfaceReconstruction;

MeshDijkstraParameters::MeshDijkstraParameters()
{
	const ParametersManager &manager = ParametersManager::getSingleton();
	bool ok = true;

	ok &= manager.get(mAngularCostsFactor, "MeshDijsktra::angularCostsFactor");
	ok &= manager.get(mBandwidthFactor, "MeshDijsktra::bandwidthFactor");
	ok &= manager.get(mMaxAngleDifference, "MeshDijsktra::maxAngleDifference");
	
	mMaxAngleDifference = convertDegreesToRadians(mMaxAngleDifference);

	if (ok)
		return;

	cerr << "Missing MeshDijkstraParameter values in application configuration file!" << endl;
	throw Exception("Missing MeshDijkstraParamter values in app config file.");
}