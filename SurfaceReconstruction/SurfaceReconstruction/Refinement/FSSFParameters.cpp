/*
 * Copyright (C) 2018 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */

#include <iostream>
#include <string>
#include "Math/MathHelper.h"
#include "Platform/FailureHandling/Exception.h"
#include "Platform/Utilities/ParametersManager.h"
#include "SurfaceReconstruction/Refinement/FSSFParameters.h"

using namespace FailureHandling;
using namespace Math;
using namespace std;
using namespace SurfaceReconstruction;
using namespace Utilities;

FSSFParameters::FSSFParameters()
{
	const ParametersManager &m = ParametersManager::getSingleton();

	// intersection fixing search length maximum & step size
	bool ok = true;
	ok &= m.get(mEdgeMergeRelativeThreshold, "FSSF::edgeMergeRelativeThreshold");

	// detection of spiky geometry
	ok &= m.get(mSpikyGeometryAngleThreshold, "FSSF::spikyGeometryAngleThreshold");
	ok &= m.get(mSpikyGeometryRelativeScaleThreshold, "FSSF::spikyGeometryRelativeScaleThreshold");

	// subdivision
	ok &= m.get(mSubdivisionRelativeErrorThreshold, "FSSF::subdivisionRelativeErrorThreshold");
	ok &= m.get(mSubdivisionRelativeScaleMinimum, "FSSF::subdivisionRelativeScaleMinimum");

	// sample support / mismatch confidence
	ok &= m.get(mProjectionConfidenceDistanceBandwidth, "FSSF::ProjectionConfidence::distanceBandwidth");
	ok &= m.get(mProjectionConfidenceMaxAngleDifference, "FSSF::ProjectionConfidence::maxDegreesDifference");

	// weak surface support threshold
	ok &= m.get(mSupportWeakThreshold, "FSSF::supportWeakThreshold");

	// when to revert vertex position changes?
	ok &= m.get(mSurfaceErrorRelativeThreshold, "FSSF::surfaceErrorRelativeThreshold");

	// smoothing
	ok &= m.get(mSmoothingUmbrellaLambdaHigh, "FSSF::Smoothing::umbrellaLambdaWeaklySupported");
	ok &= m.get(mSmoothingUmbrellaLambdaLow, "FSSF::Smoothing::umbrellaLambdaWellSupported");
	ok &= m.get(mSmoothingInitialIterCount, "FSSF::Smoothing::initialIterCount");
	ok &= m.get(mSmoothingTaubinIterCount, "FSSF::Smoothing::taubinIterCount");
	
	// rays per cone
	ok &= m.get(mRaysPerLinkedPair[0], "FSSF::raysPerLinkedPairDim0");
	ok &= m.get(mRaysPerLinkedPair[1], "FSSF::raysPerLinkedPairDim1");
	ok &= m.get(mOrientSamplingPattern, "FSSF::orientSamplingPattern");

	// outlier removal stuff
	ok &= m.get(mOutlierIsleMinKeepingSize, "FSSF::outlierIsleMinKeepingSize");

	// convert degrees to angles
	mSpikyGeometryAngleThreshold = convertDegreesToRadians(mSpikyGeometryAngleThreshold);
	mProjectionConfidenceMaxAngleDifference = convertDegreesToRadians(mProjectionConfidenceMaxAngleDifference);

	// check that all parameters were ok properly
	if (ok)
		return;
	
	string message = "FSSF: Could not load all required parameters.";
	cerr << message << endl;
	throw Exception(message);
}
