/*
 * Copyright (C) 2017 by Author: Aroudj, Samir
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
#include "Platform/ParametersManager.h"
#include "SurfaceReconstruction/Refinement/FSSFParameters.h"

using namespace FailureHandling;
using namespace Math;
using namespace Platform;
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
	ok &= m.get(mSupportSampleDistanceBandwidth, "FSSF::supportSampleDistanceBandwidth");
	ok &= m.get(mSupportSampleMaxAngleDifference, "FSSF::supportSampleMaxDegreesDifference");

	// weak surface support threshold
	ok &= m.get(mSupportWeakThreshold, "FSSF::supportWeakThreshold");

	// when to revert vertex position changes?
	ok &= m.get(mSurfaceErrorRelativeThreshold, "FSSF::surfaceErrorRelativeThreshold");

	// smoothing
	ok &= m.get(mUmbrellaSmoothingLambdaHigh, "FSSF::umbrellaSmoothingLambdaWeaklySupported");
	ok &= m.get(mUmbrellaSmoothingLambdaLow, "FSSF::umbrellaSmoothingLambdaWellSupported");
	
	// iteration counts
	ok &= m.get(mIterationCountInitialSmoothing, "FSSF::iterationCountInitialSmoothing");

	// rays per view cone
	ok &= m.get(mRaysPerViewSamplePair[0], "FSSF::raysPerViewSamplePairDim0");
	ok &= m.get(mRaysPerViewSamplePair[1], "FSSF::raysPerViewSamplePairDim1");
	ok &= m.get(mOrientSamplingPatternLikeView, "FSSF::orientSamplingPatternLikeView");

	// outlier removal stuff
	ok &= m.get(mOutlierIsleMinKeepingSize, "FSSF::outlierIsleMinKeepingSize");

	// convert degrees to angles
	mSpikyGeometryAngleThreshold = convertDegreesToRadians(mSpikyGeometryAngleThreshold);
	mSupportSampleMaxAngleDifference = convertDegreesToRadians(mSupportSampleMaxAngleDifference);

	// check that all parameters were ok properly
	if (ok)
		return;
	
	string message = "FSSF: Could not load all required parameters.";
	cerr << message << endl;
	throw Exception(message);
}