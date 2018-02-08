/*
 * Copyright (C) 2018 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */
#ifndef _FSSF_PARAMETERS_H_
#define _FSSF_PARAMETERS_H_

#include "Platform/DataTypes.h"
#include "Utilities/Size2.h"

// todo comments

namespace SurfaceReconstruction
{
	struct FSSFParameters
	{
	public:
		FSSFParameters();

	public:
		// for minimum triangle edge lengths, relative to SVO
		Real mEdgeMergeRelativeThreshold;

		// detection of spiky geometry
		Real mSpikyGeometryAngleThreshold;
		Real mSpikyGeometryRelativeScaleThreshold;

		// detection of error-based subdivisions
		Real mSubdivisionRelativeErrorThreshold;
		Real mSubdivisionRelativeScaleMinimum;

		// projection confidence
		Real mProjectionConfidenceDistanceBandwidth;
		Real mProjectionConfidenceMaxAngleDifference;

		// surface support function threshold
		Real mSupportWeakThreshold;

		// convergence / divergence parameters
		Real mSurfaceErrorRelativeThreshold; /// for detection of local refinement divergence (revert updates if they were bad as in new error >= relative threshold * old error)

		// smoothing parameters
		Real mSmoothingUmbrellaLambdaHigh;
		Real mSmoothingUmbrellaLambdaLow;
		uint32 mSmoothingInitialIterCount;
		uint32 mSmoothingTaubinIterCount;

		// handling of isles of outliers
		uint32 mOutlierIsleMinKeepingSize;

		// super sampling pattern
		Utilities::Size2<uint32> mRaysPerViewSamplePair;
		bool mOrientSamplingPatternLikeView;
	};
}

#endif // _FSSF_PARAMETERS_H_