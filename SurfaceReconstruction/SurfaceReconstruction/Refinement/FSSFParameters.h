/*
 * Copyright (C) 2017 by Author: Aroudj, Samir
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
		Real mEdgeMergeRelativeThreshold;

		Real mSpikyGeometryAngleThreshold;
		Real mSpikyGeometryRelativeScaleThreshold;

		Real mSubdivisionRelativeErrorThreshold;
		Real mSubdivisionRelativeScaleMinimum;

		Real mSupportSampleDistanceBandwidth;
		Real mSupportSampleMaxAngleDifference;

		Real mSupportWeakThreshold;

		Real mSurfaceErrorRelativeThreshold;

		Real mUmbrellaSmoothingLambdaHigh;
		Real mUmbrellaSmoothingLambdaLow;
	
		uint32 mIterationCountInitialSmoothing;
		uint32 mOutlierIsleMinKeepingSize;
		Utilities::Size2<uint32> mRaysPerViewSamplePair;
		bool mOrientSamplingPatternLikeView;
	};
}

#endif // _FSSF_PARAMETERS_H_