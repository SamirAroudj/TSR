/*
 * Copyright (C) 2018 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */
#ifndef _FSSF_STATISTICS_H_
#define _FSSF_STATISTICS_H_

#include <iostream>
#include "Platform/DataTypes.h"
#include "Platform/Storage/Path.h"

// todo comments

namespace SurfaceReconstruction
{
	class FSSFRefiner;

	class FSSFIterationStats
	{
	public:
		enum ErrorType
		{
			POSITIVE_ERROR,
			NEGATIVE_ERROR,
			ABSOLUTE_ERROR,
			ERROR_TYPE_COUNT
		};

	public:
		FSSFIterationStats();

		void addError(const Real &error);

		void clear();

		inline const uint32 getErrorCount(const ErrorType type) const;
		inline Real getMaxError() const;
		inline Real getMeanError(const ErrorType type) const;
		inline Real getMinError() const;
		inline Real getSurfaceError(const ErrorType type) const;

		FSSFIterationStats &operator +=(const FSSFIterationStats &sourceStats);

	public:
		static const char *msErrorDescriptions[FSSFIterationStats::ERROR_TYPE_COUNT];

	private:
		Real mMaxError;
		Real mMinError;
		Real mSurfaceErrors[3]; /// positive, negative and absolute ones
		uint32 mErrorCounts[3]; /// counts for mSurfaceErrors
	};

	class FSSFStatistics
	{
	public:
		FSSFStatistics();
		~FSSFStatistics();

		inline const std::vector<FSSFIterationStats> &getStats() const;
		
		void processIteration(const Real **errors, const Real **weights, const uint32 *arraySizes, const uint32 arrayCount,
			const Real weakSupportThreshold);

		bool hasConverged();

		void saveToFile(const Storage::Path &fileName) const;

	protected:
		void processErrors(const Real *errors, const Real *weights, const uint32 count, const Real weakSupportThreshold);
		
	private:
		std::vector<FSSFIterationStats> mStats;
		FSSFIterationStats *mStatsPerThread;
		Real mTargetErrorReductionThreshold;
		Real mTargetError;
		uint32 mFailedReductionCountMax;
		uint32 mFailedReductionCount;
	};

	std::ostream &operator <<(std::ostream &os, const FSSFIterationStats &rhs);
	std::ostream &operator <<(std::ostream &os, const FSSFStatistics &rhs);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///   inline function definitions   ////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	inline const uint32 FSSFIterationStats::getErrorCount(const ErrorType type) const
	{
		return mErrorCounts[type];
	}

	inline Real FSSFIterationStats::getMaxError() const
	{
		return mMaxError;
	}

	inline Real FSSFIterationStats::getMeanError(const ErrorType type) const
	{
		return mSurfaceErrors[type] / mErrorCounts[type];
	}

	inline Real FSSFIterationStats::getMinError() const
	{
		return mMinError;
	}

	inline Real FSSFIterationStats::getSurfaceError(const ErrorType type) const
	{
		return mSurfaceErrors[type];
	}

	const std::vector<FSSFIterationStats> &FSSFStatistics::getStats() const
	{
		return mStats;
	}
}

#endif // _FSSF_STATISTICS_H_