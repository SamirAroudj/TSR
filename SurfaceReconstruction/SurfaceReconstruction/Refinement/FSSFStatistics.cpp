/*
 * Copyright (C) 2018 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */

#include <omp.h>
#include <iostream>
#include <sstream>
#include "Platform/FailureHandling/Exception.h"
#include "Platform/Storage/File.h"
#include "Platform/Utilities/ParametersManager.h"
#include "SurfaceReconstruction/Refinement/FSSFRefiner.h"
#include "SurfaceReconstruction/Refinement/FSSFStatistics.h"

using namespace FailureHandling;
using namespace Math;
using namespace std;
using namespace Storage;
using namespace SurfaceReconstruction;
using namespace Utilities;

const char *FSSFIterationStats::msErrorDescriptions[FSSFIterationStats::ERROR_TYPE_COUNT] =
{
	"(positive)",
	"(negative)",
	"absolute"
};

FSSFIterationStats::FSSFIterationStats()
{
	clear();
}

void FSSFIterationStats::clear()
{
	mMaxError = -REAL_MAX;
	mMinError = REAL_MAX;

	for (uint32 i = 0; i < 3; ++i)
	{
		mSurfaceErrors[i] = 0.0f;
		mErrorCounts[i] = 0;
	}
}

void FSSFIterationStats::addError(const Real &error)
{
	// update statistics
	if (error > mMaxError)
		mMaxError = error;
	if (error < mMinError)
		mMinError = error;

	if (error > 0.0f)
	{
		mSurfaceErrors[FSSFIterationStats::POSITIVE_ERROR] += error;
		++mErrorCounts[FSSFIterationStats::POSITIVE_ERROR];
	}
	else if (error < 0.0f)
	{
		mSurfaceErrors[FSSFIterationStats::NEGATIVE_ERROR] += error;
		++mErrorCounts[FSSFIterationStats::NEGATIVE_ERROR];
	}

	mSurfaceErrors[FSSFIterationStats::ABSOLUTE_ERROR] += fabsr(error);
	++mErrorCounts[FSSFIterationStats::ABSOLUTE_ERROR];
}

FSSFIterationStats &FSSFIterationStats::operator +=(const FSSFIterationStats &sourceStats)
{
	// keep min and max of both statistics
	if (sourceStats.mMaxError > mMaxError)
		mMaxError = sourceStats.mMaxError;
	if (sourceStats.mMinError < mMinError)
		mMinError = sourceStats.mMinError;
	
	// add other errors
	for (uint32 i = 0; i < 3; ++i)
	{
		mSurfaceErrors[i] += sourceStats.mSurfaceErrors[i];
		mErrorCounts[i] += sourceStats.mErrorCounts[i];
	}

	return *this;
}

FSSFStatistics::FSSFStatistics() :
	mTargetError(EPSILON), mFailedReductionCount(0)
{
	// create thread objects for parallel stats computation
	const uint32 maxNumThreads = omp_get_max_threads();
	mStatsPerThread = new FSSFIterationStats[maxNumThreads];

	// set parameters
	// required parameters
	const string relativeThresholdName = "FSSFStatistics::targetSurfaceErrorReductionThreshold";
	const string targetErrorName = "FSSFStatistics::targetSurfaceError";
	const string maxTimesFailedErrorReductionName = "FSSFStatistics::maxTimesFailedErrorReduction";

	// get parameters
	const ParametersManager &manager = ParametersManager::getSingleton();
	
	// relative threshold
	const bool loadedRelative = manager.get(mTargetErrorReductionThreshold, relativeThresholdName);
	const char *messageBeginning = "FSSFStatistics: Missing parameter value:\n";
	if (!loadedRelative)
	{
		string message = messageBeginning;
		message += relativeThresholdName;
		message += ", setting it to  0.05.\n";
		cerr << message << endl;

		mTargetErrorReductionThreshold = 0.05f;
	}
	
	// target absolute surface error
	const bool loadedTargetError = manager.get(mTargetError, targetErrorName);
	if (!loadedTargetError)
	{
		string message = messageBeginning;
		message += targetErrorName;
		message += ", setting it to EPSILON.\n";
		cerr << message << endl;

		mTargetError = EPSILON;
	}

	const bool loadedFailedReductionMax = manager.get(mFailedReductionCountMax, maxTimesFailedErrorReductionName);
	if (!loadedFailedReductionMax)
	{
		string message = messageBeginning;
		message += maxTimesFailedErrorReductionName;
		message += ", setting it to 3. \n";
		cerr << message<< endl;

		mFailedReductionCountMax = 3;
	}
}

FSSFStatistics::~FSSFStatistics()
{
	delete [] mStatsPerThread;
	mStatsPerThread = NULL;

	mStats.clear();
}

bool FSSFStatistics::hasConverged()
{
	if (mStats.back().getMeanError(FSSFIterationStats::ABSOLUTE_ERROR) < mTargetError)
		return true;
	
	return (mFailedReductionCount >= mFailedReductionCountMax);
}

void FSSFStatistics::processIteration(const Real **errors, const Real **weights, const uint32 *arraySizes, const uint32 arrayCount,
	const Real weakSupportThreshold)
{
	// clear stats of new iteration
	const uint32 maxNumThreads = omp_get_max_threads();
	for (uint32 i = 0; i < maxNumThreads; ++i)
		mStatsPerThread[i].clear();

	// gather new stats per thread
	for (uint32 i = 0; i < arrayCount; ++i)
		processErrors(errors[i], weights[i], arraySizes[i], weakSupportThreshold);

	// merge per thread data
	FSSFIterationStats target;
	for (uint32 i = 0; i < maxNumThreads; ++i)
		target += mStatsPerThread[i];
	mStats.push_back(target);
	
	// better than before?
	if (mStats.size() < 2)
	{
		mFailedReductionCount = 0;
		return;
	}

	// get last and second last surface error
	const uint32 statsCount = (uint32) mStats.size();
	const FSSFIterationStats &lastStats = mStats[statsCount - 1];
	const FSSFIterationStats &secondLast = mStats[statsCount - 2];

	// errors
	const Real meanErrorL = lastStats.getMeanError(FSSFIterationStats::ABSOLUTE_ERROR);
	const Real meanErrorSL = secondLast.getMeanError(FSSFIterationStats::ABSOLUTE_ERROR);
	const Real meanErrorReduction = meanErrorSL - meanErrorL;
	const Real relativeMeanErrorReduction = (meanErrorL > EPSILON ? meanErrorReduction / meanErrorL : 0.0f);
	if (isNaN(meanErrorL) || isNaN(meanErrorSL))
	{
		cout << "NaN surface error" << endl;
		++mFailedReductionCount;
	}
	else if (meanErrorL > meanErrorSL) // total error got worse && mean error got worse?
	{
		cout << "Surface error increased! :*(" << endl;
		++mFailedReductionCount;
	}
	else if (relativeMeanErrorReduction < mTargetErrorReductionThreshold) // low relative mean error reduction?
	{
		cout << "Low relative error reduction." << endl;
		++mFailedReductionCount;
	}
	else
	{
		cout << "Successfully reduced surface error!\n";
		mFailedReductionCount = 0;
	}
	
	// output
	cout << getStats().back() << "\n";
	cout << "Relative error reduction w.r.t. last error: " << relativeMeanErrorReduction << endl;
	cout << "Low error improvement count: " << mFailedReductionCount << endl;
}

void FSSFStatistics::processErrors(const Real *errors, const Real *weights, const uint32 count,
	const Real weakSupportThreshold)
{
	const int64 temp = count;

	// gather stats per thread
	#pragma omp parallel for
	for (int64 i = 0; i < temp; ++i)
	{
		// reliable estimate?
		if (weights[i] < weakSupportThreshold)
			continue;

		// add error to statistics
		const Real &error = errors[i];
		if (Math::isNaN(error))
			continue;
		if (REAL_MAX == error)
			continue;

		const uint32 threadIdx = omp_get_thread_num();
		FSSFIterationStats &data = mStatsPerThread[threadIdx];
		data.addError(error);
	}
}

void FSSFStatistics::saveToFile(const Path &fileName) const
{
	// get the stats
	stringstream stream;
	stream << *this;
	string text = stream.str();

	// save them
	File file(fileName, File::CREATE_WRITING, false);
	file.writeString(text, ENCODING_ASCII);
}

ostream &SurfaceReconstruction::operator <<(ostream &os, const FSSFIterationStats &rhs)
{
	os << "minimum, maximum error = " << rhs.getMinError() << " " << rhs.getMaxError() << "\n";	
	
	os << "Total & mean error and error count:\n";
	for (FSSFIterationStats::ErrorType errorType = (FSSFIterationStats::ErrorType) 0;
		errorType < FSSFIterationStats::ERROR_TYPE_COUNT;
		errorType = (FSSFIterationStats::ErrorType) (errorType + 1))
	{
		const uint32 errorCount = rhs.getErrorCount(errorType);
		os << rhs.msErrorDescriptions[errorType] << ": ";
		os << rhs.getSurfaceError(errorType) << " ";
		os << (errorCount > 0  ? rhs.getMeanError(errorType) : 0) << " ";	
		os << errorCount << "\n";
	}

	return os;
}

ostream &SurfaceReconstruction::operator <<(ostream &os, const FSSFStatistics &rhs)
{
	const vector<FSSFIterationStats> &stats = rhs.getStats();
	const uint32 iterationCount = (uint32) stats.size();

	for (uint32 iteration = 0; iteration < iterationCount; ++iteration)
	{
		const FSSFIterationStats &iterationStats = stats[iteration];

		os << "\nIteration " << iteration << ":\n";
		os << iterationStats;
	}

	return os;
}
