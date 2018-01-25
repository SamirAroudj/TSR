/*
 * Copyright (C) 2017 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */

#include "Math/MathCore.h"
#include "Math/MathHelper.h"
#include "Platform/FailureHandling/Exception.h"
#include "SurfaceReconstruction/Image/ColorImage.h"
#include "SurfaceReconstruction/Image/Filter.h"

using namespace FailureHandling;
using namespace Math;
using namespace SurfaceReconstruction;
using namespace Utilities;

Filter::Filter(const Real *parameters, const FilterType type) :
	mFactors(NULL), mEnvSize(0, 0)
{
	// get parameters
	switch (type)
	{
		case GAUSSIAN:
		case GAUSSIAN_DERIVATIVE_WRT_X:
		case GAUSSIAN_DERIVATIVE_WRT_Y:
		{
			createGaussianBasedFunction(parameters[0], type);
			break;
		}

		default:
		{
			assert(false);
			throw Exception("Filter::Filter(): Unimplemented filter type.");
			break;
		}
	}
}

void Filter::createGaussianBasedFunction(const Real standardDeviation, const FilterType type)
{
	mEnvSize[1] = 3 * (uint32) ceilr(standardDeviation);
	mEnvSize[0] = mEnvSize[1];
			
	// allocate memory
	const Real variance = standardDeviation * standardDeviation;
	const Real twoVariance = 2.0f * variance;
	const ImgSize size = getSize();

	mFactors = new Real[size.getElementCount()];

	// set the coefficients mFactors (not normalized)
	const Real centerX = (Real) mEnvSize[0];
	const Real centerY = (Real) mEnvSize[1];

	for (uint32 y = 0; y < size[1]; ++y)
	{
		const Real dY = y - centerY;

		for (uint32 x = 0; x < size[0]; ++x)
		{
			const Real dX = x - centerX;

			// e^(-((x - mu_x)^2 + (y - mu_y)^2) / (2 * variance))
			const Real exponent = -(dX * dX + dY * dY) / twoVariance;

			Real c = expr(exponent) / (Math::TWO_PI * variance);
			if (type == GAUSSIAN_DERIVATIVE_WRT_X)
				c *= -dX / variance;
			else if (type == GAUSSIAN_DERIVATIVE_WRT_Y)
				c *= -dY / variance;

			mFactors[y * size[0] + x] = c;
		}
	}

	if (GAUSSIAN == type)
		normalize();
}

void Filter::normalize()
{
	// normalize coefficients
	const Real sum = computeSumOfCoefficients();
	const Real normTerm = 1.0f / sum;
	const uint32 factorCount = getSize().getElementCount();

	for (uint32 i = 0; i < factorCount; ++i)
		mFactors[i] *= normTerm;
}

Real Filter::computeSumOfCoefficients() const
{
	const uint32 factorCount = getSize().getElementCount();

	// sum all coefficients
	Real sum = 0.0f;
	for (uint32 i = 0; i < factorCount; ++i)
		sum += mFactors[i];

	return sum;
}

Filter::~Filter()
{
	delete [] mFactors;
	mFactors = NULL;
}

uint8 Filter::getConvolution(const uint32 imageCenterX, const uint32 imageCenterY, const uint32 channel, const ColorImage &image) const
{
	// get meta data
	const ImgSize &size = image.getSize();
	
	// compute weighted sum within Filter domain
	Real sum = 0;
	const int32 sY = mEnvSize[1];
	const int32 sX = mEnvSize[0];

	for (int32 filterY = -sY; filterY <= sY; ++filterY)
	{
		const uint32 y = imageCenterY + filterY;
		if (y >= size[1])
			continue;

		for (int32 filterX = -sX; filterX <= sX; ++filterX)
		{
			const uint32 x = imageCenterX + filterX;
			if (x >= size[0])
				continue;

			const Real value = image.get(x, y, channel);
			sum += value * getCoefficient(filterX, filterY);
		}
	}

	// return the convolution result
	return (uint8) roundr(sum);
}