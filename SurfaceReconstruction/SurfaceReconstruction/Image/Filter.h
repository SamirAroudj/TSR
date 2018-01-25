/*
 * Copyright (C) 2017 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */

#ifndef _FILTER_H_
#define _FILTER_H_

#include <cassert>
#include "Platform/DataTypes.h"
#include "Utilities/Size2.h"

// todo comments

namespace SurfaceReconstruction
{
	class ColorImage;

	class Filter
	{
	public:
		enum FilterType
		{
			GAUSSIAN,
			GAUSSIAN_DERIVATIVE_WRT_X,
			GAUSSIAN_DERIVATIVE_WRT_Y,
			FILTER_TYPE_COUNT
		};

	public:
		/** todo
		Creates a filter.
		@param type
			case GAUSSIAN:				params[0] = standard deviation
			case GAUSSIAN_DERIVATIVE_X: params[0] = standard deviation
			case GAUSSIAN_DERIVATIVE_Y: params[0] = standard deviation */
		Filter(const Real *params, const FilterType type);
		~Filter();


		Real computeSumOfCoefficients() const;

		inline Real getCoefficient(const int32 relativeX, const int32 relativeY) const;

		uint8 getConvolution(const uint32 x, const uint32 y, const uint32 channel, const ColorImage &image) const;

		inline const Utilities::ImgSize &getEnvSize() const;
		inline Utilities::ImgSize getSize() const;

	protected:
		void createGaussianBasedFunction(const Real standardDeviation, const FilterType type);
		void normalize();

	private:
		inline Filter(const Filter &copy);
		inline Filter &operator =(const Filter &rhs);

	private:
		Real *mFactors;				 /// Stores getSize().getElementCount() coefficients to define the filter kernel.
		Utilities::ImgSize mEnvSize; /// Defines how many pixels to the left (and right) and bottom (and top) of the center pixel still belong to the Filter domain.
									 /// (width of one row = mEnvWidth * 2 + 1, height of one column = mEnvSize[1] * 2 + 1)
	};

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///   inline function definitions   ////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	inline Real Filter::getCoefficient(const int32 relativeX, const int32 relativeY) const
	{
		assert(relativeX >= -((int32) mEnvSize[0]));
		assert(relativeX <= (int32) mEnvSize[0]);
		assert(relativeY >= -((int32) mEnvSize[1]));
		assert(relativeY <= (int32) mEnvSize[1]);

		const uint32 x = relativeX + mEnvSize[0];
		const uint32 y = relativeY + mEnvSize[1];
		return mFactors[y * getSize()[0] + x];
	}

	inline const Utilities::ImgSize &Filter::getEnvSize() const
	{
		return mEnvSize;
	}

	inline Utilities::ImgSize Filter::getSize() const
	{
		return Utilities::ImgSize(2 * mEnvSize[0] + 1,
								  2 * mEnvSize[1] + 1);
	}

	inline Filter::Filter(const Filter &copy) : mFactors(0)
	{
		assert(false);
	}

	inline Filter &Filter::operator =(const Filter &rhs)
	{
		assert(false);
		return *this;
	}
}

#endif // _FILTER_H_