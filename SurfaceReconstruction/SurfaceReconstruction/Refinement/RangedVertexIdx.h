/*
 * Copyright (C) 2018 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */

#ifndef _RANGED_VERTEX_INDEX_H_
#define _RANGED_VERTEX_INDEX_H_

#include "Math/MathHelper.h"
#include "Math/Vector3.h"
#include "Platform/DataTypes.h"

// todo comments

namespace SurfaceReconstruction
{
	class RangedVertexIdx
	{
	public:
		static inline Real getAngularCostsFactor(const Math::Vector3 &n0, const Math::Vector3 &n1,
			const Real maxAngleDifference, const Real angularCostsFactor);
		static inline Real getDeltaCosts(const Math::Vector3 &n0, const Math::Vector3 &n1, const Math::Vector3 &p0, const Math::Vector3 &p1,
			const Real maxAngleDifference, const Real angularCostsFactor);

	public:
		inline RangedVertexIdx(const Real previousCosts, const Math::Vector3 &n0, const Math::Vector3 &n1, 
			const Math::Vector3 &p0, const Math::Vector3 &p1, const uint32 globalVertexIdx, const uint32 localPredecessorIdx,
			const Real maxAngleDifference, const Real angularCostsFactor);
		inline RangedVertexIdx(const Real costs, const uint32 globalVertexIdx, const uint32 localPredecessorIdx);
		
		inline Real getCosts() const;
		inline uint32 getGlobalVertexIdx() const;
		inline uint32 getLocalPredecessorIdx() const;

		inline void setCosts(const Real costs);
		inline void setPredecessor(const uint32 localIdx);

	private:
		Real mCosts;
		uint32 mGlobalVertexIdx;
		uint32 mLocalPredecessorIdx;
	};

	std::ostream &operator <<(std::ostream &out, const RangedVertexIdx &rangedVertexIdx);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///   inline function definitions   ////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	inline Real RangedVertexIdx::getAngularCostsFactor(const Math::Vector3 &n0, const Math::Vector3 &n1,
		const Real maxAngleDifference, const Real relativeAngularCostsFactor)
	{
		// angular difference of normals
		const Real dotProduct = n0.dotProduct(n1);
		const Real cosAngle = Math::clamp<Real>(dotProduct, 1.0f, -1.0f);
		const Real angle = acosr(cosAngle);
		if (angle >= maxAngleDifference)
			return REAL_MAX;//return relativeAngularCostsFactor;

		//// variant 0: with Gaussian
		//const Real stdDeviation = maxAngleDifference / 3.0f;
		//const Real f = (1.0f - expr((angle * angle) /(-2.0f * stdDeviation * stdDeviation)));

		// variant 1: infinite costs at angle deviation = maxAngleDifference
		// "angular costs" ac from angle between normals
		//if angle offset == half of the max possible angle -> ac = angularCostsFactor = 1.0f
		// infinite costs for angle offset >= max angle offset
		const Real f = Math::infinityMaximumCurve(angle, maxAngleDifference, relativeAngularCostsFactor);
		return 1.0f + f;

		// variant 2: with cos function and without maxAngleDifference
		//const Real f = (0.5f * (1.0f - cosAngle));
		// todo does not depend on maxAngleDifference

		// variant 3: with polynomial function
		//const Real temp = Math::clamp(maxAngleDifference - angle, maxAngleDifference, 0.0f);
		//const Real f = Math::smoothPolynomialFCurve(temp, maxAngleDifference);

		//// variant 4
		//Real f = angle / maxAngleDifference;
		//f = f * f * f;
		
		//const Real angularCostsFactor = 1.0f + f * (relativeAngularCostsFactor - 1.0f);
		//return angularCostsFactor;
	}

	inline Real RangedVertexIdx::getDeltaCosts(
		const Math::Vector3 &n0, const Math::Vector3 &n1,
		const Math::Vector3 &p0, const Math::Vector3 &p1,
		const Real maxAngleDifference, const Real angularCostsFactor)
	{
		// angular costs factor
		const Real f = getAngularCostsFactor(n0, n1, maxAngleDifference, angularCostsFactor);
		if (REAL_MAX == f)
			return REAL_MAX;

		// scaled Euclidean costs
		const Real distance = (p0 - p1).getLength();
		return distance * f;
	}

	inline RangedVertexIdx::RangedVertexIdx(const Real previousCosts, const Math::Vector3 &n0, const Math::Vector3 &n1, 
		const Math::Vector3 &p0, const Math::Vector3 &p1, const uint32 globalVertexIdx, const uint32 localPredecessorIdx,
		const Real maxAngleDifference, const Real angularCostsFactor) :
			RangedVertexIdx(previousCosts + getDeltaCosts(n0, n1, p0, p1, maxAngleDifference, angularCostsFactor),
				globalVertexIdx, localPredecessorIdx)
	{

	}

	inline RangedVertexIdx::RangedVertexIdx(const Real costs, const uint32 globalVertexIdx, const uint32 localPredecessorIdx) :
		mCosts(costs), mGlobalVertexIdx(globalVertexIdx), mLocalPredecessorIdx(localPredecessorIdx)
	{
		
	}

	inline Real RangedVertexIdx::getCosts() const
	{
		return mCosts;
	}
	
	inline uint32 RangedVertexIdx::getGlobalVertexIdx() const
	{
		return mGlobalVertexIdx;
	}

	inline uint32 RangedVertexIdx::getLocalPredecessorIdx() const
	{
		return mLocalPredecessorIdx;	
	}

	inline void RangedVertexIdx::setCosts(const Real costs)
	{
		mCosts = costs;
	}

	inline void RangedVertexIdx::setPredecessor(const uint32 localIdx)
	{
		mLocalPredecessorIdx = localIdx;
	}
}

#endif // _RANGED_VERTEX_INDEX_H_
