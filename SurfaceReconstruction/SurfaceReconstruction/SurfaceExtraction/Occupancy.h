/*
 * Copyright (C) 2018 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */
#ifndef _SURFACE_EXTRACTION_OCCUPANCY_H_
#define _SURFACE_EXTRACTION_OCCUPANCY_H_

// todo comments

#include <string>
#include "CollisionDetection/CollisionDetection.h"
#include "Math/Vector3.h"
#include "Platform/DataTypes.h"
#include "SurfaceReconstruction/SurfaceExtraction/DualMarchingCells.h"

namespace SurfaceReconstruction
{		
	// forward declarations
	class DualMarchingCells;
	class Tree;

	class Occupancy
	{
	public:
		enum DataType
		{
			EMPTINESS,
			SAMPLENESS,
			DATA_TYPE_COUNT
		};

		enum NodeStateFlag
		{
			NODE_FLAG_SAMPLENESS	= 0x1 << 0,	/// Flag is set for leaves the center of which overlaps with a sample and inner nodes which have any child with that flag set.
			NODE_FLAG_EMPTINESS		= 0x1 << 1	/// Flag is set for leaves the center of which overlaps with a view cone and inner nodes which have any child with that flag set.
			//NODE_FLAG_EMPTY			= 0x1 << 2,	/// Flag is set for leaves with isEmpty(leafIdx) and for all inner nodes which only have empty children.
			//NODE_FLAG_ANY_EMPTY		= 0x1 << 3	/// Flag is set for leaves with isEmpty(leafIdx) and for all inner nodes which have any empty child.
		};

	public:	
		static bool getViewConeData(Math::Vector3 &viewPosWS, uint32 &sampleIdx,
			const uint32 viewConeIdx, const uint32 *sampleOffsets = NULL);

	public:
		Occupancy(const Storage::Path &fileName);
		Occupancy(const Tree *tree);
		~Occupancy();
		
		bool addKernel(Real &target,
			const Math::Vector3 &leafCenter, const CollisionDetection::ObliqueCircularCone &viewCone, 
			const DataType type, const Real sampleConfidence, const bool negative) const;

		inline void addSamples(const uint32 *chosenSamples, const uint32 count);
		void computePriors();
		inline void eraseSamples(const uint32 *doomedSamples, const uint32 count);

		const DualMarchingCells &extractCrust();
		
		inline Real getBandwidthFactor() const;

		inline Real getConfidenceThreshold() const;

		inline const DualMarchingCells *getCrust() const;

		/** Returns the conditional probability density for having a view cone / empty space at leaf leafIdx.
		@param leafIdx todo.
		@return todo */
		Real getEmptinessCPD(const uint32 leafIdx) const;

		inline const uint32 *getNodeStates() const;

		inline Real getObservationPD(const uint32 leafIdx) const;

		Real getOccupancy(const uint32 leafIdx) const;

		inline Real getPriorForEmptiness(const uint32 leafIdx) const;
		inline Real getPriorForSampleness(const uint32 leafIdx) const;
		
		/** Returns the conditional probability density for having a sample near leaf leafIdx.
		@param leafIdx todo.
		@return todo */
		Real getSamplenessCPD(const uint32 leafIdx) const;

		void outputEdgeConflicts() const;

		void saveToFile(const Storage::Path &fileName) const;
		
	private:
		static Real computeCircularDiscKernelFromSquared(const Real distanceToDiscCenterSquared, const Real discRadius);
		static Real computeRayKernel(const Real distanceAlongRay, const Real viewConeLength);

	private:
		Occupancy();

		/** Copy constructor is forbidden. Don't use it. */
		inline Occupancy(const Occupancy &other);

		/** Assignment operator is forbidden. Don't use it.*/
		inline Occupancy &operator =(const Occupancy &rhs);

		void clear();

		void computeOccupancy();
		
		uint32 forwardAnyChildStateFlag(const uint32 nodeIdx, const NodeStateFlag flag);

		uint32 getMaxDepth(const uint32 sampleIndex) const;

		void loadParameters();

		bool isEmpty(const uint32 leafIdx) const;

		inline bool isUnreliable(const uint32 leafIdx) const;
		inline bool isUnreliable(const Real emptiness, const Real sampleness) const;

		void initializeSampleness();

		void loadFromFile(const Storage::Path &fileName);
		
		void onSamplesChanged(const bool remove, const uint32 *sampleIndices, const uint32 indexCount);
		
		inline void update(const bool forRemoval, const uint32 *chosenSamples, const uint32 chosenSampleCount, const uint32 requiredFlags);
		void updateKernels(const bool forRemoval, const DataType dataType, const uint32 *chosenSamples, const uint32 chosenSampleCount, const uint32 requiredFlags);

	public:
		static const uint32 FILE_VERSION;
		static const uint32 MAX_DEPTH_DIFFERENCE;
		static const uint32 OMP_PRIOR_LEAF_BATCH_SIZE;
		static const uint32 OMP_SAMPLE_BATCH_SIZE;
		static const uint32 OMP_VIEW_CONE_BATCH_SIZE;

	private:
		DualMarchingCells *mCrust;				/// This is used to get a coarse scene approximation from free space computations. It is later refined.
		
		Real *mConeLengths[DATA_TYPE_COUNT];
		Real *mKernelSums[DATA_TYPE_COUNT];
		Real *mPriorsForEmptiness;

		uint32 *mNodeStates;

		Real mConeCount;

		Real mConfidenceThreshold;
		Real mBandwidthFactor;
		Real mRelativeRadiusForPriors;
		Real mRelativeSampleConeLength;
	};
	
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///   inline function definitions   ////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	inline Occupancy::Occupancy(const Occupancy &other) :
		Occupancy()
	{
		assert(false);
	}

	inline Occupancy &Occupancy::operator =(const Occupancy &rhs)
	{
		assert(false);
		return *this;
	}
	
	inline void Occupancy::addSamples(const uint32 *chosenSamples, const uint32 count)
	{
		onSamplesChanged(false, chosenSamples, count);
	}

	inline void Occupancy::eraseSamples(const uint32 *doomedSamples, const uint32 count)
	{
		onSamplesChanged(true, doomedSamples, count);
	}

	inline Real Occupancy::getBandwidthFactor() const
	{
		return mBandwidthFactor;
	}

	inline Real Occupancy::getConfidenceThreshold() const
	{
		return mConfidenceThreshold;
	}

	inline const DualMarchingCells *Occupancy::getCrust() const
	{
		return mCrust;
	}

	inline const uint32 *Occupancy::getNodeStates() const
	{
		return mNodeStates;
	}

	inline Real Occupancy::getObservationPD(const uint32 leafIdx) const
	{
		return getPriorForEmptiness(leafIdx) * getEmptinessCPD(leafIdx) + getPriorForSampleness(leafIdx) * getSamplenessCPD(leafIdx);
	}

	inline Real Occupancy::getPriorForEmptiness(const uint32 leafIdx) const
	{
		return mPriorsForEmptiness[leafIdx];
	}

	inline Real Occupancy::getPriorForSampleness(const uint32 leafIdx) const
	{
		return (1.0f - mPriorsForEmptiness[leafIdx]);	
	}

	inline bool Occupancy::isUnreliable(const uint32 leafIdx) const
	{
		return isUnreliable(mKernelSums[EMPTINESS][leafIdx], mKernelSums[SAMPLENESS][leafIdx]);
	}

	inline bool Occupancy::isUnreliable(const Real summedEmptinessKernels, const Real summedSamplenessKernels) const
	{
		return (summedEmptinessKernels + summedSamplenessKernels) < mConfidenceThreshold;
	}
	
	inline void Occupancy::update(const bool forRemoval, const uint32 *chosenSamples, const uint32 chosenSampleCount, const uint32 requiredFlags)
	{
		updateKernels(forRemoval, SAMPLENESS, chosenSamples, chosenSampleCount, requiredFlags);
		updateKernels(forRemoval, EMPTINESS, chosenSamples, chosenSampleCount, requiredFlags);	
		computePriors();
	}
}

#endif // _SURFACE_EXTRACTION_OCCUPANCY_H_
