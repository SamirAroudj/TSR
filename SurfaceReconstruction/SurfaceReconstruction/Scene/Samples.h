/*
 * Copyright (C) 2017 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */
#ifndef _SAMPLES_H_
#define _SAMPLES_H_

#include <map>
#include <random>
#include "Math/KernelsInterpolationsWeightings.h"
#include "Math/Vector3.h"
#include "Platform/DataTypes.h"
#include "Platform/FailureHandling/Exception.h"
#include "Platform/Storage/Path.h"

namespace SurfaceReconstruction
{
	class CapturedScene;
	class Scene;
	class SyntheticScene;

	/// Represents all surface measurements (samples) consisting of point-based values such as positions, normals, scales etc.
	class Samples
	{
	public:
		friend class CapturedScene;
		friend class Scene;
		friend class SyntheticScene;

	public:
		static inline Real getDistanceConfidence(const Real distanceSquared);

		//static inline Real getDistanceCosts(const Math::Vector3 &n0, const Math::Vector3 &p0, const Real supportRange0,
		//							const Math::Vector3 &n1, const Math::Vector3 &p1, const Real supportRange1);

	public:
		/** todo */
		explicit Samples(const Storage::Path &fileName);

		/** Creates container for all the samples.
		@param viewsPerSamples Sets the maximum number of views which were used to create a single sample.
		@param sampleCount Reserves memory for sampleCount samples if sampleCount > 0. 
		@param AABBWS min max todo*/
		explicit Samples(const uint32 viewsPerSamples, const uint32 sampleCount = 0, const Math::Vector3 *AABBWS = NULL);

		/** Frees all samples. */
		~Samples();

		/** If necessary, this call extends the entered AABB so that it contains the sample object completely.
			The sphere of size sample sampleIdx's scale around its position is completely contained after function call.
		@param AABB set this to the current minimum and maximum corners of the AABB which shall be extended if necessary to contain the sample object.
		@param sampleIdx Identifies the sample. Must be a global index relative to the list of all scene samples.**/
		void addToAABB(Math::Vector3 AABB[2], const uint32 sampleIdx) const;

		/** Checks whether sample properties look valid. */
		void check();

		/** Removes samples according to entered offsets. 
		@param sampleOffsets todo*/
		void compact(const uint32 *sampleOffsets);

		/** todo */
		void computeAABB();

		/** todo */
		bool computeMeans(Math::Vector3 &meanColor, Math::Vector3 &meanNormal, Math::Vector3 &meanPosition, Real &meanScale,
			const std::vector<uint32> &sampleSet, bool weightedByConfidences) const;

		/** todo */
		bool computeViewAngles(Real &azimuthAngle, Real &polarAngle, const uint32 parentViewIdx, const uint32 sampleIdx) const;

		/** Computes and returns the normalized (unit length) vector view = (mPosition - parentCameraPosition).
		@param viewDirection Sets this vector to the normalized (unit length) vector viewDir = (mPosition - parentCameraPosition).
		@param parentViewIdx Identifies the parent view for which the view direction is computed. Set this to an index which is in {0, 1, ..., getViewsPerSample() }.
		@param sampleIdx Identifies the sample. Must be a global index relative to the list of all scene samples.
		@return Returns true if view direction could be computed as the parent exists otherwise false is returned if the sample has not a parentViewIdx-th parent view. */
		bool computeViewDirection(Math::Vector3 &viewDirection, uint32 parentViewIdx, const uint32 sampleIdx) const;

		/** todo */
		void erase(const std::vector<uint32> theDoomed, const uint32 doomedCount);

		/** Provides access to the axis aligned bounding box which tightly includes all sample center positions.
		@return Returns two world space vectors: the minimum [0] and maximum [1] corner of the AABB of all sample center positions. */
		inline const Math::Vector3 *getAABBWS() const;

		/** Computes the world space axis aligned bounding box of the specified Sample completely including its radial influence area.
		@param minWS Is set to the lower left corner of the AABB (relative to world space, inclusive).
		@param maxWS Is set to the upper right corner of the AABB (relative to world space, inclusive).
		@param sampleIdx Identifies the sample. Must be a global index relative to the list of all scene samples.*/
		void getAABBWS(Math::Vector3 &minWS, Math::Vector3 &maxWS, const uint32 sampleIdx) const;

		/** todo */
		inline const Math::Vector3 &getColor(const uint32 sampleIdx) const;

		/** todo */
		inline Real getConfidence(const uint32 sampleIdx) const;

		/** Returns the number of samples represented by this object.
		@return Returns the number of samples represented by this object.*/
		inline uint32 getCount() const;

		/** Computes the Euclidean distance between the world space point p and the specified Sample object which is treated like an infinitely large plane.
		@param pWS Set this to the world space point you want to know the distance for. (relative to the specified Sample plane)
		@param sampleIdx Identifies the sample. Must be a global index relative to the list of all scene samples.
		@return Returns the Euclidean distance between the world space point p and the specified planar Sample object. */
		Real getDistanceToPlane(const Math::Vector3 &pWS, const uint32 sampleIdx) const;

		/** Computes the FSSR weight at vSS.
		@param evaluationPosWS Set this to a point relative to the world space you want to know the FSSR weight of sample sampleIdx for.
		@return Returns the FSSR weight at vSS.
		@see Regarding sample scale values see this paper: "Simon Fuhrmann, Floating Scale Surface Reconstruction, SIGGRAPH 2014" */
		Real getFSSRWeight(const Math::Vector3 &evaluationPosWS, const uint32 sampleIdx) const;	

		/** todo */
		inline Real getKernel3DPoly3(const Math::Vector3 &queryPosWS, const uint32 sampleIdx, const Real supportRange) const;

		/** todo */
		inline Real getKernel3DPoly3Spiky(const Math::Vector3 &queryPosWS, const uint32 sampleIdx, const Real supportRange) const;

		/** todo */
		inline Real getKernel3DPoly6(const Math::Vector3 &queryPosWS, const uint32 sampleIdx, const Real supportRange) const;
		
		inline Real getMaxRelativeSamplingDistance() const;

		inline Real getMaxSamplingDistance(const uint32 sampleIdx) const;
		
		inline uint32 getMaxViewConeCount() const;
		
		Real getMeasureDistanceSquared(const uint32 sampleIdx, const uint32 parentViewIdx = 0) const;

		/** Returns the unit length normal of the specified sample object.
		@param sampleIdx Identifies the sample. Must be a global index relative to the list of all scene samples.
		@return Returns the unit length normal of the specified sample object.*/
		inline const Math::Vector3 &getNormalWS(const uint32 sampleIdx) const;
		
		inline uint32 getParentViewIdx(const uint32 viewConeIdx) const;

		/** Returns the array block which contains the links to the parent views of sample sampleIdx.
			A sample parent view link block might contain invalid links, View::INVALID_ID.
			(Each sample is assigned getViewsPerSample() link slots but might have fewer parent views.)
		@param sampleIdx Identifies the sample for which the parent views are returned. sampleIdx must be within [0, getCount() - 1].
		@return The returned array size is getViewsPerSample().
			The array might contain invalid links (View::INVALID_ID) since each view is assigned a constant number of view links for fast access (getViewsPerSample()).*/
		inline const uint32 *getParentViewIndices(const uint32 sampleIdx) const;

		/** Returns the world space center coordinate of the specified sample patch.
		@param sampleIdx Identifies the sample. Must be a global index relative to the list of all scene samples.
		@return Returns the world space center coordinate of the specified sample patch.*/
		inline const Math::Vector3 &getPositionWS(const uint32 sampleIdx) const;

		///** Computes the Euclidean distance between two samples' positions divided by the sum of their scales.
		//@param sampleIdx0 Identifies the first sample. Must be a global index relative to the list of all scene samples.*
		//@param sampleIdx1 Identifies the second sample. Must be a global index relative to the list of all scene samples.*
		//@return Returns ||(position0 - position1)|| / (scale0 + scale1) */
		//inline Real getRelativeDistance(const uint32 sampleIdx0, const uint32 sampleIdx1) const;

		inline uint32 getSampleIdx(const uint32 veiwConeIdx) const;

		/** Returns the scale of the specified sample which is the average distance to its neighbors which were captured by the same view. (~= sampling granularity)
		@param sampleIdx Identifies the sample. Must be a global index relative to the list of all scene samples.
		@return Returns the scale of the specified sample which is the average distance to its neighbors which were captured by the same view. (~= sampling granularity)
		@see Regarding sample scale values see this paper: "Simon Fuhrmann, Floating Scale Surface Reconstruction, SIGGRAPH 2014"*/
		inline const Real getScale(const uint32 sampleIdx) const;

		/** Returns the number of valid parent views which are available for all samples.
		(= number of valid links in the array getParentViewIndices() possibly unequal to the array size (array size = viewsPerSample * sample count))
		(parent views = responsible views which see a sample)
		@return The total number of sample parent views is returned. 
		@see getParentViewIndices */
		inline uint32 getViewConeCount() const;

		inline uint32 getViewIdx(const uint32 viewConeIdx) const;

		/** Each sample has up to getViewsPerSample parent views which were used to create the specified Sample.
			The global indices of these views can be accessed by this function.
		@param parentViewIdx Identifies what parent view w.r.t. the specified Sample is returned. Set this to a value in { 0, 1, ... getViewsPerSample() - 1 }.
		@param sampleIdx Identifies the sample. Must be a global index relative to the list of all scene samples.
		@return Returns the global index of the specified parent view whereas the index is relative to Scene::getViews().
		@see getViewsPerSample, Scene::getViews() */
		uint32 getViewIdx(const uint32 parentViewIdx, const uint32 sampleIdx) const;

		/** todo */
		inline uint32 getViewsPerSample() const;

		/** Adds normal noise to a Sample object.
		@param Contains three Gaussian distributions to add positional ([0]) & angular ([1]) noise as well as noise to the sample object's scale ([2]).
		@param sampleIdx Identifies the sample. Must be a global index relative to the list of all scene samples.*/
		void makeNoisy(std::normal_distribution<Real> noise[3], const uint32 sampleIdx);

		/** todo */
		void saveToFile(const Storage::Path &fileName, const bool saveAsPly, const bool saveAsSamples) const;

		/** todo */
		inline void set(const uint32 targetIdx, const Samples &sourceSamples, const uint32 sourceIdx);

		void setSample(const uint32 targetIdx, const Math::Vector3 &color, const Math::Vector3 &normal, const Math::Vector3 &positionWS,
			const Real confidence, const Real scale, const uint32 *parentViewIDs);

		/** Transforms a position in world space (pWS) into the specified Sample's local coordinate system where its center is the origin and the x-axis is its normal. (tangents = y-,z-axis).
		@param pWS Set this to the world space point you want to transform into the specified Sample object's local frame.
		@param sampleIdx Identifies the sample. Must be a global index relative to the list of all scene samples.
		@return The returned position is relative to the specified Sample object's local coordinate system. (origin = sample center, normal = x-axis)*/
		Math::Vector3 toSampleSpace(const Math::Vector3 &pWS, const uint32 sampleIdx) const;

		/** Applies the following operations to the chosen sample: s.p = s.p * transformation + translation, s.n = s.n * transformation.
		@param sampleIdx Set this to the index of the sample you want to transform. sampleIdx should be within the range [0, getCount() - 1].
		@param transformation This 3x3 matrix is applied to the sample normal and position. Could be a rotation for example.
		@param translation This vector is added to the sample position after the matrix transformation was applied. */
		void transform(const uint32 sampleIdx, const Math::Matrix3x3 &transformation, const Math::Vector3 &translation);

	private:
		/** Basic member initialization used by other constructors. */
		Samples();

		/** Copy constructor is forbidden. Don't use it. */
		inline Samples(const Samples &other);

		/** Assignment operator is forbidden. Don't use it.*/
		inline Samples &operator =(const Samples &rhs);

		/** todo */
		uint32 addSample();

		/** Releases resources. */
		void clear();

		/** Computes the number of valid sample to parent view links stored in mParentViews. */
		void computeParentViewCount();

		/** todo */
		void deleteSample(const uint32 sampleIdx);

		/** todo */
		void invalidateViews(const uint32 sampleIdx);

		/** todo */
		void loadFromFile(const Storage::Path &fileName);

		/** todo */
		void popBackSample();

		/** todo */
		void reserve(const size_t sampleCount);

		/** todo */
		void resize(const uint32 sampleCount);

		/** Sets the scale of the specified sample which is the average distance to its neighbors which were captured by the same view. (~= sampling granularity)
		@param scale The scale of the specified sample is the average distance to its neighbors which were captured by the same view. (~= sampling granularity)
		@param sampleIdx Identifies the sample. Must be a global index relative to the list of all scene samples.
		@see Regarding sample scale values see this paper: "Simon Fuhrmann, Floating Scale Surface Reconstruction, SIGGRAPH 2014"*/
		inline void setScale(const Real scale, const uint32 sampleIdx);

		/** todo */
		void swap(const uint32 index0, const uint32 index1);

		/** Each parent view ID of a sample for which view ID >= view count is true is set to View::INVALID_ID.*/
		void unifyInvalidParentViewIDs();
		
		/** todo */
		void updateParentViews(const std::map<uint32, uint32> &oldToNewViewIDs);

	public:
		static const uint32	FILE_VERSION;		/// Defines the current file version / current code version which is stored in .Samples files
		static const uint32 INVALID_INDEX;		/// Used for invalid links to samples.

	private:
		std::vector<Math::Vector3> mColors;		/// These are the RGB values of the samples.
		std::vector<Math::Vector3> mNormals;	/// These are the normals at the centers of the samples. (Each sample is treated like a planar surface patch.)
		std::vector<Math::Vector3> mPositions;	/// These are the centers of the samples.
		std::vector<Real> mConfidences;			/// Stores a confidence value for each sample.
		std::vector<Real> mScales;				/// Contains the average distance of a sample to its (same depth map) neighbors for each sample. (~= radius)
												/// Regarding sample scale values see this paper: "Simon Fuhrmann, Floating Scale Surface Reconstruction, SIGGRAPH 2014"
		std::vector<uint32> mParentViews;		/// Stores for each sample the mViewsPerSample IDs of the views which were used to create the sample.

		Math::Vector3 mAABBWS[2];				/// Minimum [0] and maximum[1] corner of all sample center positions in world space.
		Real mMaxRelativeSamplingDistance;		/// For each sample, there must be a sampling position (e.g., tree node center) at both of the sample sides with a distance of
												/// sample scale * max distance at maximum (w.r.t. the sample center).
		uint32 mViewConeCount;					/// Stores how many valid links are stored in mParentViews. (Number of parent views for all samples.)
		uint32 mViewsPerSample;					/// Stores how many view IDs are stored for each sample.
	};

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///   inline function definitions   ////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////	
	
	inline Samples::Samples(const Samples &other) :
		Samples(0)
	{
		assert(false);
	}

	//inline Real Samples::getDistanceConfidence(const uint32 sampleIdx, const uint32 parentViewIdx) const
	//{
	//	const Real dSq = getMeasureDistanceSquared(sampleIdx, parentViewIdx);
	//	return getDistanceConfidence(dSq);
	//}

	//inline Real Samples::getDistanceConfidence(const Real measureDistanceSquared)
	//{
	//	if (measureDistanceSquared < Math::EPSILON * Math::EPSILON)
	//		return 0.0f;
	//	return 1.0f / sqrtr(measureDistanceSquared);
	//}

	inline const Math::Vector3 *Samples::getAABBWS() const
	{
		return mAABBWS;
	}

	inline uint32 Samples::getCount() const
	{
		return (uint32) mNormals.size();
	}

	inline const Math::Vector3 &Samples::getColor(const uint32 sampleIdx) const
	{
		return mColors[sampleIdx];
	}

	inline Real Samples::getConfidence(const uint32 sampleIdx) const
	{
		return mConfidences[sampleIdx];
	}

	inline Real Samples::getKernel3DPoly3(const Math::Vector3 &queryPosWS, const uint32 sampleIdx, const Real R) const
	{
		// is queryPosWS outside compact support range?
		const Real r2 = (queryPosWS - mPositions[sampleIdx]).getLengthSquared();
		if (r2 >= R * R)
			return 0.0f;
		else
			return Math::getKernel3DPoly3(sqrtr(r2), R);
	}

	inline Real Samples::getKernel3DPoly3Spiky(const Math::Vector3 &queryPosWS, const uint32 sampleIdx, const Real R) const
	{
		// is queryPosWS outside compact support range?
		const Real r2 = (queryPosWS - mPositions[sampleIdx]).getLengthSquared();
		if (r2 >= R * R)
			return 0.0f;
		else
			return Math::getKernel3DPoly3Spiky(sqrtr(r2), R);
	}
	
	inline Real Samples::getKernel3DPoly6(const Math::Vector3 &queryPosWS, const uint32 sampleIdx, const Real R) const
	{		
		// get sample position & support range
		const Real r2 = (queryPosWS - mPositions[sampleIdx]).getLengthSquared();
		return Math::getKernel3DPoly6FromSquared(r2, R);
	}

	inline Real Samples::getMaxRelativeSamplingDistance() const
	{
		return mMaxRelativeSamplingDistance;
	}

	inline Real Samples::getMaxSamplingDistance(const uint32 sampleIdx) const
	{
		return getScale(sampleIdx) * mMaxRelativeSamplingDistance;
	}
	
	inline uint32 Samples::getMaxViewConeCount() const
	{
		const uint64 count = getCount() * getViewsPerSample();
		if (count >= ((uint32) -1))
			throw FailureHandling::Exception("Number of traced rays exceeds maximum count. Supported maximum is 2^32-1.");
		
		return (uint32) count;
	}

	inline const Math::Vector3 &Samples::getNormalWS(const uint32 sampleIdx) const
	{
		return mNormals[sampleIdx];
	}
	
	inline uint32 Samples::getParentViewIdx(uint32 viewConeIdx) const
	{
		return viewConeIdx % getViewsPerSample();
	}

	inline const uint32 *Samples::getParentViewIndices(const uint32 sampleIdx) const
	{
		return &mParentViews[sampleIdx * mViewsPerSample];
	}

	inline const Math::Vector3 &Samples::getPositionWS(const uint32 sampleIdx) const
	{
		return mPositions[sampleIdx];
	}

	//inline Real Samples::getRelativeDistance(const uint32 i, const uint32 j) const
	//{
	//	return (mPositions[i] - mPositions[j]).getLength() / (getSupportRange(i) + getSupportRange(j));
	//}

	inline uint32 Samples::getSampleIdx(const uint32 viewConeIdx) const
	{
		return viewConeIdx / getViewsPerSample();
	}

	inline const Real Samples::getScale(const uint32 sampleIdx) const
	{
		return mScales[sampleIdx];
	}
	
	inline uint32 Samples::getViewConeCount() const
	{
		return mViewConeCount;
	}
	
	inline uint32 Samples::getViewIdx(const uint32 viewConeIdx) const
	{
		return getViewIdx(getParentViewIdx(viewConeIdx), getSampleIdx(viewConeIdx));
	}

	inline uint32 Samples::getViewsPerSample() const
	{
		return mViewsPerSample;
	}

	inline Samples &Samples::operator =(const Samples &rhs)
	{
		assert(false);
		return *this;
	}

	inline void Samples::set(const uint32 targetIdx, const Samples &source, const uint32 sourceIdx)
	{
		setSample(targetIdx, source.mColors[sourceIdx], source.mNormals[sourceIdx], source.mPositions[sourceIdx],
			source.mConfidences[sourceIdx], source.mScales[sourceIdx], source.mParentViews.data() + sourceIdx * source.mViewsPerSample);
	}

	inline void Samples::setScale(const Real scale, const uint32 sampleIdx)
	{
		mScales[sampleIdx] = scale;
	}
}

#endif // _SAMPLES_H_
