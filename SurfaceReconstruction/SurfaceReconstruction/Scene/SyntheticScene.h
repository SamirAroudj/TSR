/*
 * Copyright (C) 2017 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */
#ifndef _SYNTHETIC_SCENE_H_
#define _SYNTHETIC_SCENE_H_

#include "SurfaceReconstruction/Scene/Scene.h"
#include "Utilities/Size2.h"

namespace SurfaceReconstruction
{
	// forward declarations
	class RayTracer;

	/// Represents a Scene object which is created by means of a known ground truth and sampling of it.
	class SyntheticScene : public Scene
	{
	public:
		static bool isDepthDiscontinuity(const Real footprints[4], const Real blockDepths[4], const uint32 i1, const uint32 i2);

	public:
		/** todo */
		SyntheticScene(const Storage::Path &syntheticSceneMetaFileName, const std::vector<IReconstructorObserver *> &observers);

		/** Frees all synthetic scene data. */
		virtual ~SyntheticScene();

	private:
		/** Copy constructor is forbidden. Don't use it. */
		inline SyntheticScene(const SyntheticScene &other);

		/** Assignment operator is forbidden. Don't use it.*/
		inline SyntheticScene &operator =(const SyntheticScene &rhs);
		
		/** todo */
		void addNoise(std::vector<Math::Vector3> &hitsWS, std::vector<Real> &depthMap, const Math::Vector3 &camPosWS);
		
		void addToSamples(std::vector<std::vector<uint32>> &vertexNeighbors, std::vector<uint32> &indices, std::vector<uint32> &hitToVertexLinks,
			const std::vector<Math::Vector3> &hitsWS, const std::vector<Real> &depthMap, const std::vector<uint32> &pixelToHitMap,
			const uint32 hitCount, const uint32 viewIdx);

		/** todo */
		View *createSyntheticView(const uint32 viewIdx);
		void createAndSaveSamples();

		/** Samples the surfaces of input objects by means of rays orginating from entered view.
		@param view Set this to the view you want to create samples for.
		@param viewID Set this to the global index / identifier which is used to find the entered view. This is forwareded to the samples of the view.
		@return Returns the number of created samples. */
		uint32 createSamples(View &view, const uint32 viewID);

		/** todo */
		bool fill(std::vector<Real> &depthMap, std::vector<Math::Vector3> &hitsMap, std::vector<uint32> &pixelToHitMap,
			RayTracer &rayTracer, uint32 &hitCount, const Graphics::PinholeCamera &camera);
		
		/** Gets synthetic scene description from a parameters file.
		@param fileName Describes where to create the scene, what data to load, how to create the scene, etc.*/
		virtual bool getParameters(const Storage::Path &fileName);

		/** todo */
		void saveToFile(const std::vector<Real> &depthMap, const uint32 viewIdx, const bool withNoise) const;

		/** todo */
		void saveCamerasToFile(const Storage::Path &fileName) const;

		/** todo */
		FlexibleMesh *triangulate(std::vector<std::vector<uint32>> &vertexNeighbors, std::vector<uint32> &indices, std::vector<uint32> &hitToVertexLinks,
			const std::vector<Math::Vector3> &hits, const std::vector<Real> &depthMap, const std::vector<uint32> &pixelToHitMap, const uint32 hitCount,
			const Math::Matrix3x3 &pixelToViewSpace) const;
		uint32 triangulateBlock(std::vector<uint32> &indices, std::vector<uint32> &hitToVertexLinks, uint32 vertexCount,
			const std::vector<Math::Vector3> &hits, const std::vector<Real> &depthMap, const std::vector<uint32> &pixelToHitMap,
			const uint32 x, const uint32 y,	const Math::Matrix3x3 &pixelToViewSpace) const;		

	public:
		static const Real DEPTH_DIFFERENCE_FACTOR;
		static const uint32 RANDOM_SEED;

	private:
		Storage::Path mGroundTruthName;		// file name of ground truth object
		Real mDepthMapNoise[2];				// mean and standard deviation
		Math::Vector3 mAABB[2];				/// This is the axis aligned bounding box which contains mViews and mGroundTruth. ([0] -> min, [1] -> max)
		Math::Vector3 mMeshAABB[2];
		Math::Vector3 mRelativeSceneBorder;	/// The tight AABB (mMeshAABB) of all test input surfaces (mGroundTruth) is enlarged at all sides by these relative factors to get
											/// a complete scene AABB (mAABB) in which capturing views are placed.
		Utilities::ImgSize mViewResolution;	/// Defines how many surface samples are created per View object at maximum. This is the resolution of the view image planes in pixels. (width x height)

		Real mMaxFocalLength;
		Real mMinFocalLength;
		Real mViewBalance;

		// loaded view creation parameters

		Real mMinSampleViewAngle;			/// Defines the minimum angle between sample tangent and view vector that is required for Sample object creation.
		Real mMinSampleDistance;			/// Defines the mininum distance between a surface and a camera that is required for Sample creation (per sample).
		uint32 mSeed;						/// Defines what random numbers are generated to produce the sample noise.
		uint32 mMaxViewCount;				/// Defines how many projectice capturing views are created for sample point generation at maximum.
	};

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///   inline function definitions   ////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	inline SyntheticScene::SyntheticScene(const SyntheticScene &other) :
		Scene(other.mRefinerObservers)
	{
		assert(false);
	}

	inline SyntheticScene &SyntheticScene::operator =(const SyntheticScene &rhs)
	{
		assert(false);
		return *this;
	}
}

#endif // _SYNTHETIC_SCENE_H_

