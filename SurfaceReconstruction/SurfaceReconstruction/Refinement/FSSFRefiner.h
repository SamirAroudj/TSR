/*
 * Copyright (C) 2017 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */
#ifndef _FSSF_REFINER_H_
#define _FSSF_REFINER_H_

// todo comments

#include "SurfaceReconstruction/Geometry/IVertexChecker.h"
#include "SurfaceReconstruction/Geometry/Surfel.h"
#include "SurfaceReconstruction/Refinement/MeshDijkstraParameters.h"
#include "SurfaceReconstruction/Refinement/MeshRefiner.h"
#include "SurfaceReconstruction/Refinement/FSSFParameters.h"
#include "SurfaceReconstruction/Refinement/FSSFStatistics.h"

namespace SurfaceReconstruction
{
	class IslesEraser;
	class MeshDijkstra;

	class FSSFRefiner : public MeshRefiner, public IVertexChecker
	{
	public:
		enum VertexFlags
		{
			UNSUPPORTED		= 0x1 << 0,
			SPIKY			= 0x1 << 1,
			OUTLIER			= 0x1 << 2,
			BORDER_INLIER	= 0x1 << 3
		};

		struct ProjectedSample
		{
			Surfel mSurfel;
			Math::Vector3 mViewingDir;
			Real mConfidence;
		};

	public:
		static void findDepthExtrema(Real &minDepth, Real &maxDepth, const Real *depthMap, uint32 pixelCount);

	public:
		FSSFRefiner(const FlexibleMesh &initialMesh);
		FSSFRefiner(const std::string &meshFileName);
		~FSSFRefiner();

		void clear();

		//inline const std::vector<Math::Vector3> &getEdgeVectorField() const;
		//inline const std::vector<Real> &getEdgeWeights() const;
		inline const MeshDijkstra *getDijkstras() const;
		
		inline const MeshDijkstraParameters &getMeshDijkstraParameters() const;

		inline const FSSFParameters &getParameters() const;

		inline virtual bool isBad(const uint32 vertexIdx) const;

		void loadFromFile(const std::string &meshFileName);

		void refine();

	protected:
		static void saveMesh(const Mesh &mesh, const uint32 iteration, const char *text, const bool saveAsMesh = false);

	protected:
		FSSFRefiner();

		void addFloatingScaleQuantities(const MeshDijkstra &dijkstra, //const std::vector<Real> &edgeWeights,
			const ProjectedSample &projectedSample, const uint32 sampleIdx);
		void addFloatingScaleQuantities(Math::Vector3 *targetColor, Math::Vector3 &targetSumOfCorrections,
			Real &targetSumOfSurfaceErrors, Real &targetSumOfWeights,
			const Math::Vector3 &correctionDir, const Math::Vector3 &surfacePosWS,
			const Math::Vector3 *sampleColor, const Math::Vector3 &sampleNormalWS, const Math::Vector3 &samplePosWS,
			const Real weight);
		
		void addToOutliers(const std::vector<uint32> &newOutliers);

		void averageVertexAngles();

		bool computeErrorStatistics(const uint32 iteration);

		void computeOffsetsForUnsupportedGeometry(uint32 *vertexOffsets, uint32 *edgeOffsets, uint32 *triangleOffsets, uint32 oldVertexCount);

		/** todo 
		@return Returns true if FSSF has converged. */
		bool computeStatistics();

		//void computeSurfaceErrors(const uint32 iteration);

		Real computeSurfaceWeights(MeshDijkstra &dijkstra, //std::vector<Real> &edgeWeights,
			const Real sampleSurfaceSupport) const;

		void computeVertexAngles();
	
		void createNewVertex(const uint32 targetVertex, const uint32 vertexIdx0, const uint32 vertexIdx1, const Real f0);

		bool doRefinementStep(uint32 iteration);

		virtual void doSelfCheck();

		// methods for deletion of hallucinated geometry
		void deleteHallucinatedGeometry();

		// methods for deletion of outlier samples
		void deleteOutlierSamples();

		void enforceRegularGeometry(const uint32 iteration);
		void enforceRegularGeometryForOutliers(const uint32 iteration);

		void extendProjectedSample(std::vector<Math::Vector3> &hitNormalsWS, std::vector<uint32> &hitVertices, const Surfel &surfel);

		void fillHoles(const std::vector<std::vector<uint32>> &holeBorders);
		
		void findSpikyGeometry(const uint32 FSSFIteration, const uint32 smoothingIt);
		void findInlierSamples(bool *inliers);
		bool findInvalidBorderEdges();
		bool findIsolatedInliers();
		void findMergingEdges();
		void findOutliersUsingReliability();
		void findSubdivisionEdges();
		//void findSubdivisionEdgesViaErrors();
		void findSubdivisionEdgesViaNodes();
	
		uint32 *getEdgeMergeTriangleSearchSet(uint32 &triangleCount);
		Real getMinIntersectionTreeNodeLength(const Math::Vector3 triangle[3]) const;
		uint32 getNeighborCount(const uint8 flag, const uint32 vertexIdx) const;

		Real getProjectionConfidence(const Surfel &surfel, const uint32 sampleIdx) const;
		void getProjectedSample(ProjectedSample &projectedSample, const uint32 localPairIdx, const uint32 sampleIdx) const;
				
		bool getSurfaceErrorCorrection(Math::Vector3 &correction, 
			const Math::Vector3 &correctionDirection, const Math::Vector3 &surfacePosition,
			const Math::Vector3 &sampleNormalWS, const Math::Vector3 &samplePosWS) const;

		uint32 getVertexWithMostNeighbors(const uint32 triangle[3], const uint8 flag) const;

		bool hasNeighbor(const uint8 flag, const uint32 vertexIdx) const;

		bool isNotForSubdivision(const uint32 *triangle, const uint32 triangleIdx) const;

		inline bool isSpiky(const uint32 vertexIdx) const;

		void kernelInterpolation();

		//void limitVertexCorrections();

		void markBorderInliers();
		void markIsolatedVerticesAsOutliers();
		void markUnreliableVerticesViaSupport();

		void maskLargeOutlierIsles(const IslesEraser &outlierFinder);

		void moveVertices();
		bool moveSpikyGeometryBack();

		/** Normalizes weighted sums (e.g. mMovmentField, mColors) for weighted means using mWeightField. */
		void normalize();
		
		void createWellFormedOutlierIsles();

		virtual void onEdgeMerging(const uint32 targetVertex, const uint32 edgeVertex0, const uint32 edgeVertex1);
	
		virtual void onEdgeSplitVertex(const uint32 newVertexIdx, const uint32 edgeVertex0, const uint32 edgeVertex1);

		virtual void onFilterData(
			const uint32 *vertexOffsets, const uint32 vertexOffsetCount,
			const uint32 *edgeOffsets, const uint32 edgeOffsetCount,
			const uint32 *triangleOffsets, const uint32 triangleOffsetCount);

		virtual void onNewElements(
			const uint32 firstNewVertex, const uint32 newVertexCount,
			const uint32 firstNewEdge, const uint32 newEdgeCount,
			const uint32 firstNewTriangle, const uint32 newTriangleCount);

		virtual void onNewStartMesh();
		virtual void onReserveMemory(const uint32 vertexCapacity, const uint32 edgeCapacity, const uint32 indexCapacity);
		

		void outlierClosing();
		void outputErrorColoredMesh(Real *tempVertexColors, const Real *errors, const uint32 iteration, const std::string &coreName) const;

		void processProjectedSample(const ProjectedSample &projectedSample, const uint32 sampleIdx);

		void removeTangentialCorrections();

		void reserve(const uint32 vertexCapacity, const uint32 edgeCapacity, const uint32 triangleCapacity);

		void resize(const uint32 newVertexCount, const uint32 newEdgeCount, const uint32 newTriangleCount);
		
		void saveAngleColoredMesh(const uint32 iteration) const;
		void saveResult(const uint32 iteration);

		static void saveMeshForDebugging(FlexibleMesh &copy, 
			const uint32 iteration, const std::vector<std::vector<uint32>> &holeBorders, const uint32 afterDeletionVertexCount);
		
		void saveOutlierColoredMesh(const uint32 iteration) const;

		void simplifyMesh(const uint32 iteration);
		void subdivideMesh();
		
		bool smoothUntilConvergence(const uint8 requiredFlags);
		bool smoothIsleUntilConvergence(const std::vector<uint32> &isle);

		void zeroFloatingScaleQuantities();

	private:
		FSSFRefiner(const FSSFRefiner &copy);
		inline FSSFRefiner &operator = (const FSSFRefiner &rhs);

	public:
		static const uint32 EMBREE_PAIR_BATCH_SIZE;
		static const uint32 EMBREE_RAY_BATCH_SIZE;
		static const uint32 MEMORY_ALLOCATION_FACTOR;
		static const uint32 OMP_PAIR_BATCH_COUNT;
		static const uint32 OMP_PAIR_BATCH_SIZE;

	private:
		// triangle data
		std::vector<Math::Vector3> mTriangleNormals;
		
		//// edge data
		//std::vector<Math::Vector3> mEdgeVectorField;
		//std::vector<Real> mEdgeScales;
		//std::vector<Real> mEdgeWeights;

		// vertex data
		std::vector<Math::Vector3> mBestPositions;
		std::vector<Real> mBestSurfaceErrors;
		std::vector<Real> mSurfaceErrors;		/// For each vertex: scalar, signed error along corresponding normal.
		//std::vector<Real> mRelativeSurfaceErrors; /// = mSurfaceErrors normalized by vertex scale values.
		std::vector<uint8> mVertexStates;		/// Contains data to identify vertex support, outliers, bad normals used for replacing geometry or smoothing for robustness.
		std::vector<Real> mAverageAngles[2];

		// for surface hits processing
		std::vector<Real> *mLocalConfidences;

		// for quick Dijkstra searches
		MeshDijkstra *mDijkstras;						/// For each process: Dijkstra object for shortest path searches along surface.
		//std::vector<Real> *mLocalEdgeWeights;			/// Stores data-driven surface kernel edge weights for local surface refinements (subdivision).
		std::vector<uint32> mVertexNeighborsOffsets;	/// mVertexNeighbors[i] starts at mVertexNeighborsOffsets[i] and ends at (exclusive) mVertexNeighborsOffsets[i + 1];
		std::vector<uint32> mVertexNeighbors;			/// mVertexNeighbors[i] contains the global direct vertex neighbor indices of vertex i

		// merging & subdivision data
		std::vector<uint32> mEdgeMergeCandidates;
		std::vector<uint32> mLeftEdgeMergeCandidates;
		std::vector<uint32> mLeftTriangleMergeCandidates;
		std::vector<uint32> mSubdivisionEdges;

		// temp vector for enforcing regular geometry
		std::vector<uint32> mTempVertices;

		// iteration, error statistics & convergence stuff
		FSSFStatistics mStatistics;					/// Error statistics. Used to estimate convergence.
		uint32 mFirstHoleFillingTriangle;			/// triangles with indices >= mFirstHoleFillingTriangle until the last one were created to fill holes.

		const MeshDijkstraParameters mDijkstraParams;
		const FSSFParameters mParams;
	};

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///   inline function definitions   ////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	//inline const std::vector<Math::Vector3> &FSSFRefiner::getEdgeVectorField() const
	//{
	//	return mEdgeVectorField;
	//}

	//inline const std::vector<Real> &FSSFRefiner::getEdgeWeights() const
	//{
	//	return mEdgeWeights;
	//}

	inline const MeshDijkstra *FSSFRefiner::getDijkstras() const
	{
		return mDijkstras;
	}

	inline const MeshDijkstraParameters &FSSFRefiner::getMeshDijkstraParameters() const
	{
		return mDijkstraParams;
	}

	inline const FSSFParameters &FSSFRefiner::getParameters() const
	{
		return mParams;
	}
	
	inline bool FSSFRefiner::isBad(const uint32 vertexIdx) const
	{
		const uint8 badFlags = (UNSUPPORTED | OUTLIER);
		return 0 != (badFlags & mVertexStates[vertexIdx]);
	}
	
		
	inline bool FSSFRefiner::isSpiky(const uint32 vertexIdx) const
	{
		return 0 !=  (SPIKY & mVertexStates[vertexIdx]);
	}

	inline FSSFRefiner &FSSFRefiner::operator =(const FSSFRefiner &rhs)
	{
		assert(false);
		return *this;
	}
}

#endif // _FSSF_MESH_REFINER_H_
