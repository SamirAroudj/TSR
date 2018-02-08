/*
 * Copyright (C) 2018 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */

#include <string>
#include <vector>
#include "Math/Vector2.h"
#include "Math/Vector3.h"
#include "Platform/Storage/Path.h"

class Evaluator
{
public:
	Evaluator(const std::string &groundTruthFile, const std::string &reconstructionFile);

	void saveResults(const std::string &targetFolder) const;

private:
	Math::Vector2 computeClosestPointToGroundTruth(uint32 &closestEdgeIdx, Real &distance, const Math::Vector2 &p, const Math::Vector2 &n);
	void computeErrors();
	
	void saveErrors(const std::string &targetFolder) const;
	void save2DMesh(const Storage::Path &fileName, const Math::Vector2 *oldPositions, const uint32 *oldIndices,
		const uint64 oldVertexCount, const uint64 oldIndexCount, const bool keepEdges) const;

private:
	// ground truth data
	std::vector<Math::Vector2> mGroundTruthPositions;
	std::vector<Math::Vector2> mGroundTruthEdgeNormals;
	std::vector<uint32> mGroundTruthIndices;

	// reconstruction data
	std::vector<Math::Vector2> mPositions;
	std::vector<Math::Vector2> mNormals;
	std::vector<uint32> mIndices;

	// error data
	std::vector<Math::Vector2> mClosestPoints;
	std::vector<Math::Vector2> mDirectedErrors;
	std::vector<Real> mSignedErrors;

	// AABB - bit smaller than ground truth AABB to filter mesh border geometry
	Math::Vector3 mAABB[2];
};