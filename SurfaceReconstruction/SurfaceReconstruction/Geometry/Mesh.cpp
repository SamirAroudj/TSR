/*
 * Copyright (C) 2018 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */

#include "Graphics/Color.h"
#include "Math/MathHelper.h"
#include "Platform/Storage/File.h"
#include "Platform/Utilities/PlyFile.h"
#include "SurfaceReconstruction/Geometry/Mesh.h"
#include "SurfaceReconstruction/Scene/FileNaming.h"

using namespace FailureHandling;
using namespace Graphics;
using namespace Math;
using namespace std;
using namespace Storage;
using namespace SurfaceReconstruction;
using namespace Utilities;

// Mesh constants
const uint32 Mesh::FILE_VERSION = 0;

void Mesh::applyMovementField(const Vector3 *vectorField)
{
	//cout << "Applying vertex movements." << endl;
	const Vector3 *positions = getPositions();
	const int64 vertexCount = getVertexCount();

	#pragma omp parallel for
	for (int64 vertexIdx = 0; vertexIdx < vertexCount; ++vertexIdx)
	{
		const Vector3 &move = vectorField[vertexIdx];
		const Vector3 &oldP = positions[vertexIdx];

		if (move.hasNaNComponent())
			continue;

		const Vector3 newP = oldP + move;
		getPosition((uint32) vertexIdx) = newP;
	}
}

void Mesh::computeAABB(Vector3 &min, Vector3 &max, const Vector3 *positions, const uint32 vertexCount)
{
	// initial bounding box values
	min.set(REAL_MAX, REAL_MAX, REAL_MAX);
	max.set(-REAL_MAX, -REAL_MAX, -REAL_MAX);

	// find min & max in O(n) ;)
	for (uint32 vertexIdx = 0; vertexIdx < vertexCount; ++vertexIdx)
	{
		const Vector3 &p = positions[vertexIdx];
		min = min.minimum(p);
		max = max.maximum(p);
	}
}

void Mesh::computeVertexScales(Real *scales, const vector<uint32> *verticesNeighbors, const Vector3 *positions, const uint32 vertexCount)
{
	// compute each vertex's mean distance to its vertex neighbors
	#pragma omp parallel for
	for (int64 vertexIdx = 0; vertexIdx < vertexCount; ++vertexIdx)
	{
		const Vector3 &center = positions[vertexIdx];
		const vector<uint32> &vertexNeighbors = verticesNeighbors[vertexIdx];
		const uint32 neighborCount = (uint32) vertexNeighbors.size();

		Real &scale = scales[vertexIdx];
		scale = computeAverageDistance(center, positions, vertexNeighbors.data(), neighborCount, (uint32) vertexIdx);
	}
}

void Mesh::computeVertexScales(Real *scales, const uint32 *neighbors, const uint32 *neighborOffsets, const Vector3 *positions, const uint32 vertexCount)
{	
	// compute each vertex's mean distance to its vertex neighbors
	#pragma omp parallel for
	for (int64 vertexIdx = 0; vertexIdx < vertexCount; ++vertexIdx)
		 scales[vertexIdx] = computeVertexScale(neighbors, neighborOffsets, positions, (uint32) vertexIdx);
}

Real Mesh::computeVertexScale(const uint32 *neighbors, const uint32 *neighborOffsets, const Vector3 *positions, const uint32 vertexIdx) 
{
	// get neighborhood data
	const Vector3 &center = positions[vertexIdx];
	const uint32 startIdx = neighborOffsets[vertexIdx];
	const uint32 endIdx = neighborOffsets[vertexIdx + 1];
	const uint32 neighborCount = endIdx - startIdx;
	const uint32 *vertexNeighbors = neighbors + startIdx;

	// average distance to neighbors
	const Real scale = computeAverageDistance(center, positions, vertexNeighbors, neighborCount, vertexIdx);
	return scale;
}

Real Mesh::computeAverageDistance(const Vector3 &center, const Vector3 *positions,
	const uint32 *neighborIndices, const uint32 indexCount, const uint32 skipIdx)
{
	uint32 count = indexCount;
	Real sum = 0.0f;

	for (uint32 localIdx = 0; localIdx < indexCount; ++localIdx)
	{
		const uint32 globalIdx = neighborIndices[localIdx];
		if (globalIdx == skipIdx)
		{
			--count;
			continue;
		}

		const Vector3 &p = positions[globalIdx];
		sum += (p - center).getLength();
	}

	if (0 == count)
		return -REAL_MAX;

	const Real average = sum / count;
	return average;
}

void Mesh::computeNormalsOfTriangles(Vector3 *normals, 
	const Vector3 *positions, const uint32 *indices, const uint32 indexCount)
{
	const uint32 triangleCount = indexCount / 3;

	#pragma omp parallel for
	for (int64 triangleIdx = 0; triangleIdx < triangleCount; ++triangleIdx)
	{
		// get triangle
		const uint32 *triangle = indices + 3 * triangleIdx;
		const Vector3 &v0 = positions[triangle[0]];
		const Vector3 &v1 = positions[triangle[1]];
		const Vector3 &v2 = positions[triangle[2]];

		// set normal
		computeTriangleNormal(normals[triangleIdx], v0, v1, v2);
	}
}

void Mesh::computeNormalsWeightedByAngles(Vector3 *normals,
	const Vector3 *positions, const uint32 vertexCount, const uint32 *indices, const uint32 indexCount)
{	
	//cout << "Computing normals for " << vertexCount << " vertices via " << indexCount / 3 << " triangles. \n";

	zeroNormals(normals, vertexCount);

	// for each triangle: add its normal to its adjacent normal vectors
	const int64 triangleCount = indexCount / 3;
	
	#pragma omp parallel for
	for (int64 triangleIdx = 0; triangleIdx < triangleCount; ++triangleIdx)
	{
		// get triangle
		const uint32 *triangle = indices + 3 * triangleIdx;
		const Vector3 &v0 = positions[triangle[0]];
		const Vector3 &v1 = positions[triangle[1]];
		const Vector3 &v2 = positions[triangle[2]];

		// get normal and edge directions
		Vector3 dir0 = v1 - v0;
		Vector3 dir1 = v2 - v1;
		Vector3 dir2 = v2 - v0;
		Vector3 normal = dir0.crossProduct(dir2);

		dir0.normalize();
		dir1.normalize();
		dir2.normalize();
		normal.normalize();

		// get cosines and angles
		const Real cosAlpha = dir0.dotProduct(dir2);
		const Real cosBeta = -dir0.dotProduct(dir1);
		const Real cosGamma = -dir1.dotProduct(-dir2);
		const Real angles[3] =
		{
			acosr(clamp<Real>(cosAlpha, 1.0f, -1.0f)),
			acosr(clamp<Real>(cosBeta, 1.0f, -1.0f)),
			acosr(clamp<Real>(cosGamma, 1.0f, -1.0f))
		};

		// update adjacent normals
		for (uint32 cornerIdx = 0; cornerIdx < 3; ++cornerIdx)
		{
			const uint32 vertexIdx = triangle[cornerIdx];
			const Vector3 weightedNormal = normal * angles[cornerIdx];

			// add triangle normal to vertex normal
			#pragma omp atomic
			normals[vertexIdx].x += weightedNormal.x;
			#pragma omp atomic
			normals[vertexIdx].y += weightedNormal.y;
			#pragma omp atomic
			normals[vertexIdx].z += weightedNormal.z ;
		}
	}

	normalizeNormals(normals, vertexCount);
}

void Mesh::computeNormalsWeightedByArea(Vector3 *normals, const Vector3 *positions, const uint32 vertexCount, const uint32 *indices, const uint32 indexCount)
{
	cout << "Computing normals for " << vertexCount << " vertices via " << indexCount / 3 << " triangles. \n";
	cout << "Normals are computed via area weighted average of adjacent triangles." << endl;

	zeroNormals(normals, vertexCount);

	// for each triangle: add its normal to its adjacent normal vectors
	const int64 triangleCount = indexCount / 3;
	
	#pragma omp parallel for
	for (int64 triangleIdx = 0; triangleIdx < triangleCount; ++triangleIdx)
	{
		const uint32 *triangle = indices + 3 * triangleIdx;

		// calculate triangle normal
		const Vector3 &v0 = positions[triangle[0]];
		const Vector3 &v1 = positions[triangle[1]];
		const Vector3 &v2 = positions[triangle[2]];

		Vector3 normal = (v1 - v0).crossProduct(v2 - v0);

		// update adjacent normals
		for (uint32 cornerIdx = 0; cornerIdx < 3; ++cornerIdx)
		{
			const uint32 vertexIdx = triangle[cornerIdx];

			// add triangle normal to vertex normal
			#pragma omp atomic
			normals[vertexIdx].x += normal.x;
			#pragma omp atomic
			normals[vertexIdx].y += normal.y;
			#pragma omp atomic
			normals[vertexIdx].z += normal.z;
		}
	}

	normalizeNormals(normals, vertexCount);
}

void Mesh::zeroNormals(Vector3 *normals, const uint32 count)
{
	#pragma omp parallel for
	for (int64 i = 0; i < count; ++i)
		normals[i].set(0, 0, 0);
}

void Mesh::normalizeNormals(Vector3 *normals, const uint32 count)
{
	#pragma omp parallel for
	for (int64 i = 0; i < count; ++i)
	{
		const Real lengthSq = normals[i].getLengthSquared();
		if (lengthSq < EPSILON * EPSILON)
			normals[i].set(-REAL_MAX, -REAL_MAX, -REAL_MAX);
		else
			normals[i] /= sqrtr(lengthSq);
	}
}

Mesh::~Mesh()
{

}

void Mesh::loadFromFile(const Path &fileName)
{
	clear();	
	
	const string plyEnding = FileNaming::ENDING_PLY;
	const string &name = fileName.getString();
	const uint32 nameLength = (uint32) name.length();
	
	// FileNaming::ENDING_MESH or FileNaming::ENDING_PLY?
	string fileNameEnding = name.substr(nameLength - plyEnding.size(), plyEnding.size());
	const bool fromPly = (fileNameEnding == plyEnding);
	
	if (fromPly)
	{
		loadFromPly(fileName);
		return;
	}
	
	const string meshEnding = FileNaming::ENDING_MESH;
	fileNameEnding = name.substr(nameLength - meshEnding.size(), meshEnding.size());
	const bool fromMesh = (fileNameEnding == meshEnding);
	if (fromMesh)
	{
		loadFromMesh(fileName);
		return;
	}

	throw FileException("Could not load mesh data. Unsupported mesh file type.", fileName);
}

void Mesh::loadFromMesh(const Path &fileName)
{
	// open file
	File file(fileName, File::OPEN_READING, true, FILE_VERSION);

	// read vertex & index count & reserve memory
	uint32 vertexCount = 0;
	uint32 indexCount = 0;
	file.read(&vertexCount, sizeof(uint32), sizeof(uint32), 1);
	file.read(&indexCount, sizeof(uint32), sizeof(uint32), 1);

	allocateMemory(vertexCount, indexCount);

	// read vertex buffers
	const uint64 vectorBytes = sizeof(Vector3) * vertexCount;
	const uint64 realBytes = sizeof(Real) * vertexCount;

	file.read(getColors(), vectorBytes, sizeof(Vector3), vertexCount);
	file.read(getNormals(), vectorBytes, sizeof(Vector3), vertexCount);
	file.read(getPositions(), vectorBytes, sizeof(Vector3), vertexCount);
	file.read(getScales(), realBytes, sizeof(Real), vertexCount);

	// read indices
	file.read((uint32 *) getIndices(), sizeof(uint32) * indexCount, sizeof(uint32), indexCount);

	cout << "Loaded mesh: " << fileName << "\n";
	cout << "Vertices: " << vertexCount << ", indices: " << indexCount << endl;
}

void Mesh::loadFromPly(const Path &fileName)
{
	// open file
	PlyFile file(fileName, File::OPEN_READING, true);

	// get meta data
	FacesDescription facesFormat;
	VerticesDescription verticesFormat;
	file.loadHeader(verticesFormat, &facesFormat);

	// allocate memory for vertices & load them
	const uint32 vertexCount = verticesFormat.getElementCount();

	allocateMemory(vertexCount, 0);
	loadVertices(file, verticesFormat);

	// load & set indices
	{
		vector<uint32> indices;
		file.loadTriangles(indices, facesFormat);

		const uint32 indexCount = (uint32) indices.size();
		setIndices(indices.data(), indexCount);
	}
}

void Mesh::loadVertices(PlyFile &file, const VerticesDescription &verticesFormat)
{
	// get data format
	const ElementsSemantics &semantics = verticesFormat.getSemantics();
	const ElementsSyntax &syntax = verticesFormat.getTypeStructure();
	const uint32 propertyCount = (uint32) syntax.size();
	const uint32 vertexCount = verticesFormat.getElementCount();

	for (uint32 vertexIdx = 0; vertexIdx < vertexCount; ++vertexIdx)
	{
		Vector3 *color = getColors() + vertexIdx;
		Vector3 *normal = getNormals() + vertexIdx;
		Vector3 &position = getPosition(vertexIdx);
		Vector2 *uvCoords = NULL; //mUVCoords + vertexIdx;
		Real *confidence = NULL; //mConfidences + vertexIdx;
		Real *scale = getScales() + vertexIdx;
		uint32 *parentIDs = NULL; // parentIDs + parentsPerSample * vertexIdx;

		color->set(0.5f, 0.5f, 0.5f);
		for (uint32 propertyIdx = 0; propertyIdx < propertyCount; ++propertyIdx)
			file.readVertexProperty(color, normal, position, uvCoords, confidence, scale, parentIDs,
				syntax[propertyIdx], (VerticesDescription::SEMANTICS) semantics[propertyIdx]);
	}
}

void Mesh::saveToFile(const Path &fileNameBeginning, const bool saveAsPly, const bool saveAsMesh) const
{	
	const uint32 vertexCount = getVertexCount();
	const uint32 indexCount = getIndexCount();

	// is there anything to save?
	if (0 == vertexCount || 0 == indexCount)
		return;
	
	// save the reconstruction in the internal and or in the ply file format

	// save in ply file format?
	if (saveAsPly)
	{
		const Path fileName = Path::extendLeafName(fileNameBeginning, FileNaming::ENDING_PLY);
		cout << "Saving " << fileName << "\n";

		PlyFile file(fileName, File::CREATE_WRITING, true);
		file.saveTriangleMesh(ENCODING_BINARY_LITTLE_ENDIAN, true,
			vertexCount, indexCount, getColors(), getNormals(), getPositions(),
			NULL, getScales(), NULL, 0, getIndices());
	}
	
	// save in internal file format?
	if (saveAsMesh)
	{
		const Path fileName = Path::extendLeafName(fileNameBeginning, FileNaming::ENDING_MESH);
		cout << "Saving " << fileName << "\n";

		File file(fileName, File::CREATE_WRITING, true, FILE_VERSION);

		// write vertex & index count
		file.write(&vertexCount, sizeof(uint32), 1);
		file.write(&indexCount, sizeof(uint32), 1);

		// write vertex buffers
		file.write(getColors(), sizeof(Vector3), vertexCount);
		file.write(getNormals(), sizeof(Vector3), vertexCount);
		file.write(getPositions(), sizeof(Vector3), vertexCount);
		file.write(getScales(), sizeof(Real), vertexCount);

		// write indices
		file.write(getIndices(), sizeof(uint32), indexCount);
	}

	cout << flush;
}


void Mesh::smoothByTaubinOp(Math::Vector3 *movementField, Real *weightField, const Real passBandEigenvalue, const Real smoothingLambda)
{
	// transfer function parameters: f(k) = (1 - smothingLambda * k) * (1 - smoothingMy * k) for Taubin operator
	Real temp = 1 - smoothingLambda * passBandEigenvalue;
	temp = 1.0f - 1.0f / temp;
	const Real smoothingMy = temp / passBandEigenvalue;
	
	// 1. the shrinking step
	{
		smoothByUmbrellaOp(movementField, weightField, smoothingLambda);
	}

	// 2. the un-shrinking step 
	{
		// compute laplacian again for each vertex
		const uint32 vertexCount = getVertexCount();
		computeLaplacianVectorField(movementField, weightField);

		// Taubin (un-shrinking) vector field
		#pragma omp parallel for
		for (int64 vertexIdx = 0; vertexIdx < vertexCount; ++vertexIdx)
		{
			const Vector3 laplacian = movementField[vertexIdx];
			movementField[vertexIdx] = laplacian * smoothingMy;
		}

		applyMovementField(movementField);
	}
}

void Mesh::smoothByUmbrellaOp(Vector3 *movementField, Real *weightField, const Real smoothingLambda)
{
	// compute laplacian for each vertex
	const uint32 vertexCount = getVertexCount();
	computeLaplacianVectorField(movementField, weightField);

	// umbrella smoothing vector field
	#pragma omp parallel for
	for (int64 vertexIdx = 0; vertexIdx < vertexCount; ++vertexIdx)
		movementField[vertexIdx] *= smoothingLambda;

	applyMovementField(movementField);
}

void Mesh::computeLaplacianVectorField(Vector3 *laplacianVectorField, Real *weightField)
{
	// zero movments & weights
	const uint32 vertexCount = getVertexCount();
	#pragma omp parallel for
	for (int64 vertexIdx = 0; vertexIdx < vertexCount; ++vertexIdx)
		laplacianVectorField[vertexIdx].set(0.0f, 0.0f, 0.0f);

	#pragma omp parallel for
	for (int64 vertexIdx = 0; vertexIdx < vertexCount; ++vertexIdx)
		weightField[vertexIdx] = 0.0f;

	// for each vertex: compute sum of weighted neighbors
	const uint32 indexCount = getIndexCount();
	const uint32 *indices = getIndices();

	#pragma omp parallel for
	for (int64 i = 0; i < indexCount; i += 3)
	{
		const uint32 *triangle = indices + i;
		for (uint32 cornerIdx = 0; cornerIdx < 3; ++cornerIdx)
			prepareLaplacianSmoothing(laplacianVectorField, weightField, triangle[cornerIdx], triangle[(cornerIdx + 1) % 3]);
	}

	// normalize weighted sums and subtract vertex positions to get umbrella operator movements
	#pragma omp parallel for
	for (int64 i = 0; i < vertexCount; ++i)
	{
		const uint32 vertexIdx = (uint32) i;
		const Real &sumOfWeights = weightField[vertexIdx];
		if (sumOfWeights < EPSILON)
			continue;

		const Vector3 &weightedSum = laplacianVectorField[vertexIdx];
		const Vector3 &position = getPosition(vertexIdx);
		const Vector3 laplacian = (weightedSum / sumOfWeights) - position;
		laplacianVectorField[vertexIdx] = laplacian;
	}
}

void Mesh::prepareLaplacianSmoothing(Vector3 *movementField, Real *weightField, const uint32 vertexIdx0, const uint32 vertexIdx1) const
{
	// vertex positions
	const Vector3 *positions = getPositions();
	const Vector3 &position0 = positions[vertexIdx0];
	const Vector3 &position1 = positions[vertexIdx1];

	// weight & movments
	const Real weight = 1.0f;//computeLaplacianWeight(position0, position1);
	const Vector3 t0 = position1 * weight;
	const Vector3 t1 = position0 * weight;

	// update summed movements & weights
	Vector3 &movement0 = movementField[vertexIdx0];
	Vector3 &movement1 = movementField[vertexIdx1];
	Real &sumOfWeights0 = weightField[vertexIdx0];
	Real &sumOfWeights1 = weightField[vertexIdx1];

	// udpate weights
	#pragma omp atomic
	sumOfWeights0 += weight;
	#pragma omp atomic
	sumOfWeights1 += weight;
	
	// upate movement 0
	#pragma omp atomic
	movement0.x += t0.x;
	#pragma omp atomic
	movement0.y += t0.y;
	#pragma omp atomic
	movement0.z += t0.z;
	
	// upate movement 1
	#pragma omp atomic
	movement1.x += t1.x;
	#pragma omp atomic
	movement1.y += t1.y;
	#pragma omp atomic
	movement1.z += t1.z;
}
