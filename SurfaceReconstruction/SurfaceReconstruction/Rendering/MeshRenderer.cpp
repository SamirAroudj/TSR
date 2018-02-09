/*
 * Copyright (C) 2018 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */

#include <GL/glew.h>
#include <GL/gL.h>
#include "Platform/FailureHandling/GraphicsException.h"
#include "Math/Vector3.h"
#include "Platform/Storage/File.h"
#include "SurfaceReconstruction/Geometry/Mesh.h"
#include "SurfaceReconstruction/Rendering/MeshRenderer.h"

using namespace FailureHandling;
using namespace Math;
using namespace std;
using namespace Storage;
using namespace SurfaceReconstruction;

const Path MeshRenderer::SHADER_DIRECTORY = "Data/Shaders/";
const uint32 MeshRenderer::VERTEX_BUFFER_BINDING_INDEX = 0;

MeshRenderer::MeshRenderer()
{
	setupShaders();
}

void MeshRenderer::setupShaders()
{
	// load vertex & fragment shader
	const Path vertexShaderFileName = Path::appendChild(SHADER_DIRECTORY, "PNCVertexShader.glsl");
	const Path fragmentShaderFileName = Path::appendChild(SHADER_DIRECTORY, "PNCFragmentShader.glsl");
	string vertexShader;
	string fragmentShader;
	File::loadTextFile(vertexShader, vertexShaderFileName);
	File::loadTextFile(fragmentShader, fragmentShaderFileName);

	// create vertex shader
	const uint32 vertexShaderSourceCount = 1;
	const char *vertexShaderSources[vertexShaderSourceCount] = { vertexShader.c_str() };
	const int32 vertexShaderSourceLengths[vertexShaderSourceCount] = { (int32) vertexShader.length() };
	mPNCProgramIDs[INDEX_VERTEX_SHADER] = glCreateShader(GL_VERTEX_SHADER);
	glShaderSource(mPNCProgramIDs[INDEX_VERTEX_SHADER], vertexShaderSourceCount, vertexShaderSources, vertexShaderSourceLengths);
	glCompileShader(mPNCProgramIDs[INDEX_VERTEX_SHADER]);

	// create fragment shader
	const uint32 fragmentShaderSourceCount = 1;
	const char *fragmentShaderSources[fragmentShaderSourceCount] = { fragmentShader.c_str() };
	const int32 fragmentShaderSourceLengths[fragmentShaderSourceCount] = { (int32) fragmentShader.length() };
	mPNCProgramIDs[INDEX_FRAGMENT_SHADER] = glCreateShader(GL_FRAGMENT_SHADER);
	glShaderSource(mPNCProgramIDs[INDEX_FRAGMENT_SHADER], fragmentShaderSourceCount, fragmentShaderSources, fragmentShaderSourceLengths);
	glCompileShader(mPNCProgramIDs[INDEX_FRAGMENT_SHADER]);

	// put vertex & fragment shader together in one program
	mPNCProgramIDs[INDEX_PROGRAM] = glCreateProgram();
	glAttachShader(mPNCProgramIDs[INDEX_PROGRAM], mPNCProgramIDs[INDEX_VERTEX_SHADER]);
	glAttachShader(mPNCProgramIDs[INDEX_PROGRAM], mPNCProgramIDs[INDEX_FRAGMENT_SHADER]);

	// specify mapping of vertex attributes to shader variables
	glBindAttribLocation(mPNCProgramIDs[INDEX_PROGRAM], PNCVertex::VERTEX_ATTRIBUTE_POSITION, "inPosition");
	glBindAttribLocation(mPNCProgramIDs[INDEX_PROGRAM], PNCVertex::VERTEX_ATTRIBUTE_NORMAL, "inNormal");
	glBindAttribLocation(mPNCProgramIDs[INDEX_PROGRAM], PNCVertex::VERTEX_ATTRIBUTE_COLOR, "inColor");

	// link & use the program
	glLinkProgram(mPNCProgramIDs[INDEX_PROGRAM]);
	checkProgramAndShaders();
	glUseProgram(mPNCProgramIDs[INDEX_PROGRAM]);
}

MeshRenderer::~MeshRenderer()
{
	clear();

	// free shaders etc.
	glDeleteShader(mPNCProgramIDs[INDEX_FRAGMENT_SHADER]);
	glDeleteShader(mPNCProgramIDs[INDEX_VERTEX_SHADER]);
	glDeleteProgram(mPNCProgramIDs[INDEX_PROGRAM]);
}

void MeshRenderer::clear()
{
	// free each mesh
	while (!mMeshes.empty())
		deleteUploadedMesh(0);

	// free access data structures for meshes on the GPU
	mMeshes.clear();
	mMeshes.shrink_to_fit();
}

void MeshRenderer::deleteUploadedMesh(const Mesh &mesh)
{
	// find the mesh & free it
	const uint32 meshCount = (uint32) mMeshes.size();
	for (uint32 meshIdx = 0; meshIdx < meshCount; ++meshIdx)
	{
		if (mMeshes[meshIdx].mMesh != &mesh)
			continue;
		
		deleteUploadedMesh(meshIdx);
		return;
	}

	throw Exception("Mesh on the GPU to be deleted was not uploaded to the GPU or freed already.");
}

void MeshRenderer::deleteUploadedMesh(const uint32 targetMeshIdx)
{
	// free the mesh on the GPU
	MeshOnGPU &meshOnGPU = mMeshes[targetMeshIdx];

	glDeleteVertexArrays(1, meshOnGPU.mIDs + MeshOnGPU::INDEX_VERTEX_ARRAY_OBJECT);
	glDeleteBuffers(2, meshOnGPU.mIDs + MeshOnGPU::INDEX_VERTEX_BUFFER);

	mMeshes[targetMeshIdx] = mMeshes.back();
	mMeshes.pop_back();
}

void MeshRenderer::renderUploadedMeshes() const
{
	// render each mesh
	const uint32 meshCount = (uint32) mMeshes.size();
	for (uint32 meshIdx = 0; meshIdx < meshCount; ++meshIdx)
	{
		// draw mesh using the uploaded vertex & index buffers
		const MeshOnGPU &mesh = mMeshes[meshIdx];
		const uint32 indexCount = mesh.mMesh->getIndexCount();
		
		// define what data is drawn
		glBindVertexArray(mesh.mIDs[MeshOnGPU::INDEX_VERTEX_ARRAY_OBJECT]);
		glDrawElements(GL_TRIANGLES, indexCount, GL_UNSIGNED_INT, 0);
		//glBindBuffer(GL_ARRAY_BUFFER, mesh.mIDs[MeshOnGPU::INDEX_VERTEX_BUFFER]);
		//glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mesh.mIDs[MeshOnGPU::INDEX_INDEX_BUFFER]);
	}
}

void MeshRenderer::uploadData(const Mesh &mesh)
{
	// meta data for uploading
	const uint32 vertexCount = mesh.getVertexCount();
	const uint32 indexCount = mesh.getIndexCount();
	const uint64 vertexBytes = vertexCount * sizeof(PNCVertex);
	const uint64 indexBytes = indexCount * sizeof(uint32);
	const uint32 baseOffset = 0;
	const GLbitfield flags = 0;

	// MeshOnGPU = access point to data on the GPU
	MeshOnGPU meshOnGPU;
	meshOnGPU.mMesh = &mesh;

	// create data for vertex buffer in a format OpenGL can use
	PNCVertex *vertexBuffer = createVertexBuffer(mesh);

	// create and bind vertex buffer object
	glCreateVertexArrays(1, meshOnGPU.mIDs + MeshOnGPU::INDEX_VERTEX_ARRAY_OBJECT);
	glBindVertexArray(meshOnGPU.mIDs[MeshOnGPU::INDEX_VERTEX_ARRAY_OBJECT]);
	defineVertexFormat();

	// create, bind and fill vertex & index buffer
	glGenBuffers(2, meshOnGPU.mIDs + MeshOnGPU::INDEX_VERTEX_BUFFER);

	// create & upload vertex bufferGLuint baseOffset = 0;
	glBindVertexBuffer(VERTEX_BUFFER_BINDING_INDEX, meshOnGPU.mIDs[MeshOnGPU::INDEX_VERTEX_BUFFER], baseOffset, sizeof(PNCVertex));
	//glBindBuffer(GL_ARRAY_BUFFER, meshOnGPU.mBufferIndices[0]);
	glBufferStorage(GL_ARRAY_BUFFER, vertexBytes, vertexBuffer, flags);

	// create & upload index buffer
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, meshOnGPU.mIDs[MeshOnGPU::INDEX_INDEX_BUFFER]);
	glBufferStorage(GL_ELEMENT_ARRAY_BUFFER, indexBytes, mesh.getIndices(), flags);

	// free resources
	delete [] vertexBuffer;
	vertexBuffer = NULL;
}

void MeshRenderer::defineVertexFormat()
{
	// define vertex format according to PNCVertex vertices
	// attribute 0: 3 x float for positions
	glEnableVertexAttribArray(PNCVertex::VERTEX_ATTRIBUTE_POSITION);
	glVertexAttribFormat(PNCVertex::VERTEX_ATTRIBUTE_POSITION, 3, GL_FLOAT, GL_FALSE, offsetof(PNCVertex, mPosition));
	glVertexAttribBinding(PNCVertex::VERTEX_ATTRIBUTE_POSITION, VERTEX_BUFFER_BINDING_INDEX);
	
	// attribute 1: 3 x float for normals
	glEnableVertexAttribArray(PNCVertex::VERTEX_ATTRIBUTE_NORMAL);
	glVertexAttribFormat(PNCVertex::VERTEX_ATTRIBUTE_NORMAL, 3, GL_FLOAT, GL_FALSE, offsetof(PNCVertex, mNormal));
	glVertexAttribBinding(PNCVertex::VERTEX_ATTRIBUTE_NORMAL, VERTEX_BUFFER_BINDING_INDEX);

	// attribute 2: 3 x float for colors
	glEnableVertexAttribArray(PNCVertex::VERTEX_ATTRIBUTE_COLOR);
	glVertexAttribFormat(PNCVertex::VERTEX_ATTRIBUTE_COLOR, 3, GL_FLOAT, GL_FALSE, offsetof(PNCVertex, mColor));
	glVertexAttribBinding(PNCVertex::VERTEX_ATTRIBUTE_COLOR, VERTEX_BUFFER_BINDING_INDEX);
}

void MeshRenderer::checkProgramAndShaders() const
{
	// check shaders
	for (IndexType indexType = INDEX_VERTEX_SHADER; indexType < INDEX_TYPE_COUNT; indexType = (IndexType) (indexType + 1))
		checkShader(mPNCProgramIDs[indexType]);

	// check program
	checkProgram();
}

void MeshRenderer::checkProgram() const
{
	// check program
	glValidateProgram(mPNCProgramIDs[INDEX_PROGRAM]);

	// valid program?
	GLint valid = GL_FALSE;
	glGetProgramiv(mPNCProgramIDs[INDEX_PROGRAM], GL_VALIDATE_STATUS, &valid);
	if (GL_TRUE == valid)
		return;

	// get link status & log entry on failure
	GLint linked = GL_FALSE;
	glGetProgramiv(mPNCProgramIDs[INDEX_PROGRAM], GL_LINK_STATUS, &linked);
	if (GL_TRUE != linked)
	{
		// todo log this properly
		cerr << "Program could not be linked!" << endl;
	}

	// get program log length
	GLint infoLogLength = 0;
	glGetProgramiv(mPNCProgramIDs[INDEX_PROGRAM], GL_INFO_LOG_LENGTH, &infoLogLength);

	// get program log
	GLsizei actualLength;
	char *log = new char[infoLogLength];
	glGetProgramInfoLog(mPNCProgramIDs[INDEX_PROGRAM], infoLogLength, &actualLength, log);

	// output log
	// todo log this properly
	cerr << "glGetProgramInfoLog:\n";
	cerr << log << endl;
	GraphicsException exception(log, mPNCProgramIDs[INDEX_PROGRAM]);

	delete [] log;
	log = NULL;

	throw exception;
}

void MeshRenderer::checkShader(const uint32 shaderID)
{
	// compiled?
	GLint compiled = GL_FALSE;
	glGetShaderiv(shaderID, GL_COMPILE_STATUS, &compiled);
	if (GL_TRUE == compiled)
		return;

	// get shader log length
	GLint infoLogLength = 0;
	glGetShaderiv(shaderID, GL_INFO_LOG_LENGTH, &infoLogLength);

	// get shader log
	GLsizei actualLength;
	char *log = new char[infoLogLength];
	glGetShaderInfoLog(shaderID, infoLogLength, &actualLength, log);

	// output log
	cerr << "glGetShaderInfoLog:\n";
	cerr << log << endl;
	GraphicsException exception(log, shaderID);

	delete [] log;
	log = NULL;

	throw exception;
}

PNCVertex *MeshRenderer::createVertexBuffer(const Mesh &mesh)
{
	// allocate memory
	const uint32 vertexCount = mesh.getVertexCount();
	PNCVertex *vertexBuffer = new PNCVertex[vertexCount];

	// fill the vertex buffer in a format proper for OpenGL (float, interleaved vertex attributes)
	for (uint32 vertexIdx = 0; vertexIdx < vertexCount; ++vertexIdx)
	{
		// get data
		const Vector3 &sourcePos = mesh.getPosition(vertexIdx);
		const Vector3 &sourceNormal = mesh.getNormal(vertexIdx);
		const Vector3 &sourceColor = mesh.getColor(vertexIdx);

		// fill vertex for OpenGL
		PNCVertex &v = vertexBuffer[vertexIdx];

		// copy position
		v.mPosition[0] = static_cast<float>(sourcePos.x);
		v.mPosition[1] = static_cast<float>(sourcePos.y);
		v.mPosition[2] = static_cast<float>(sourcePos.z);

		// copy normal
		v.mNormal[0] = static_cast<float>(sourceNormal.x);
		v.mNormal[1] = static_cast<float>(sourceNormal.y);
		v.mNormal[2] = static_cast<float>(sourceNormal.z);

		// copy color
		v.mColor[0] = static_cast<float>(sourceColor.x);
		v.mColor[1] = static_cast<float>(sourceColor.y);
		v.mColor[2] = static_cast<float>(sourceColor.z);
	}

	return vertexBuffer;
}