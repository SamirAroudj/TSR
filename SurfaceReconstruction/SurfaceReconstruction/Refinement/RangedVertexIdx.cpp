/*
 * Copyright (C) 2018 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */
#include "SurfaceReconstruction/Refinement/FSSFRefiner.h"
#include "SurfaceReconstruction/Refinement/RangedVertexIdx.h"
#include "SurfaceReconstruction/Scene/Scene.h"

using namespace Math;
using namespace SurfaceReconstruction;
using namespace std;


ostream &SurfaceReconstruction::operator <<(std::ostream &out, const RangedVertexIdx &rangedVertexIdx)
{
	out << "RangedVertexIdx";

	out << ", global vertex index " << rangedVertexIdx.getGlobalVertexIdx();
	out << ", local predecessor " << rangedVertexIdx.getLocalPredecessorIdx();
	out << ", costs " << rangedVertexIdx.getCosts();

	out << "\n";

	return out;
}