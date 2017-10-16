/*
 * Copyright (C) 2017 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */
#ifndef _I_RECONSTRUCTOR_OBSERVER_H_
#define _I_RECONSTRUCTOR_OBSERVER_H_

// todo comments

#include <string>
#include "Platform/DataTypes.h"
	
namespace SurfaceReconstruction
{
	class FlexibleMesh;
	
	class IReconstructorObserver
	{
	public:
		enum ReconstructionType
		{
			RECONSTRUCTION_VIA_OCCUPANCIES,			/// Reconstruction from free space and surface constraints (occupanccy scalar field of Occupancy object)
			RECONSTRUCTION_VIA_PHOTOS,				/// Reconstruction which was improved by means of photos.
			RECONSTRUCTION_VIA_SAMPLES,				/// Reconstruction from sample-based refinement of mCrust.
			RECONSTRUCTION_TYPE_COUNT				/// Number of different reconstruction types.
		};

	public:
		/** todo
		@param todo
		@param responsible If this is set to true then the observer is asked for responsibility for memory management.
			If responsibility is taken (deletion of the mesh and its resources) then true should be returned.
			The observer does not need to take over responsibility. (If no object takes responsibility the mesh is deleted directly.)
		@return Return true if this observer takes over the mesh and owns it and thus is responsible for its deletion.
			This is only valid if the model asks for taking responsibility by setting the parameter responsible to true. */
		virtual bool onNewReconstruction(FlexibleMesh *reconstruction,
			const uint32 iteration, const std::string &text, const ReconstructionType type, const bool responsible) = 0;

	public:
		static const char *RECONSTRUCTION_TYPE_TEXTS[RECONSTRUCTION_TYPE_COUNT];	/// Texts for ReconstructionType enumeration.
	};
}

#endif // _I_RECONSTRUCTOR_OBSERVER_H_