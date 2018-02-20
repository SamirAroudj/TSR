/*
* Copyright (C) 2018 by Author: Aroudj, Samir
* TU Darmstadt - Graphics, Capture and Massively Parallel Computing
* All rights reserved.
*
* This software may be modified and distributed under the terms
* of the BSD 3-Clause license. See the License.txt file for details.
*/

#ifndef _VIEWS_IMAGE_H_
#define _VIEWS_IMAGE_H_

#include "SurfaceReconstruction/Image/Image.h"
#include "Platform/Utilities/Size2.h"

// todo comments

namespace SurfaceReconstruction
{

	class ViewsImage : public Image
	{
	public:
		static ViewsImage *request(const std::string &resourceName, const Storage::Path &imageFileName);

	public:
			

	protected:
		ViewsImage(const std::string &resourceName, const Storage::Path &imageFileName);
		virtual ~ViewsImage();

		virtual void clear();

	private:
		uint32 *mViewIDs;
	};
	
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///   inline function definitions   ////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

}

#endif // _VIEWS_IMAGE_
