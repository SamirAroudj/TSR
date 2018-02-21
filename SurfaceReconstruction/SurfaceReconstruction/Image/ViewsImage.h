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

#include <cassert>
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
		inline uint32 get(const uint32 &pixelIdx, const uint32 &channelIdx) const;

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

	inline uint32 ViewsImage::get(const uint32 &pixelIdx, const uint32 &channelIdx) const
	{
		assert(channelIdx < mChannelCount);
		return mViewIDs[mChannelCount * pixelIdx + channelIdx];
	}

}

#endif // _VIEWS_IMAGE_
