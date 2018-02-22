/*
* Copyright (C) 2018 by Author: Aroudj, Samir
* TU Darmstadt - Graphics, Capture and Massively Parallel Computing
* All rights reserved.
*
* This software may be modified and distributed under the terms
* of the BSD 3-Clause license. See the License.txt file for details.
*/

#include "SurfaceReconstruction/Image/MVEIHeader.h"
#include "Platform/FailureHandling/FileException.h"
#include "SurfaceReconstruction/Image/ViewsImage.h"

using namespace FailureHandling;
using namespace std;
using namespace SurfaceReconstruction;
using namespace Utilities;

ViewsImage *ViewsImage::request(const std::string &resourceName, const Storage::Path &imageFileName)
{	
	// exists?
	Image *image = Image::request(resourceName);
	if (image)
	{
		ViewsImage *viewsImage = dynamic_cast<ViewsImage *>(image);
		if (viewsImage)
			return viewsImage;
		throw FileException("An image of another type than ViewsImage but with the same resource name already exists!", imageFileName);
	}

	// load it from file
	MVEIHeader header;
	uint32 *viewIDs = reinterpret_cast<uint32 *>(Image::loadMVEI(header, imageFileName, true));
	if (!viewIDs)
		return NULL;
	return new ViewsImage(viewIDs, header.mSize, header.mChannelCount, resourceName);
}

ViewsImage::ViewsImage(uint32 *&viewIDs, const ImgSize &size, const uint32 &channelCount, const string &resourceName) :
	Image(size, channelCount, resourceName), mViewIDs(viewIDs)
{
	viewIDs = NULL;
}

ViewsImage::~ViewsImage()
{
	clear();
}

void ViewsImage::clear()
{
	delete [] mViewIDs;
	mViewIDs = NULL;
}