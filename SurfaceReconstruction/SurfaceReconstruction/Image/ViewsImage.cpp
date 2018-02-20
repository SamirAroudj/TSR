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

	return new ViewsImage(resourceName, imageFileName);
}

ViewsImage::ViewsImage(const std::string &resourceName, const Storage::Path &imageFileName) :
	Image(ImgSize(0,0), 0, resourceName), mViewIDs(NULL)
{
	MVEIHeader header;
	mViewIDs = (uint32 *) Image::loadMVEI(header, imageFileName, true);
	mSize = header.mSize;
	mChannelCount = header.mChannelCount;
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