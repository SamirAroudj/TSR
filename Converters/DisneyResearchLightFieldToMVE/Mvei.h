/*
 * Copyright (C) 2017 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */
#ifndef _MVEI_H_
#define _MVEI_H_

#include <map>
#include <filesystem>

#include "Platform/Storage/Directory.h"
#include "Platform/Storage/File.h"
#include "Platform/Utilities/HelperFunctions.h"

using namespace Platform;
using namespace std;
using namespace Storage;

/* The signature to identify MVEI image files and loader limits. */
#define MVEI_FILE_SIGNATURE "\211MVE_IMAGE\n"
#define MVEI_FILE_SIGNATURE_LEN 11
#define MVEI_MAX_PIXEL_AMOUNT (16384 * 16384) /* 2^28 */



/* This header contains required classes with only basic functionality from the MVE library 
 * Only necessary for development on Windows
 */

enum ImageType
{
	IMAGE_TYPE_UNKNOWN,
	/* Unsigned integer types. */
	IMAGE_TYPE_UINT8, // uint8_t, unsigned char
	IMAGE_TYPE_UINT16, // uint16_t
	IMAGE_TYPE_UINT32, // uint32_t, unsigned int
	IMAGE_TYPE_UINT64, // uint64_t
	/* Signed integer types. */
	IMAGE_TYPE_SINT8, // int8_t, char, signed char
	IMAGE_TYPE_SINT16, // int16_t
	IMAGE_TYPE_SINT32, // int32_t, int
	IMAGE_TYPE_SINT64, // int64_t
	/* Floating point types. */
	IMAGE_TYPE_FLOAT, // float
	IMAGE_TYPE_DOUBLE // double
};

/**
* View meta information that stores key/value pairs and the camera.
* The key is the INI section name, a dot ".", and the key name.
* The value is an arbitrary string (newlines are disallowed).
*/
struct MetaData
{
	typedef map<std::string, std::string> KeyValueMap;

	KeyValueMap data;
	bool is_dirty = false;
};


/**
* Base class for images without type information.
* This class basically provides width, height and channel
* information and a framework for type information and data access.
*/
class ImageBase
{
public:
	typedef std::shared_ptr<ImageBase> Ptr;
	typedef std::shared_ptr<ImageBase const> ConstPtr;

public:
	/** Initializes members with 0. */
	ImageBase(void);
	virtual ~ImageBase(void);

	/** Returns the width of the image. */
	int width(void) const;
	/** Returns the height of the image. */
	int height(void) const;
	/** Returns the amount of channels in the image. */
	int channels(void) const;

	/** Returns false if one of width, height or channels is 0. */
	bool valid(void) const;

protected:
	int w, h, c;
};

/**
* Base class for images of arbitrary type. Image values are stored
* in a standard STL Vector. Type information is provided. This class
* makes no assumptions about the image structure, i.e. it provides no
* pixel access methods.
*/
template <typename T>
class TypedImageBase : public ImageBase
{
public:
	typedef T ValueType;
	typedef std::shared_ptr<TypedImageBase<T> > Ptr;
	typedef std::shared_ptr<TypedImageBase<T> const> ConstPtr;
	typedef std::vector<T> ImageData;

public:
	/** Default constructor creates an empty image. */
	TypedImageBase(void);

	/** Copy constructor duplicates another image. */
	TypedImageBase(TypedImageBase<T> const& other);

	virtual ~TypedImageBase(void);

	/** Allocates new image space, clearing previous content. */
	void allocate(int width, int height, int chans);

	/**
	* Resizes the underlying image data vector.
	* Note: This leaves the existing/remaining image data unchanged.
	* Warning: If the image is shrunk, the data vector is resized but
	* may still consume the original amount of memory. Use allocate()
	* instead if the previous data is not important.
	*/
	void resize(int width, int height, int chans);

	/** Fills the data with a constant value. */
	void fill(T const& value);

	/** Swaps the contents of the images. */
	void swap(TypedImageBase<T>& other);

	/** Returns the data vector for the image. */
	ImageData const& get_data(void) const;
	/** Returns the data vector for the image. */
	ImageData& get_data(void);

	/** Returns the data pointer. */
	T const* get_data_pointer(void) const;
	/** Returns the data pointer. */
	T* get_data_pointer(void);

	/** Returns the size of the image in bytes (w * h * c * BPV). */
	std::size_t get_byte_size(void) const;
	/** Returns the char pointer to the data. */
	char const* get_byte_pointer(void) const;
	/** Returns the char pointer to the data. */
	char* get_byte_pointer(void);

protected:
	ImageData data;
};

/* ================================================================ */

inline
ImageBase::ImageBase(void)
: w(0), h(0), c(0)
{
}

inline
ImageBase::~ImageBase(void)
{
}

inline int
ImageBase::width(void) const
{
	return this->w;
}

inline int
ImageBase::height(void) const
{
	return this->h;
}

inline int
ImageBase::channels(void) const
{
	return this->c;
}

inline bool
ImageBase::valid(void) const
{
	return this->w && this->h && this->c;
}

template <typename T>
inline
TypedImageBase<T>::TypedImageBase(void)
{
}

template <typename T>
inline
TypedImageBase<T>::~TypedImageBase(void)
{
}

template <typename T>
inline void
TypedImageBase<T>::allocate(int width, int height, int chans)
{
	this->clear();
	this->resize(width, height, chans);
}

template <typename T>
inline void
TypedImageBase<T>::resize(int width, int height, int chans)
{
	this->w = width;
	this->h = height;
	this->c = chans;
	this->data.resize(width * height * chans);
}

template <typename T>
inline void
TypedImageBase<T>::fill(T const& value)
{
	std::fill(this->data.begin(), this->data.end(), value);
}

template <typename T>
inline typename TypedImageBase<T>::ImageData&
TypedImageBase<T>::get_data(void)
{
	return this->data;
}

template <typename T>
inline typename TypedImageBase<T>::ImageData const&
TypedImageBase<T>::get_data(void) const
{
	return this->data;
}

template <typename T>
inline T const*
TypedImageBase<T>::get_data_pointer(void) const
{
	if (this->data.empty())
		return nullptr;
	return &this->data[0];
}

template <typename T>
inline T*
TypedImageBase<T>::get_data_pointer(void)
{
	if (this->data.empty())
		return nullptr;
	return &this->data[0];
}

template <typename T>
inline std::size_t
TypedImageBase<T>::get_byte_size(void) const
{
	return this->data.size() * sizeof(T);
}

template <typename T>
inline char const*
TypedImageBase<T>::get_byte_pointer(void) const
{
	return reinterpret_cast<char const*>(this->get_data_pointer());
}

template <typename T>
inline char*
TypedImageBase<T>::get_byte_pointer(void)
{
	return reinterpret_cast<char*>(this->get_data_pointer());
}

// slightly modified functions from mve, for simplicity
template <typename T>
void
save_mvei_file(TypedImageBase<T>* image, std::string const& filename, int32_t type)
{
	if (image == nullptr)
		throw std::invalid_argument("Null image given");

	int32_t width = image->width();
	int32_t height = image->height();
	int32_t channels = image->channels();

	char const* data = image->get_byte_pointer();
	std::size_t size = image->get_byte_size();

	std::ofstream out(filename.c_str(), std::ios::binary);
	if (!out.good())
		cerr << "Could not create target MVEI file: " << std::strerror(errno) << endl;

	out.write(MVEI_FILE_SIGNATURE, MVEI_FILE_SIGNATURE_LEN);
	out.write(reinterpret_cast<char const*>(&width), sizeof(int32_t));
	out.write(reinterpret_cast<char const*>(&height), sizeof(int32_t));
	out.write(reinterpret_cast<char const*>(&channels), sizeof(int32_t));
	out.write(reinterpret_cast<char const*>(&type), sizeof(int32_t));
	out.write(data, size);

	if (!out.good())
		cerr << "Could not create target MVEI file: " << std::strerror(errno) << endl;
}

void writeMetaData(MetaData metaData, Path dir)
{
	Path dst = Path::appendChild(dir, Path("meta.ini"));
	File f(dst, File::FileMode::CREATE_WRITING, false);
	Encoding enc = Encoding::ENCODING_ASCII; //TODO change dep on platform?

	fprintf(&f.getHandle(), "%s", "# MVE view meta data is stored in INI-file syntax.\n");
	fprintf(&f.getHandle(), "%s", "# This file is generated, formatting will get lost.\n");

	string lastSection;
	map<string, string>::const_iterator iter;
	for (iter = metaData.data.begin(); iter != metaData.data.end(); iter++)
	{
		string key = iter->first;
		string value = iter->second;

		size_t sectionPos = key.find_first_of('.');
		string section = key.substr(0, sectionPos);

		key = key.substr(sectionPos + 1);

		if (section != lastSection)
		{
			//f.writeString("\n[" + section + "]\n", enc);
			fprintf(&f.getHandle(), "%s", ("\n[" + section + "]\n").data());
			lastSection = section;
		}
		fprintf(&f.getHandle(), "%s", (key + " = " + value + "\n").data());
		//f.writeString(key + " = " + value + "\n", enc);
	}
}

#endif // _MVEI_H_