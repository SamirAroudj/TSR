/*
 * Copyright (C) 2017 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 *
 * This header contains required classes with only basic functionality from the MVE library
 * Enables development on Windows
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

/**
* View meta information that stores key/value pairs and the camera.
* The key is the INI section name, a dot ".", and the key name.
* The value is an arbitrary string (newlines are disallowed).
*/
struct MetaData
{
	typedef map<std::string, std::string> KeyValueMap;

	KeyValueMap data;
};

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
			fprintf(&f.getHandle(), "%s", ("\n[" + section + "]\n").data());
			lastSection = section;
		}
		fprintf(&f.getHandle(), "%s", (key + " = " + value + "\n").data());
	}
}

#endif // _MVEI_H_