/*
    open source routing machine
    Copyright (C) Dennis Luxen, others 2010

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU AFFERO General Public License as published by
the Free Software Foundation; either version 3 of the License, or
any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU Affero General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
or see http://www.gnu.org/licenses/agpl.txt.
 */

#include <string>
#include <luabind/luabind.hpp>
#include "../typedefs.h"
#include "../Util/BaseConfiguration.h"
#include "ExtractionContainers.h"
#include "ExtractorStructs.h"

typedef BaseConfiguration ExtractorConfiguration;


class Extractor {
public:
	Extractor(const char* fileName, const char* profileName);
	
	void extract();
    bool parseNode(_Node n);
    bool parseRestriction(_RawRestrictionContainer r);
    bool parseWay(_Way w);
    
    lua_State* getLuaState();
    
private:
	void setupLua();
	void checkRAM();
	
	std::string mFileName;
	std::string mProfileName;
	lua_State* mLuaState;
    StringMap mStringMap;
    ExtractionContainers mExternalMemory;
};
