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

#include "Extractor.h"
#include "../Util/InputFileUtil.h"
#include "../Util/MachineInfo.h"
#include "LuaUtil.h"
#include "PBFParser.h"
#include "XMLParser.h"
#include "ExtractionHelperFunctions.h"
#include "../Util/fileExtensions.h"
#include <cfloat>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/regex.hpp>
#include <boost/regex.hpp>
extern "C" {
#include <lua.h>
#include <lauxlib.h>
#include <lualib.h>
}

Extractor::Extractor(boost::filesystem::path& file, boost::filesystem::path& profile) : 
mFileName(file), mProfileName(profile) {
    mStringMap[""] = 0;

	std::string extension = FileExtensions::extension(file);
	if( extension == ".osm.pbf" || extension == ".osm.bz2" )
        mParser = new PBFParser(this,file.c_str());
	else
        mParser = new XMLParser(this,file.c_str());
};

Extractor::~Extractor() {
	delete mParser;
    mStringMap.clear();
}

lua_State* Extractor::getLuaState() {
	return mLuaState;
}

void Extractor::extract() {
    INFO("extracting data from input file " << mFileName);

	checkRAM();
	setupLua();

	mParser->Init();
    mParser->Parse();

    unsigned amountOfRAM = 1;
	boost::filesystem::path	outFile = FileExtensions::change_extension(mFileName, ".osrm");
	boost::filesystem::path	outRestrictions = FileExtensions::change_extension(mFileName, ".osrm.restrictions");
    
	mExternalMemory.PrepareData(outFile.c_str(), outRestrictions.c_str(), amountOfRAM);

	INFO("[extractor] finished.");
    std::cout << "\nTo preprocess the extracted data, run this command:\n./osrm-prepare " << outFile.c_str() << " " << outRestrictions.c_str() << std::endl;
}

void Extractor::checkRAM() {	
    unsigned installedRAM = GetPhysicalmemory(); 
    if(installedRAM < 2048264) {
        WARN("Machine has less than 2GB RAM.");
    }
/*    if(testDataFile("extractor.ini")) {
        ExtractorConfiguration extractorConfig("extractor.ini");
        unsigned memoryAmountFromFile = atoi(extractorConfig.GetParameter("Memory").c_str());
        if( memoryAmountFromFile != 0 && memoryAmountFromFile <= installedRAM/(1024*1024))
            amountOfRAM = memoryAmountFromFile;
        INFO("Using " << amountOfRAM << " GB of RAM for buffers");
    }
	*/
}

void Extractor::setupLua() {
    // Create a new lua state
    mLuaState = luaL_newstate();

    // Connect LuaBind to this lua state
    luabind::open(mLuaState);

    // Add our function to the state's global scope
    luabind::module(mLuaState) [
      luabind::def("print", LUA_print<std::string>),
      luabind::def("parseMaxspeed", parseMaxspeed),
      luabind::def("durationIsValid", durationIsValid),
      luabind::def("parseDuration", parseDuration)
    ];

    if( luaL_dostring( mLuaState, "print('Initializing LUA engine')\n" )!=0 ) {
        ERR(lua_tostring(mLuaState,-1)<< " occured in scripting block");
    }

    luabind::module(mLuaState) [
      luabind::class_<HashTable<std::string, std::string> >("keyVals")
      .def("Add", &HashTable<std::string, std::string>::Add)
      .def("Find", &HashTable<std::string, std::string>::Find)
    ];

    luabind::module(mLuaState) [
      luabind::class_<ImportNode>("Node")
          .def(luabind::constructor<>())
          .def_readwrite("lat", &ImportNode::lat)
          .def_readwrite("lon", &ImportNode::lon)
          .def_readwrite("id", &ImportNode::id)
          .def_readwrite("bollard", &ImportNode::bollard)
          .def_readwrite("traffic_light", &ImportNode::trafficLight)
          .def_readwrite("tags", &ImportNode::keyVals)
    ];

    luabind::module(mLuaState) [
      luabind::class_<_Way>("Way")
          .def(luabind::constructor<>())
          .def_readwrite("name", &_Way::name)
          .def_readwrite("speed", &_Way::speed)
          .def_readwrite("type", &_Way::type)
          .def_readwrite("access", &_Way::access)
          .def_readwrite("roundabout", &_Way::roundabout)
          .def_readwrite("is_duration_set", &_Way::isDurationSet)
          .def_readwrite("is_access_restricted", &_Way::isAccessRestricted)
          .def_readwrite("ignore_in_grid", &_Way::ignoreInGrid)
          .def_readwrite("tags", &_Way::keyVals)
          .def_readwrite("direction", &_Way::direction)
          .enum_("constants")
          [
           luabind::value("notSure", 0),
           luabind::value("oneway", 1),
           luabind::value("bidirectional", 2),
           luabind::value("opposite", 3)
          ]
    ];
    // Now call our function in a lua script
	INFO("Parsing speedprofile from " << mProfileName );
    if(0 != luaL_dofile(mLuaState, mProfileName.c_str() )) {
        ERR(lua_tostring(mLuaState,-1)<< " occured in scripting block");
    }

    //open utility libraries string library;
    luaL_openlibs(mLuaState);	
}

/** warning: caller needs to take care of synchronization! */
bool Extractor::parseNode(_Node n) {
    if(n.lat <= 85*100000 && n.lat >= -85*100000)
        mExternalMemory.allNodes.push_back(n);
    return true;
}

bool Extractor::parseRestriction(_RawRestrictionContainer r) {
    mExternalMemory.restrictionsVector.push_back(r);
    return true;
}

/** warning: caller needs to take care of synchronization! */
bool Extractor::parseWay(_Way w) {
    /*** Store name of way and split it into edge segments ***/

    if ( w.speed > 0 ) { //Only true if the way is specified by the speed profile

        //Get the unique identifier for the street name
        const StringMap::const_iterator strit = mStringMap.find(w.name);
        if(strit == mStringMap.end()) {
            w.nameID = mExternalMemory.nameVector.size();
            mExternalMemory.nameVector.push_back(w.name);
            mStringMap.insert(StringMap::value_type(w.name, w.nameID));
        } else {
            w.nameID = strit->second;
        }

        if(fabs(-1. - w.speed) < FLT_EPSILON){
            WARN("found way with bogus speed, id: " << w.id);
            return true;
        }
        if(w.id == UINT_MAX) {
            WARN("found way with unknown type: " << w.id);
            return true;
        }

        if ( w.direction == _Way::opposite ){
            std::reverse( w.path.begin(), w.path.end() );
        }

        for(vector< NodeID >::size_type n = 0; n < w.path.size()-1; ++n) {
            mExternalMemory.allEdges.push_back(_Edge(w.path[n], w.path[n+1], w.type, w.direction, w.speed, w.nameID, w.roundabout, w.ignoreInGrid, w.isDurationSet, w.isAccessRestricted));
            mExternalMemory.usedNodeIDs.push_back(w.path[n]);
        }
        mExternalMemory.usedNodeIDs.push_back(w.path.back());

        //The following information is needed to identify start and end segments of restrictions
        mExternalMemory.wayStartEndVector.push_back(_WayIDStartAndEndEdge(w.id, w.path[0], w.path[1], w.path[w.path.size()-2], w.path[w.path.size()-1]));
    }
    return true;
}