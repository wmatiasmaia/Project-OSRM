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

  Created on: 09.02.2013
  Author: emil
*/

#ifndef LUAROUTEITERATOR_H_
#define LUAROUTEITERATOR_H_

class LuaRouteIterator {
public:
    LuaRouteIterator(ExtractionWay& w, WayToRouteMap& wayToRouteMap, RouteMap& _routeMap) : 
        iters( wayToRouteMap.equal_range(w.id) ), routeMap(_routeMap) {}

    inline void Next(lua_State *L) {
        if( iters.first != iters.second ) {
            std::string role = iters.first->second.first;
            int64_t routeId = iters.first->second.second;
            ++iters.first;
            
            RouteMap::iterator routeIter = routeMap.find( routeId );
            if( routeIter != routeMap.end()) {
                //push two return values to lua
                lua_pushstring( L, role.c_str() ); 
                luabind::object o = luabind::object(L,routeIter->second);
        		o.push(L);
            }
        }
	}

private:    
    std::pair<WayToRouteMap::iterator, WayToRouteMap::iterator>   iters;
    RouteMap& routeMap;
};

#endif /* LUAROUTEITERATOR_H_ */
