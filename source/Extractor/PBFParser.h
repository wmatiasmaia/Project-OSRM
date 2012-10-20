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

#ifndef PBFPARSER_H_
#define PBFPARSER_H_

#include <fstream>
#include <zlib.h>
#include <boost/make_shared.hpp>
#include <boost/ref.hpp>
#include <boost/shared_ptr.hpp>

#include <osmpbf/fileformat.pb.h>
#include <osmpbf/osmformat.pb.h>

#include "../typedefs.h"

#include "BaseParser.h"
#include "ExtractorStructs.h"
#include "../DataStructures/HashTable.h"
#include "../DataStructures/ConcurrentQueue.h"

class Extractor;

class PBFParser : public BaseParser<_Node, _RawRestrictionContainer, _Way> {

    typedef BaseParser<_Node, _RawRestrictionContainer, _Way> super;

    enum EntityType {
        TypeNode = 1,
        TypeWay = 2,
        TypeRelation = 4,
        TypeDenseNode = 8
    } ;

    enum Endianness {
        LittleEndian = 1,
        BigEndian = 2
    };

    struct _ThreadData {
        int currentGroupID;
        int currentEntityID;
        short entityTypeIndicator;

        OSMPBF::BlobHeader PBFBlobHeader;
        OSMPBF::Blob PBFBlob;

        OSMPBF::HeaderBlock PBFHeaderBlock;
        OSMPBF::PrimitiveBlock PBFprimitiveBlock;

        std::vector<char> charBuffer;
    };

    static const int NANO = 1000 * 1000 * 1000;
    static const int MAX_BLOB_HEADER_SIZE = 64 * 1024;
    static const int MAX_BLOB_SIZE = 32 * 1024 * 1024;


public:
    PBFParser(Extractor* extractor, const char * fileName);
    ~PBFParser();

    bool Init();
    void ReadData();
    void ParseData();
    bool Parse();

private:
    void parseDenseNode(_ThreadData * threadData);
    void parseNode(_ThreadData * );
    void parseRelation(_ThreadData * threadData);
    void parseWay(_ThreadData * threadData);
    void loadGroup(_ThreadData * threadData);
    void loadBlock(_ThreadData * threadData);

    bool readPBFBlobHeader(std::fstream& stream, _ThreadData * threadData);
    bool unpackZLIB(std::fstream &, _ThreadData * threadData);
    bool unpackLZMA(std::fstream &, _ThreadData * );
    bool readBlob(std::fstream& stream, _ThreadData * threadData);
    bool readNextBlock(std::fstream& stream, _ThreadData * threadData);
    
    Endianness getMachineEndianness() const;
    
    // Reverses Network Byte Order into something usable
    inline unsigned swapEndian(unsigned x) {
        if(getMachineEndianness() == LittleEndian)
            return ( (x>>24) | ((x<<8) & 0x00FF0000) | ((x>>8) & 0x0000FF00) | (x<<24) );
        return x;
    }
    
    // counting the number of read blocks and groups
    unsigned groupCount;
    unsigned blockCount;

    
    // the input stream to parse
    std::fstream input;

    // ThreadData Queue
    boost::shared_ptr<ConcurrentQueue < _ThreadData* > > threadDataQueue;
};

#endif /* PBFPARSER_H_ */
