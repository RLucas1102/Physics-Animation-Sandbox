 //-------------------------------------------------------
 //
 //  Edge.h
 //
 //  Class for creating edges
 //
 //  Copyright (c) 2017 Jerry Tessendorf
 //
 //  Adapted by Lucas Robenolt
 //
 //--------------------------------------------------------

#ifndef EDGE_H
#define EDGE_H

#include <memory>

namespace pba{
    class SoftEdgeData {
        public:

            SoftEdgeData(const size_t i, const size_t j, const double edgeLength);
            
            ~SoftEdgeData(){}

            const size_t& GetFirstNode() const { return inode; }

            const size_t& GetSecondNode() const { return jnode; }

            const double GetEdgeLength() const { return length; }

            private:

                size_t inode, jnode;    // These are two particle indices
                double length;          // Lab

        };

        typedef std::shared_ptr<SoftEdgeData> SoftEdge;

        SoftEdge CreateSoftEdge(const size_t i, const size_t j, const double edgeLength);

}

#endif