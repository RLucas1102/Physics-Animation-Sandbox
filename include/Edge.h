#ifndef EDGE_H
#define EDGE_H

class SoftEdgeData {
    public:

        SoftEdgeData(const size_t i, const size_t j, const double edgeLength) :
            inode (i),
            jnode (j),
            length (edgeLength)
            {}

        ~SoftEdgeData(){}

        const size_t& GetFirstNode() const { return inode; }

        const size_t& GetSecondNode() const { return jnode; }

        const double GetEdgeLength() const { return length; }

        private:

            size_t inode, jnode;    // These are two particle indices
            double length;          // Lab

    };


#endif