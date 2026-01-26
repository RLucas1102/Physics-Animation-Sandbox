//-------------------------------------------------------
 //
 //  DynamicalState.h
 //
 //  Container for data associated with the dynamics
 //  degrees of freedom in a system.
 //
 //  Copyright (c) 2017 Jerry Tessendorf
 //
 //
 //--------------------------------------------------------

#include "DynamicalState.h"
#include <iostream>

namespace pba {

    DynamicalStateData::DynamicalStateData(const std::string& nam) :
        name                (nam),
        t                   (0.0),
        nb_items            (0)
        {}

    DynamicalStateData::DynamicalStateData(const DynamicalStateData &d) {}

    DynamicalStateData &DynamicalStateData::operator=(const DynamicalStateData &d)
    {
        // TODO: insert return statement here
    }

    DynamicalState CreateDynamicalState(const std::string& nam) {
        DynamicalState outState = DynamicalState(new DynamicalStateData(nam));
        std::cout << "Created DynamicalState: " << nam << std::endl;
        return outState;
    }

}