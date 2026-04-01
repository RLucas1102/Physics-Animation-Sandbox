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

    void DynamicalStateData::create_attr(const std::string &nam, const int &def) {
        DSAttribute<int> newAttrib = DSAttribute<int>(nam, def);
        int_attributes.insert({nam, newAttrib});
    }

    void DynamicalStateData::create_attr(const std::string &nam, const float &def) {
        DSAttribute<float> newAttrib = DSAttribute<float>(nam, def);
        float_attributes.insert({nam, newAttrib});
    }

    void DynamicalStateData::create_attr(const std::string &nam, const Vector &def) {
        DSAttribute<Vector> newAttrib = DSAttribute<Vector>(nam, def);
        vector_attributes.insert({nam, newAttrib});
    }

    void DynamicalStateData::create_attr(const std::string &nam, const Color &def) {
        DSAttribute<Color> newAttrib = DSAttribute<Color>(nam, def);
        color_attributes.insert({nam, newAttrib});
    }

    DynamicalState CreateDynamicalState(const std::string& nam) {
        DynamicalState outState = DynamicalState(new DynamicalStateData(nam));
        std::cout << "Created DynamicalState: " << nam << std::endl;
        return outState;
    }

}