/*
 * intersection.h
 *
 *  Created on: Sep 22, 2014
 *      Author: petr
 */

#pragma once

#include "state.h"

namespace opendubins {

    struct StateAtDistance {

        State state;
        double distance;

        StateAtDistance() {
            state = State(Point(NAN, NAN), NAN);
            distance = NAN;
        }

        StateAtDistance(State state, double distance) {
            this->state = state;
            this->distance = distance;
        }

    };

}
