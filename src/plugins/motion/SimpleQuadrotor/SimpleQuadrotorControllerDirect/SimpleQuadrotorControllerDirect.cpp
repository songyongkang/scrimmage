/*!
 * @file
 *
 * @section LICENSE
 *
 * Copyright (C) 2017 by the Georgia Tech Research Institute (GTRI)
 *
 * This file is part of SCRIMMAGE.
 *
 *   SCRIMMAGE is free software: you can redistribute it and/or modify it under
 *   the terms of the GNU Lesser General Public License as published by the
 *   Free Software Foundation, either version 3 of the License, or (at your
 *   option) any later version.
 *
 *   SCRIMMAGE is distributed in the hope that it will be useful, but WITHOUT
 *   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *   FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
 *   License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with SCRIMMAGE.  If not, see <http://www.gnu.org/licenses/>.
 *
 * @author Kevin DeMarco <kevin.demarco@gtri.gatech.edu>
 * @author Eric Squires <eric.squires@gtri.gatech.edu>
 * @date 31 July 2017
 * @version 0.1.0
 * @brief Brief file description.
 * @section DESCRIPTION
 * A Long description goes here.
 *
 */

#include <scrimmage/common/Utilities.h>
#include <scrimmage/math/Angles.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/plugins/motion/SimpleQuadrotor/SimpleQuadrotorControllerDirect/SimpleQuadrotorControllerDirect.h>

#include <boost/algorithm/clamp.hpp>

REGISTER_PLUGIN(scrimmage::Controller,
                SimpleQuadrotorControllerDirect,
                SimpleQuadrotorControllerDirect_plugin)

namespace sc = scrimmage;

#include <iostream>
using std::endl;
using std::cout;

void SimpleQuadrotorControllerDirect::init(std::map<std::string, std::string> &params)
{}

bool SimpleQuadrotorControllerDirect::step(double t, double dt) {
    //double des_yaw = desired_state_->quat().yaw();

    double turn_force = 0;
    u_(0) = desired_state_->vel()(0);
    u_(1) = desired_state_->vel()(1);
    u_(2) = desired_state_->vel()(2);
    u_(3) = turn_force;

    cout << "u: " << u_ << endl;

    return true;
}
