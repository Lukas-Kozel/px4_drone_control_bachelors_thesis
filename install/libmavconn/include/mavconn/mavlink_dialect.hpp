//
// libmavconn
// Copyright 2014,2015,2016,2021 Vladimir Ermakov, All rights reserved.
//
// This file is part of the mavros package and subject to the license terms
// in the top-level LICENSE file of the mavros repository.
// https://github.com/mavlink/mavros/tree/master/LICENSE.md
//
/**
 * @brief MAVConn mavlink.h selector
 * @file mavlink_dialect.hpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup mavconn
 * @{
 */

#pragma once
#ifndef MAVCONN__MAVLINK_DIALECT_HPP_
#define MAVCONN__MAVLINK_DIALECT_HPP_

// AUTOMATIC GENERATED FILE!
// from include/mavconn/mavlink_dialect.hpp.em

namespace mavlink
{
#ifndef MAVLINK_VERSION
#include <mavlink/config.h>
constexpr auto version = MAVLINK_VERSION;
#undef MAVLINK_VERSION
#else
constexpr auto version = "unknown";
#endif
}  // namespace mavlink

#define MAVLINK_START_SIGN_STREAM(link_id)
#define MAVLINK_END_SIGN_STREAM(link_id)

// NOTE(vooon): ignore warning
// warning: ISO C++ prohibits anonymous structs
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"

#include <mavlink/v2.0/common/common.hpp>
#include <mavlink/v2.0/ardupilotmega/ardupilotmega.hpp>
#include <mavlink/v2.0/ASLUAV/ASLUAV.hpp>
#include <mavlink/v2.0/AVSSUAS/AVSSUAS.hpp>
#include <mavlink/v2.0/all/all.hpp>
#include <mavlink/v2.0/csAirLink/csAirLink.hpp>
#include <mavlink/v2.0/cubepilot/cubepilot.hpp>
#include <mavlink/v2.0/development/development.hpp>
#include <mavlink/v2.0/icarous/icarous.hpp>
#include <mavlink/v2.0/matrixpilot/matrixpilot.hpp>
#include <mavlink/v2.0/paparazzi/paparazzi.hpp>
#include <mavlink/v2.0/standard/standard.hpp>
#include <mavlink/v2.0/storm32/storm32.hpp>
#include <mavlink/v2.0/uAvionix/uAvionix.hpp>
#include <mavlink/v2.0/ualberta/ualberta.hpp>


#pragma GCC diagnostic pop

#endif  // MAVCONN__MAVLINK_DIALECT_HPP_
