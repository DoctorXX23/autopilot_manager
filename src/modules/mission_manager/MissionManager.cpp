/****************************************************************************
 *
 *   Copyright (c) 2021 Auterion AG. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name Auterion nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @brief Mission Manager
 * @file MissionManager.cpp
 * @author Nuno Marques <nuno@auterion.com>
 */

#include <MissionManager.hpp>
#include <atomic>
#include <future>
#include <iostream>
#include <string>
#include <utility>

static std::atomic<bool> int_signal{false};

MissionManager::MissionManager(std::shared_ptr<mavsdk::System> mavsdk_system,
                               const std::string& path_to_custom_action_file)
    : _config_update_callback([]() { return MissionManagerConfiguration{}; }),
      _path_to_custom_action_file{std::move(path_to_custom_action_file)},
      _mission_manager_config{},
      _mavsdk_system{std::move(mavsdk_system)},
      _action_triggered{false},
      _get_gps_origin_success{false},
      _global_origin_reference_set{false},
      _is_stationary_debounce_counter{0},
      _current_latitude{0.0},
      _current_longitude{0.0},
      _current_altitude_amsl{0.0},
      _new_latitude{0.0},
      _new_longitude{0.0},
      _new_altitude_amsl{0.0},
      _previously_set_waypoint_latitude{0.0},
      _previously_set_waypoint_longitude{0.0},
      _previously_set_waypoint_altitude_amsl{0.0},
      _original_max_speed{-1.0},
      _landing_planner{} {}

MissionManager::~MissionManager() { deinit(); }

void MissionManager::init() {
    std::cout << missionManagerOut << "Started!" << std::endl;

    // Actions are processed and executed in the Mission Manager decion maker
    _action = std::make_shared<mavsdk::Action>(_mavsdk_system);

    // Telemetry data checks are fundamental for proper execution
    _telemetry = std::make_shared<mavsdk::Telemetry>(_mavsdk_system);

    // Bring up a ServerUtility instance to allow sending status messages
    _server_utility = std::make_shared<mavsdk::ServerUtility>(_mavsdk_system);

    _custom_action_handler =
        std::make_shared<CustomActionHandler>(_mavsdk_system, _telemetry, _path_to_custom_action_file);
}

void MissionManager::deinit() {
    int_signal.store(true, std::memory_order_relaxed);

    _decision_maker_th.join();
    _global_origin_reference_th.join();
    _custom_action_handler.reset();
}

void MissionManager::run() {
    // Start the desicion maker thread
    _decision_maker_th = std::thread(&MissionManager::decision_maker_run, this);

    // Get the GPS global origin and set the global origin reference
    _global_origin_reference_th = std::thread(&MissionManager::set_global_position_reference, this);

    // Start custom action handler
    if (_custom_action_handler->start()) {
        _custom_action_handler->run();
    }
}

void MissionManager::set_global_position_reference() {
    bool result{false};
    std::string status{};

    while (!_get_gps_origin_success && !int_signal) {
        // Get global origin to set the reference global position
        auto cmd_result = _telemetry->get_gps_global_origin();

        if (cmd_result.first == mavsdk::Telemetry::Result::Success) {
            std::cout << std::string(missionManagerOut) + "Successfully received a GPS_GLOBAL_ORIGIN MAVLink message"
                      << std::endl;
            _get_gps_origin_success = true;

            if (cmd_result.second.latitude_deg != 0.0 && cmd_result.second.longitude_deg != 0.0) {
                _ref_latitude = cmd_result.second.latitude_deg;
                _ref_longitude = cmd_result.second.longitude_deg;
                _ref_altitude = cmd_result.second.altitude_m;

                status = std::string(missionManagerOut) +
                         "Global position reference initialiazed: Latitude: " + std::to_string(_ref_latitude) +
                         " deg | Longitude: " + std::to_string(_ref_longitude) +
                         " deg | Altitude (AMSL): " + std::to_string(_ref_altitude) + " meters";
                _server_utility->send_status_text(mavsdk::ServerUtility::StatusTextType::Info, status);

                result = true;

            } else {
                status = std::string(missionManagerOut) +
                         "Failed to set the global origin reference because the received values are invalid.";
                _server_utility->send_status_text(mavsdk::ServerUtility::StatusTextType::Error, status);
            }

        } else if (cmd_result.first == mavsdk::Telemetry::Result::Timeout) {
            std::cout << std::string(missionManagerOut) +
                             "GPS_GLOBAL_ORIGIN stream request timeout. Message not yet received. Retrying..."
                      << std::endl;
        } else {
            std::cout << std::string(missionManagerOut) + "GPS_GLOBAL_ORIGIN stream request failed. Retrying..."
                      << std::endl;
        }
    }

    std::cout << status << std::endl;
    _global_origin_reference_set = result;
}

mavsdk::geometry::CoordinateTransformation::LocalCoordinate MissionManager::get_local_position_from_local_offset(
    const double& offset_x, const double& offset_y) const {
    // Given an offset in body-frame x and y, compute the local position
    const double local_position_x =
        (std::cos(_current_yaw) * offset_x - std::sin(_current_yaw) * offset_y) + _current_pos_x;
    const double local_position_y =
        (std::sin(_current_yaw) * offset_x + std::cos(_current_yaw) * offset_y) + _current_pos_y;
    return mavsdk::geometry::CoordinateTransformation::LocalCoordinate{local_position_x, local_position_y};
}

mavsdk::geometry::CoordinateTransformation::GlobalCoordinate MissionManager::get_global_position_from_local_position(
    mavsdk::geometry::CoordinateTransformation::LocalCoordinate local_position) const {
    const mavsdk::geometry::CoordinateTransformation ct({_ref_latitude, _ref_longitude});
    return ct.global_from_local(local_position);
}

mavsdk::geometry::CoordinateTransformation::GlobalCoordinate MissionManager::get_global_position_from_local_offset(
    const double& offset_x, const double& offset_y) const {
    const mavsdk::geometry::CoordinateTransformation::LocalCoordinate local_position =
        get_local_position_from_local_offset(offset_x, offset_y);
    return get_global_position_from_local_position(local_position);
}

void MissionManager::set_new_waypoint(const double& lat, const double& lon, const double& alt_amsl) {
    _new_latitude = lat;
    _new_longitude = lon;
    _new_altitude_amsl = alt_amsl;
}

bool MissionManager::arrived_to_new_waypoint() {
    if ((std::abs(_new_latitude - _current_latitude) <= 1.0E-5) &&
        (std::abs(_new_longitude - _current_longitude) <= 1.0E-5) &&
        (std::abs(_new_altitude_amsl - _current_altitude_amsl) <= 1.0)) {  // ~1 meter acceptance radius
        return true;
    }

    return false;
}

bool MissionManager::is_stationary() {
    static const double vel_tol = 0.2;
    const double vel_mag =
        std::sqrt(std::pow(_current_vel_x, 2) + std::pow(_current_vel_y, 2) + std::pow(_current_vel_z, 2));
    return debounce_is_stationary(vel_mag < vel_tol);
}

bool MissionManager::debounce_is_stationary(bool is_stationary) {
    // The purpose of the debouncing is not only to prevent multiple triggers, but also to include a delay between when
    // the vehicle stops and when it is allowed to process any actions. This gives the landing site detection
    // time to assess the ground below the stopping point.
    static const int DEBOUNCE_COUNT_REQUIRED = 10;
    if (is_stationary) {
        _is_stationary_debounce_counter++;
        if (_is_stationary_debounce_counter >= DEBOUNCE_COUNT_REQUIRED) {
            // Vehicle is confirmed to be stationary
            _is_stationary_debounce_counter = 0;
            return true;
        }
    } else {
        _is_stationary_debounce_counter = 0;
    }
    return false;
}

void MissionManager::go_to_new_local_waypoint(
    mavsdk::geometry::CoordinateTransformation::LocalCoordinate local_waypoint, const double altitude,
    const double yaw_rad) {
    // Convert to global position
    const mavsdk::geometry::CoordinateTransformation::GlobalCoordinate waypoint =
        get_global_position_from_local_position(local_waypoint);

    // Go to waypoint
    _action->goto_location(waypoint.latitude_deg, waypoint.longitude_deg, altitude, yaw_rad);
    set_new_waypoint(waypoint.latitude_deg, waypoint.longitude_deg, altitude);
}

void MissionManager::handle_safe_landing(std::chrono::time_point<std::chrono::system_clock> now) {
    std::unique_lock<std::mutex> lock(mission_manager_config_mtx);
    const bool safe_landing_enabled = _mission_manager_config.safe_landing_enabled;
    const float safe_landing_distance_to_ground = _mission_manager_config.safe_landing_distance_to_ground;
    const bool safe_landing_try_landing_after_action = _mission_manager_config.safe_landing_try_landing_after_action;
    const std::string safe_landing_on_no_safe_land = _mission_manager_config.safe_landing_on_no_safe_land;
    const double landing_site_search_max_speed = _mission_manager_config.landing_site_search_max_speed;
    const double landing_site_search_spiral_spacing = _mission_manager_config.landing_site_search_spiral_spacing;
    const double landing_site_search_spiral_max_size = _mission_manager_config.landing_site_search_spiral_max_size;
    const int landing_site_search_spiral_points = _mission_manager_config.landing_site_search_spiral_points;
    const double global_position_waypoint_lat = _mission_manager_config.global_position_waypoint_lat;
    const double global_position_waypoint_lon = _mission_manager_config.global_position_waypoint_lon;
    const double global_position_waypoint_alt_amsl = _mission_manager_config.global_position_waypoint_alt_amsl;
    const double local_position_offset_x = _mission_manager_config.local_position_offset_x;
    const double local_position_offset_y = _mission_manager_config.local_position_offset_y;
    const double local_position_offset_z = _mission_manager_config.local_position_offset_z;
    lock.unlock();

    const uint8_t safe_landing_state = _landing_condition_state_update_callback();
    const float height_above_obstacle = _height_above_obstacle_update_callback();

    if (safe_landing_enabled) {
        /*
         * If we're not handling an action already, check if we need to start one.
         */
        if (!_action_triggered) {
            std::string status{};

            if (_landed_state == mavsdk::Telemetry::LandedState::Landing) {
                if (height_above_obstacle > 1.5 && safe_landing_state == 0 /*eLandingMapperState::UNHEALTHY*/) {
                    // If the safe landing status is unhealthy, then hold position.
                    _action->hold();

                    status = std::string(missionManagerOut) + "Safe landing system not healthy. Holding position...";
                    _server_utility->send_status_text(mavsdk::ServerUtility::StatusTextType::Warning, status);
                    std::cout << status << std::endl;

                    _action_triggered = true;
                    _last_time = now;

                } else if (height_above_obstacle > 1.5 && safe_landing_state == 1 /*eLandingMapperState::UNKNOWN*/) {
                    // If the vehicle is bellow the defined maximum distance to ground to determine if it is
                    // safe to land or not, then it will still try to land until it reaches an height that
                    // allows it to determine if it can land or not. Otherwise, it holds position.
                    _action->hold();

                    status =
                        std::string(missionManagerOut) + "Cannot determine if it is safe to land. Holding position...";
                    _server_utility->send_status_text(mavsdk::ServerUtility::StatusTextType::Warning, status);
                    std::cout << status << std::endl;

                    _action_triggered = true;
                    _last_time = now;

                } else if (safe_landing_state == 4 /*eLandingMapperState::CAN_NOT_LAND*/) {
                    std::cout << std::string(missionManagerOut) << "Cannot land! -----------------------" << std::endl;
                    if (safe_landing_on_no_safe_land == "HOLD") {
                        _action->hold();

                        status = std::string(missionManagerOut) + "Position hold triggered for Safe Landing";
                        _server_utility->send_status_text(mavsdk::ServerUtility::StatusTextType::Info, status);
                        std::cout << status << std::endl;

                    } else if (safe_landing_on_no_safe_land == "RTL") {
                        _action->return_to_launch();

                        status = std::string(missionManagerOut) + "RTL triggered for Safe Landing";
                        _server_utility->send_status_text(mavsdk::ServerUtility::StatusTextType::Info, status);
                        std::cout << status << std::endl;

                    } else if (safe_landing_on_no_safe_land == "MOVE_XYZ_WRT_CURRENT") {
                        // get the global origin from the FMU and set the reference
                        if (_global_origin_reference_set) {
                            const auto waypoint =
                                get_global_position_from_local_offset(local_position_offset_x, local_position_offset_y);
                            const double waypoint_altitude = _current_altitude_amsl + local_position_offset_z;

                            _action->goto_location(waypoint.latitude_deg, waypoint.longitude_deg, waypoint_altitude,
                                                   NAN);

                            set_new_waypoint(waypoint.latitude_deg, waypoint.longitude_deg, waypoint_altitude);

                            status =
                                std::string(missionManagerOut) +
                                "Moving XYZ WRT to current vehicle position triggered for Safe Landing. Heading to "
                                "determined "
                                "Latitude " +
                                std::to_string(waypoint.latitude_deg) + " deg, Longitude " +
                                std::to_string(waypoint.longitude_deg) + " deg, Altitude (AMSL) " +
                                std::to_string(waypoint_altitude) + " meters";
                            _server_utility->send_status_text(mavsdk::ServerUtility::StatusTextType::Info, status);

                        } else {
                            _action->hold();

                            status = std::string(missionManagerOut) + "Holding position...";
                            _server_utility->send_status_text(mavsdk::ServerUtility::StatusTextType::Warning, status);
                        }

                        std::cout << status << std::endl;

                    } else if (safe_landing_on_no_safe_land == "LANDING_SITE_SEARCH") {
                        if (_global_origin_reference_set) {
                            std::cout << "*" << std::endl
                                      << "***" << std::endl
                                      << "***** Starting Landing Site Search" << std::endl
                                      << "***" << std::endl
                                      << "*" << std::endl;

                            _landing_planner.startSearch(_current_pos_x, _current_pos_y, _current_yaw,
                                                         _current_altitude_amsl, landing_site_search_spiral_spacing,
                                                         landing_site_search_spiral_max_size,
                                                         landing_site_search_spiral_points);

                            if (_landing_planner.isActive()) {
                                // Lower the maximum speed
                                _action->set_maximum_speed(landing_site_search_max_speed);
                                std::cout << std::string(missionManagerOut) << "Lowering maximum speed from "
                                          << _original_max_speed << " to " << landing_site_search_max_speed
                                          << std::endl;

                                // Set the first waypoint in the search pattern
                                go_to_new_local_waypoint(_landing_planner.getCurrentWaypoint(),
                                                         _landing_planner.getSearchAltitude(),
                                                         _current_yaw * 180. / M_PI);
                                status = std::string(missionManagerOut) + "Landing site search started at: Latitude " +
                                         std::to_string(_new_latitude) + " deg, Longitude " +
                                         std::to_string(_new_longitude) + " deg, Altitude (AMSL) " +
                                         std::to_string(_new_altitude_amsl) + " meters";
                                _server_utility->send_status_text(mavsdk::ServerUtility::StatusTextType::Info, status);
                            } else {
                                // Planner did not start correctly.
                                _action->hold();

                                status = std::string(missionManagerOut) +
                                         "Landing planner could not start. Holding position...";
                                _server_utility->send_status_text(mavsdk::ServerUtility::StatusTextType::Warning,
                                                                  status);
                            }
                        } else {
                            _action->hold();

                            status = std::string(missionManagerOut) +
                                     "Global position reference not set. Holding position...";
                            _server_utility->send_status_text(mavsdk::ServerUtility::StatusTextType::Warning, status);
                        }

                        std::cout << status << std::endl;

                    } else if (safe_landing_on_no_safe_land == "GO_TO_WAYPOINT_XYZ") {
                        _action->hold();

                        status =
                            std::string(missionManagerOut) +
                            "GO_TO_WAYPOINT_XYZ action currently not supported for Safe Landing. Holding position...";
                        _server_utility->send_status_text(mavsdk::ServerUtility::StatusTextType::Warning, status);
                        std::cout << status << std::endl;

                    } else if (safe_landing_on_no_safe_land == "MOVE_LLA_WRT_CURRENT") {
                        _action->hold();

                        status =
                            std::string(missionManagerOut) +
                            "MOVE_LLA_WRT_CURRENT action currently not supported for Safe Landing. Holding position...";
                        _server_utility->send_status_text(mavsdk::ServerUtility::StatusTextType::Warning, status);
                        std::cout << status << std::endl;

                    } else if (safe_landing_on_no_safe_land == "GO_TO_WAYPOINT") {
                        std::string status{};

                        // If this action run previously, but the user didn't change the waypoint online after the
                        // action was triggered, the previous waypoint will match the current waypoint. If that's the
                        // case enfore an RTL so to avoid the vehicle getting stuck trying to land in a place it can't
                        // land
                        if ((std::abs(global_position_waypoint_lat - _current_latitude) <= 1.0E-5) &&
                            (std::abs(global_position_waypoint_lon - _current_longitude) <= 1.0E-5) &&
                            (std::abs(_previously_set_waypoint_altitude_amsl - _current_altitude_amsl) <= 1.0)) {
                            _action->return_to_launch();

                            status =
                                std::string(missionManagerOut) +
                                "Go-To Global Position Waypoint not triggered for Safe Landing, as the waypoint set "
                                "is the same as the "
                                "global position of the vehicle."
                                "RTL triggered instead...";
                            _server_utility->send_status_text(mavsdk::ServerUtility::StatusTextType::Warning, status);

                        } else {
                            // get the global origin from the FMU and set the reference
                            if (_global_origin_reference_set) {
                                // then send the DO_REPOSITION
                                _action->goto_location(global_position_waypoint_lat, global_position_waypoint_lon,
                                                       global_position_waypoint_alt_amsl, NAN);

                                set_new_waypoint(global_position_waypoint_lat, global_position_waypoint_lon,
                                                 global_position_waypoint_alt_amsl);

                                status =
                                    std::string(missionManagerOut) +
                                    "Go-To Global Position Waypoint triggered for Safe Landing. Heading to Latitude " +
                                    std::to_string(global_position_waypoint_lat) + " deg, Longitude " +
                                    std::to_string(global_position_waypoint_lon) + " deg, Altitude (AMSL) " +
                                    std::to_string(global_position_waypoint_alt_amsl) + " meters";
                                _server_utility->send_status_text(mavsdk::ServerUtility::StatusTextType::Info, status);

                            } else {
                                _action->hold();

                                status = std::string(missionManagerOut) + "Holding position...";
                                _server_utility->send_status_text(mavsdk::ServerUtility::StatusTextType::Warning,
                                                                  status);
                            }
                        }

                        std::cout << status << std::endl;

                    } else if (safe_landing_on_no_safe_land == "SCRIPT_CALL") {
                        _action->hold();

                        status = std::string(missionManagerOut) +
                                 "SCRIPT_CALL action currently not supported for Safe Landing. Holding position...";
                        _server_utility->send_status_text(mavsdk::ServerUtility::StatusTextType::Warning, status);
                        std::cout << status << std::endl;

                    } else if (safe_landing_on_no_safe_land == "API_CALL") {
                        _action->hold();

                        status = std::string(missionManagerOut) +
                                 "API_CALL action currently not supported for Safe Landing. Holding position...";
                        _server_utility->send_status_text(mavsdk::ServerUtility::StatusTextType::Warning, status);
                        std::cout << status << std::endl;
                    }

                    _action_triggered = true;
                    _last_time = now;
                }
            }
        }

        /*
         *  Process any active actions
         */
        if (_landing_planner.isActive()) {
            if (_landed_state == mavsdk::Telemetry::LandedState::OnGround) {
                // Vehicle landed. Stop the landing site search.
                _landing_planner.endSearch();
                landing_site_search_has_ended();
            } else {
                update_landing_site_search(safe_landing_state, safe_landing_try_landing_after_action);
            }
        } else if (std::isfinite(_new_latitude) && std::isfinite(_new_longitude) && std::isfinite(_new_altitude_amsl)) {
            if (arrived_to_new_waypoint()) {
                // if the user set it wants to try landing again after arriving to the waypoint
                // then land and unset the new waypoint
                if (safe_landing_try_landing_after_action) {
                    _action->land();
                    set_new_waypoint(NAN, NAN, NAN);
                }
            }
        }
    }
}

void MissionManager::handle_simple_collision_avoidance(std::chrono::time_point<std::chrono::system_clock> now) {
    if (_mission_manager_config.simple_collision_avoid_enabled != 0U) {
        const bool in_air = (_landed_state == mavsdk::Telemetry::LandedState::InAir);
        // std::cout << "Depth measured: " << _distance_to_obstacle_update_callback()
        //           << " | threshold: " << _mission_manager_config.simple_collision_avoid_distance_threshold
        //           << " | in air: " << in_air << " | is action triggered? " << std::boolalpha
        //           << _action_triggered << std::endl;

        if (std::isfinite(_distance_to_obstacle_update_callback()) &&
            _distance_to_obstacle_update_callback() <=
                _mission_manager_config.simple_collision_avoid_distance_threshold &&
            in_air && !_action_triggered) {  // only trigger the condition when the vehicle is in-air
            if (_mission_manager_config.simple_collision_avoid_action_on_condition_true == "HOLD") {
                _action->hold();
                std::cout << std::string(missionManagerOut) << "Position hold triggered for Simple Obstacle Avoidance"
                          << std::endl;
            } else if (_mission_manager_config.simple_collision_avoid_action_on_condition_true == "RTL") {
                _action->return_to_launch();
                std::cout << std::string(missionManagerOut) << "RTL triggered for Simple Obstacle Avoidance"
                          << std::endl;
            } else if (_mission_manager_config.simple_collision_avoid_action_on_condition_true ==
                       "MOVE_XYZ_WRT_CURRENT") {
                _action->hold();
                std::cout << std::string(missionManagerOut)
                          << "MOVE_XYZ_WRT_CURRENT action currently not supported for Simple Obstacle Avoidance. "
                             "Holding position..."
                          << std::endl;
            } else if (_mission_manager_config.simple_collision_avoid_action_on_condition_true ==
                       "GO_TO_WAYPOINT_XYZ") {
                _action->hold();
                std::cout << std::string(missionManagerOut)
                          << "GO_TO_WAYPOINT_XYZ action currently not supported for Simple Obstacle Avoidance. Holding "
                             "position..."
                          << std::endl;
            } else if (_mission_manager_config.simple_collision_avoid_action_on_condition_true ==
                       "MOVE_LLA_WRT_CURRENT") {
                _action->hold();
                std::cout << std::string(missionManagerOut)
                          << "MOVE_LLA_WRT_CURRENT action currently not supported for Simple Obstacle Avoidance. "
                             "Holding position..."
                          << std::endl;
            } else if (_mission_manager_config.simple_collision_avoid_action_on_condition_true == "GO_TO_WAYPOINT") {
                _action->hold();
                std::cout << std::string(missionManagerOut)
                          << "GO_TO_WAYPOINT action currently not supported for Simple Obstacle Avoidance. Holding "
                             "position..."
                          << std::endl;
            } else if (_mission_manager_config.simple_collision_avoid_action_on_condition_true == "LAND") {
                _action->land();
                std::cout << std::string(missionManagerOut) << "Land triggered for Simple Obstacle Avoidance"
                          << std::endl;
            } else if (_mission_manager_config.simple_collision_avoid_action_on_condition_true == "SCRIPT_CALL") {
                _action->hold();
                std::cout
                    << std::string(missionManagerOut)
                    << "SCRIPT_CALL action currently not supported for Simple Obstacle Avoidance. Holding position..."
                    << std::endl;
            } else if (_mission_manager_config.simple_collision_avoid_action_on_condition_true == "API_CALL") {
                _action->hold();
                std::cout
                    << std::string(missionManagerOut)
                    << "API_CALL action currently not supported for Simple Obstacle Avoidance. Holding position..."
                    << std::endl;
            }

            _action_triggered = true;
            _last_time = now;
        }
    }
}

void MissionManager::update_landing_site_search(const uint8_t safe_landing_state, const bool land_when_found_site) {
    /*
     *  STEP 1: Decide what to do
     */
    bool should_initiate_landing = false;
    const landing_planner::LandingSearchState search_state = _landing_planner.state();
    if (search_state == landing_planner::LandingSearchState::SEARCH_ACTIVE &&
        safe_landing_state == 3 /*eLandingMapperState::CAN_LAND*/) {
        // Observed a safe place while searching for a landing site.
        _landing_planner.candidateSiteFoundAt({_current_pos_x, _current_pos_y});
    } else if (search_state == landing_planner::LandingSearchState::ATTEMPTING_TO_LAND) {
        if (safe_landing_state == 0 /*eLandingMapperState::UNHEALTHY*/ ||
            safe_landing_state == 1 /*eLandingMapperState::UNKNOWN*/ ||
            safe_landing_state == 4 /*eLandingMapperState::CAN_NOT_LAND*/) {
            // Attempting to land, but there's a problem.
            _action->hold();
            _landing_planner.abortLanding();
            std::string status = std::string(missionManagerOut) + "Aborting landing at candidate site";
            _server_utility->send_status_text(mavsdk::ServerUtility::StatusTextType::Info, status);
            std::cout << status << std::endl;
        }
    } else if (arrived_to_new_waypoint()) {
        _landing_planner.arrivedAtWaypoint(is_stationary(), safe_landing_state);
        if (_landing_planner.state() == landing_planner::LandingSearchState::ATTEMPTING_TO_LAND) {
            // Switched into ATTEMPTING_TO_LAND
            should_initiate_landing = true;
        }
    } else {
        // Didn't find a landing site AND
        // Not attempting to land AND
        // Not at a waypoint:
        // --> We're flying to a waypoint (or not initialised)
        // --> Keep going
    }

    /*
     *  STEP 2: Send commands to vehicle
     */
    std::string status = std::string(missionManagerOut);
    if (should_initiate_landing) {
        status += "Landing site found. ";
        set_new_waypoint(NAN, NAN, NAN);
        if (land_when_found_site) {
            status += "Landing...";
            _action->land();
        } else {
            status += "Holding position...";
            _action->hold();
            // End the search
            _landing_planner.endSearch();
        }
        _server_utility->send_status_text(mavsdk::ServerUtility::StatusTextType::Info, status);
    } else if (_landing_planner.state() == landing_planner::LandingSearchState::ENDED) {
        // End of search pattern.
        // Hold position.
        status += "End of landing site search. Holding position...";
        set_new_waypoint(NAN, NAN, NAN);
        _action->hold();
        _server_utility->send_status_text(mavsdk::ServerUtility::StatusTextType::Info, status);
    } else if (_landing_planner.waypointUpdated()) {
        // Command new waypoint
        go_to_new_local_waypoint(_landing_planner.getCurrentWaypoint(), _landing_planner.getSearchAltitude(),
                                 _current_yaw * 180. / M_PI);
        status += "[Landing Site Search] Waypoint set: Latitude " + std::to_string(_new_latitude) + " deg, " +
                  "Longitude " + std::to_string(_new_longitude) + " deg, Altitude (AMSL) " +
                  std::to_string(_new_altitude_amsl) + " meters";
        // _server_utility->send_status_text(mavsdk::ServerUtility::StatusTextType::Info, status);
    } else {
        // No change in behaviour
        return;
    }
    std::cout << status << std::endl;

    /*
     *  STEP 3: Restore normal flight configuration if search has ended
     */
    if (_landing_planner.state() == landing_planner::LandingSearchState::ENDED) {
        landing_site_search_has_ended();
    }
}

void MissionManager::landing_site_search_has_ended() {
    _action->set_maximum_speed(_original_max_speed);
    std::cout << std::string(missionManagerOut) << "Restoring speed to " << _original_max_speed << std::endl;
    _action_triggered = false;
    std::cout << "    *" << std::endl
              << "  ***" << std::endl
              << "***** Landing Site Search has ended" << std::endl
              << "  ***" << std::endl
              << "    *" << std::endl;
}

void MissionManager::decision_maker_run() {
    // Init action trigger timer
    _last_time = std::chrono::system_clock::now();

    // Get global position
    _telemetry->subscribe_position([this](mavsdk::Telemetry::Position position) {
        _current_latitude = position.latitude_deg;
        _current_longitude = position.longitude_deg;
        _current_altitude_amsl = position.absolute_altitude_m;
    });

    // Get global and home positions health
    _telemetry->subscribe_health([this](mavsdk::Telemetry::Health health) {
        _is_global_position_ok = health.is_global_position_ok;
        _is_home_position_ok = health.is_home_position_ok;
    });

    // Get local position and velocity
    _telemetry->subscribe_position_velocity_ned([this](mavsdk::Telemetry::PositionVelocityNed position_velocity) {
        _current_pos_x = position_velocity.position.north_m;
        _current_pos_y = position_velocity.position.east_m;
        _current_pos_z = position_velocity.position.down_m;
        _current_vel_x = position_velocity.velocity.north_m_s;
        _current_vel_y = position_velocity.velocity.east_m_s;
        _current_vel_z = position_velocity.velocity.down_m_s;
    });

    // Get the original maximum speed when not doing a landing site search
    _original_max_speed = _action->get_maximum_speed().second;
    std::cout << "Original maximum speed = " << _original_max_speed << std::endl;

    // Get yaw
    _telemetry->subscribe_attitude_euler(
        [this](mavsdk::Telemetry::EulerAngle euler_angle) { _current_yaw = euler_angle.yaw_deg * M_PI / 180.0; });

    // Get the landing state so we know when the vehicle is in-air, landing or on-ground
    _telemetry->subscribe_landed_state(
        [this](mavsdk::Telemetry::LandedState landed_state) { _landed_state = landed_state; });

    while (!int_signal) {
        // Update configuration at each iteration
        _mission_manager_config = _config_update_callback();

        if (_mission_manager_config.autopilot_manager_enabled) {
            auto now = std::chrono::system_clock::now();

            if (_mission_manager_config.decision_maker_input_type == "SAFE_LANDING") {
                handle_safe_landing(now);
            } else if (_mission_manager_config.decision_maker_input_type == "SIMPLE_COLLISION_AVOIDANCE") {
                handle_simple_collision_avoidance(now);
            }

            // After an action is triggered, we give it 5 seconds to process it before retrying.
            // Except the landing site search, which must terminate itself.
            if (std::chrono::duration_cast<std::chrono::milliseconds>(now - _last_time).count() >= 5000 &&
                !_landing_planner.isActive()) {
                _action_triggered = false;
            }
        }

        // Decision maker runs at 20hz
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}
