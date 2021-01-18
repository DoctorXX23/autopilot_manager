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
 * @file main.cpp
 *
 * @author Nuno Marques <nuno@auterion.com>
 */

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/signalfd.h>
#include <unistd.h>

#include <mavsdk/mavsdk.h>

#include "modules/MissionManager.hpp"

static void usage(const std::string& bin_name);

/**
 * @brief Waits until we receive one of the signals requesting graceful termination.
 */
static void wait_for_termination_signal() {
	sigset_t sigs;
	sigemptyset(&sigs);
	sigaddset(&sigs, SIGINT);
	sigaddset(&sigs, SIGTERM);
	int sigfd = ::signalfd(-1, &sigs, SFD_CLOEXEC);
	struct signalfd_siginfo si;
	while (::read(sigfd, &si, sizeof(si)) == 0) {
	}
}

int main(int argc, char* argv[]) {
	// if (argc == 2) {
		// Configure MAVSDK Mission Manager instance
		mavsdk::Mavsdk mavsdk_mission_computer;

		// Change configuration so the instance is treated as a mission computer
		mavsdk::Mavsdk::Configuration config_cc(mavsdk::Mavsdk::Configuration::UsageType::CompanionComputer);
		mavsdk_mission_computer.set_configuration(config_cc);

		auto system = std::shared_ptr<mavsdk::System>{nullptr};

		mavsdk::ConnectionResult ret_comp = mavsdk_mission_computer.add_udp_connection(14540);
		if (ret_comp == mavsdk::ConnectionResult::Success) {
			std::cout << "Waiting to discover vehicle from the mission computer side" << std::endl;
			std::promise<void> prom;
			std::future<void> fut = prom.get_future();

			mavsdk_mission_computer.subscribe_on_new_system([&prom, &mavsdk_mission_computer, &system]() {
				for (const auto& sys : mavsdk_mission_computer.systems()) {
					if (sys->has_autopilot()) {
						system = sys;
						prom.set_value();
						break;
					}
				}
			});

			fut.wait_for(std::chrono::seconds(10));

			auto mission_manager = std::make_shared<MissionManager>(system);
			if (mission_manager->init() == 0) {
				mission_manager->run();
			}

		} else {
			std::cerr << "Failed to connect to port 14540" << std::endl;
		}

	// } else {
	// 	usage(argv[0]);
	// 	return 1;
	// }

	wait_for_termination_signal();
}

void usage(const std::string& bin_name) { std::cout << "Usage : " << bin_name << " mission_computer" << std::endl; }
