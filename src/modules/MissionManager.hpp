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
 * @file MissionManager.hpp
 *
 * @author Nuno Marques <nuno@auterion.com>
 */

#pragma once

#include <atomic>
#include <future>
#include <iostream>
#include <string>

#include <mavsdk/mavsdk.h>
#include "ModuleBase.hpp"

// MAVSDK dependencies
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/custom_action/custom_action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

class MissionManager : public ModuleBase {
public:
	MissionManager(std::shared_ptr<mavsdk::System> system);
	~MissionManager();
	MissionManager(const MissionManager&) = delete;
	const MissionManager& operator=(const MissionManager&) = delete;

	int init() override;
	void deinit() override;
	void run() override;

private:
	std::shared_ptr<mavsdk::System> _mavsdk_system;
	std::shared_ptr<mavsdk::CustomAction> _custom_action;
	std::shared_ptr<mavsdk::Telemetry> _telemetry;

	std::atomic<bool> _received_custom_action{false};
	std::atomic<bool> _mission_finished{false};
	std::atomic<bool> _action_stopped{false};
	std::atomic<bool> _new_action{false};
	std::atomic<bool> _new_actions_check_int{false};

	std::mutex cancel_mtx;
	std::condition_variable cancel_signal;

	std::vector<mavsdk::CustomAction::ActionToExecute> _actions;
	std::vector<double> _actions_progress;
	std::vector<mavsdk::CustomAction::Result> _actions_result;
	std::vector<mavsdk::CustomAction::ActionMetadata> _actions_metadata;
	std::vector<std::thread> _progress_threads;

	void new_action_check();
	void send_progress_status(mavsdk::CustomAction::ActionToExecute action);
	void process_custom_action(mavsdk::CustomAction::ActionToExecute action);
	void execute_custom_action(mavsdk::CustomAction::ActionMetadata action_metadata);
};
