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
 * @brief Time Sync
 * @file TimeSync.hpp
 * @author Bastian Jäger <bastian@auterion.com>
 */

#pragma once

#include <common.h>

#include <chrono>
#include <iostream>

static constexpr double ALPHA_GAIN_INITIAL = 0.05;
static constexpr double BETA_GAIN_INITIAL = 0.05;
static constexpr double ALPHA_GAIN_FINAL = 0.003;
static constexpr double BETA_GAIN_FINAL = 0.003;

static constexpr uint32_t CONVERGENCE_WINDOW = 250;
static constexpr uint64_t MAX_RTT_SAMPLE = 10*1000;
static constexpr uint64_t MAX_DEVIATION_SAMPLE = 100*1000;
static constexpr uint32_t MAX_CONSECUTIVE_HIGH_DEVIATION = 5;

class TimeSync {
   public:
    TimeSync();
    ~TimeSync();
    TimeSync(const TimeSync&) = delete;
    auto operator=(const TimeSync&) -> const TimeSync& = delete;

    void run(int64_t _ts1, int64_t _tc1, uint64_t _now_us);
    uint64_t sync_stamp(uint64_t usec, uint64_t _now_us);

   private:
    bool sync_converged();
    void reset_filter();
    void add_sample(int64_t offset_us);

    uint32_t _sequence{0};

    // Timesync statistics
    double _time_offset{0};
    double _time_skew{0};

    // Filter parameters
    double _filter_alpha{ALPHA_GAIN_INITIAL};
    double _filter_beta{BETA_GAIN_INITIAL};

    // Outlier rejection and filter reset
    uint32_t _high_deviation_count{0};
    uint32_t _high_rtt_count{0};
};
