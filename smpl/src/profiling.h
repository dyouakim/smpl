////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2010, Andrew Dornbush
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     1. Redistributions of source code must retain the above copyright notice
//        this list of conditions and the following disclaimer.
//     2. Redistributions in binary form must reproduce the above copyright
//        notice, this list of conditions and the following disclaimer in the
//        documentation and/or other materials provided with the distribution.
//     3. Neither the name of the copyright holder nor the names of its
//        contributors may be used to endorse or promote products derived from
//        this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////////////////////////////////////

/// \author Andrew Dornbush

#ifndef sbpl_manip_profiling_h
#define sbpl_manip_profiling_h

// standard includes
#include <chrono>
#include <functional>

// project includes
#include <smpl/time.h>
#include <smpl/console/console.h>

#ifndef SBPL_MANIP_PROFILING
#define SBPL_MANIP_PROFILING 0
#endif

namespace sbpl {

class RunUponDestruction
{
public:

    RunUponDestruction(const std::function<void()>& fun) : fun_(fun) { }
    ~RunUponDestruction() { fun_(); }

private:

    std::function<void()> fun_;
};

class Stopwatch
{
public:

    Stopwatch(const std::string& name, int throttle) :
        m_name(name),
        m_throttle(throttle),
        m_then(),
        m_elapsed(0.0),
        m_times(0)
    { }

    // Start a stopwatch and log the call frequency at the throttled rate
    void start()
    {
#if SBPL_MANIP_PROFILING
        if (m_elapsed != 0.0 && (m_times % m_throttle) == 0) {
            SMPL_INFO("%s: \t%0.3f Hz", m_name.c_str(), m_times / m_elapsed);
        }
        m_then = clock::now();
#endif
    }

    // Stop a stopwatch
    void stop()
    {
#if SBPL_MANIP_PROFILING
        auto now = clock::now();
        m_elapsed += std::chrono::duration<double>(now - m_then).count();
        ++m_times;
#endif
    }

    // Tick a stopwatch and log the call frequency at the throttled rate
    void tick()
    {
#if SBPL_MANIP_PROFILING
    if (m_elapsed != 0.0 && (m_times % m_throttle) == 0) {
        SMPL_INFO("%s: \t%0.3f Hz", m_name.c_str(), m_times / m_elapsed);
    }
    ++m_times;
#endif
    }

private:

    const std::string m_name;
    int m_throttle;
    clock::time_point m_then;
    double m_elapsed;
    int m_times;
};

} // namespace sbpl

// Create an object to automatically stop a stopwatch upon exiting the current scope
#if SBPL_MANIP_PROFILING
#define PROFAUTOSTOP(watch) ::sbpl::RunUponDestruction watch##_rod([&](){ watch.stop(); });
#else
#define PROFAUTOSTOP(watch)
#endif

#endif
