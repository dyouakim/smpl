////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2016, Andrew Dornbush
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

#ifndef SMPL_VISUALIZE_H
#define SMPL_VISUALIZE_H

// standard includes
#include <chrono>
#include <string>
#include <unordered_map>

#include <smpl/config.h>

// system includes
#ifdef SMPL_SV_VISUALIZATION_MSGS
#include <visualization_msgs/MarkerArray.h>
#endif

// project includes
#include <smpl/time.h>
#include <smpl/debug/marker.h>

namespace sbpl {
namespace visual {

enum struct Level
{
    Invalid = -1,
    Debug,
    Info,
    Warn,
    Error,
    Fatal,

    NumLevels
};

/// \name Global Visualizer
///@{

class VisualizerBase
{
public:

    virtual ~VisualizerBase() { }

    virtual void visualize(Level level, const visual::Marker& marker);
    virtual void visualize(Level level, const std::vector<visual::Marker>& markers);
#ifdef SMPL_SV_VISUALIZATION_MSGS
    virtual void visualize(Level level, const visualization_msgs::Marker& m);
    virtual void visualize(Level level, const visualization_msgs::MarkerArray& markers);
#endif
};

void set_visualizer(VisualizerBase* visualizer);
void unset_visualizer();
auto visualizer() -> VisualizerBase*;

///@}

/// \name Global Visualization Management
///@{

using VisualizationMap = std::unordered_map<std::string, Level>;
void get_visualizations(VisualizationMap& visualizations);

bool set_visualization_level(const std::string& name, Level level);

///@}

/// \name Internal
///@{

void visualize(Level level, const visual::Marker& marker);
void visualize(Level level, const std::vector<visual::Marker>& markers);
#ifdef SMPL_SV_VISUALIZATION_MSGS
void visualize(Level level, const visualization_msgs::Marker& marker);
void visualize(Level level, const visualization_msgs::MarkerArray& markers);
#endif

struct VizLocation {
    void* handle; // struct representing the named visualization at location
    VizLocation *next; // forward list pointer to the next viz location
    ::sbpl::visual::Level level;
    bool initialized;
    bool enabled;
};

void InitializeVizLocation(
    VizLocation* loc,
    const std::string& name,
    Level level);
///@}

} // namespace viz

namespace viz = visual;

} // namespace sbpl

#define SBPL_VISUALIZE_SEVERITY_DEBUG   0
#define SBPL_VISUALIZE_SEVERITY_INFO    1
#define SBPL_VISUALIZE_SEVERITY_WARN    2
#define SBPL_VISUALIZE_SEVERITY_ERROR   3
#define SBPL_VISUALIZE_SEVERITY_FATAL   4
#define SBPL_VISUALIZE_SEVERITY_NONE    5

#ifndef SBPL_VISUALIZE_MIN_SEVERITY
#define SBPL_VISUALIZE_MIN_SEVERITY SBPL_VISUALIZE_SEVERITY_DEBUG
#endif

#define SV_ROOT_VIZ_NAME "sv"

#ifdef SV_PACKAGE_NAME
#define SV_NAME_PREFIX SV_ROOT_VIZ_NAME "." SV_PACKAGE_NAME
#else
#define SV_NAME_PREFIX SV_ROOT_VIZ_NAME
#endif

#define SV_SHOW_DEFINE_LOCATION(cond_, level_, name_) \
    static ::sbpl::visual::VizLocation __sv_define_location__loc = { \
             nullptr, \
             nullptr, \
            ::sbpl::visual::Level::NumLevels, \
            false, \
            false, \
    }; \
    if (!__sv_define_location__loc.initialized) { \
        InitializeVizLocation(&__sv_define_location__loc, name_, level_); \
    } \
    bool __sv_define_location__enabled = \
            __sv_define_location__loc.enabled && (cond_)

#define SV_SHOW_COND(cond, level, name, markers) \
    do { \
        SV_SHOW_DEFINE_LOCATION(cond, level, name); \
        if (__sv_define_location__enabled) { \
            ::sbpl::visual::visualize(level, markers); \
        } \
    } \
    while (0)

#define SV_SHOW_ONCE(level, name, markers) \
    do { \
        static bool hit = false; \
        if (!hit) { \
            hit = true; \
            ::sbpl::visual::visualize(level, markers); \
        } \
    } while (0)

#define SV_SHOW_THROTTLE(rate, level, name, markers) \
    do { \
        static ::sbpl::clock::time_point last_hit; \
        static auto rate_dur = \
                ::std::chrono::duration_cast<::sbpl::clock::duration>( \
                        ::std::chrono::duration<double>(1.0 / (double)rate)); \
        auto now = ::sbpl::clock::now(); \
        if (last_hit + rate_dur <= now) { \
            last_hit = now; \
            ::sbpl::visual::visualize(level, markers); \
        } \
    } while (0)

#define SV_SHOW(level, name, markers) SV_SHOW_COND(true, level, name, markers)

#if (SBPL_VISUALIZE_MIN_SEVERITY > SBPL_VISUALIZE_SEVERITY_DEBUG)
#define SV_SHOW_DEBUG(markers)
#define SV_SHOW_DEBUG_NAMED(name, markers)
#define SV_SHOW_DEBUG_COND(cond, markers)
#define SV_SHOW_DEBUG_COND_NAMED(name, cond, markers)
#define SV_SHOW_DEBUG_THROTTLE(rate, markers)
#define SV_SHOW_DEBUG_THROTTLE_NAMED(name, rate, markers)
#define SV_SHOW_DEBUG_ONCE(markers)
#define SV_SHOW_DEBUG_ONCE_NAMED(name, markers)
#else
#define SV_SHOW_DEBUG(markers) SV_SHOW(::sbpl::visual::Level::Debug, SV_NAME_PREFIX, markers)
#define SV_SHOW_DEBUG_NAMED(name, markers) SV_SHOW(::sbpl::visual::Level::Debug, std::string(SV_NAME_PREFIX) + "." + name, markers)
#define SV_SHOW_DEBUG_COND(cond, markers) SV_SHOW_COND(cond, ::sbpl::visual::Level::Debug, SV_NAME_PREFIX, markers)
#define SV_SHOW_DEBUG_COND_NAMED(name, cond, markers) SV_SHOW_COND(cond, ::sbpl::visual::Level::Debug, std::string(SV_NAME_PREFIX) + "." + name, markers)
#define SV_SHOW_DEBUG_THROTTLE(rate, markers) SV_SHOW_THROTTLE(rate, ::sbpl::visual::Level::Debug, SV_NAME_PREFIX, markers)
#define SV_SHOW_DEBUG_THROTTLE_NAMED(name, rate, markers) SV_SHOW_THROTTLE(rate, ::sbpl::visual::Level::Debug, std::string(SV_NAME_PREFIX) + "." + name, markers)
#define SV_SHOW_DEBUG_ONCE(markers) SV_SHOW_ONCE(::sbpl::visual::Level::Debug, SV_NAME_PREFIX, markers)
#define SV_SHOW_DEBUG_ONCE_NAMED(name, markers) SV_SHOW_ONCE(::sbpl::visual::Level::Debug, std::string(SV_NAME_PREFIX) + "." + name, markers)
#endif

#if (SBPL_VISUALIZE_MIN_SEVERITY > SBPL_VISUALIZE_SEVERITY_INFO)
#define SV_SHOW_INFO(markers)
#define SV_SHOW_INFO_NAMED(name, markers)
#define SV_SHOW_INFO_COND(cond, markers)
#define SV_SHOW_INFO_COND_NAMED(name, cond, markers)
#define SV_SHOW_INFO_THROTTLE(rate, markers)
#define SV_SHOW_INFO_THROTTLE_NAMED(name, rate, markers)
#define SV_SHOW_INFO_ONCE(markers)
#define SV_SHOW_INFO_ONCE_NAMED(name, markers)
#else
#define SV_SHOW_INFO(markers) SV_SHOW(::sbpl::visual::Level::Info, SV_NAME_PREFIX, markers)
#define SV_SHOW_INFO_NAMED(name, markers) SV_SHOW(::sbpl::visual::Level::Info, std::string(SV_NAME_PREFIX) + "." + name, markers)
#define SV_SHOW_INFO_COND(cond, markers) SV_SHOW_COND(cond, ::sbpl::visual::Level::Info, SV_NAME_PREFIX, markers)
#define SV_SHOW_INFO_COND_NAMED(name, cond, markers) SV_SHOW_COND(cond, ::sbpl::visual::Level::Info, std::string(SV_NAME_PREFIX) + "." + name, markers)
#define SV_SHOW_INFO_THROTTLE(rate, markers) SV_SHOW_THROTTLE(rate, ::sbpl::visual::Level::Info, SV_NAME_PREFIX, markers)
#define SV_SHOW_INFO_THROTTLE_NAMED(name, rate, markers) SV_SHOW_THROTTLE(rate, ::sbpl::visual::Level::Info, std::string(SV_NAME_PREFIX) + "." + name, markers)
#define SV_SHOW_INFO_ONCE(markers) SV_SHOW_ONCE(::sbpl::visual::Level::Info, SV_NAME_PREFIX, markers)
#define SV_SHOW_INFO_ONCE_NAMED(name, markers) SV_SHOW_ONCE(::sbpl::visual::Level::Info, std::string(SV_NAME_PREFIX) + "." + name, markers)
#endif

#if (SBPL_VISUALIZE_MIN_SEVERITY > SBPL_VISUALIZE_SEVERITY_WARN)
#define SV_SHOW_WARN(markers)
#define SV_SHOW_WARN_NAMED(name, markers)
#define SV_SHOW_WARN_COND(cond, markers)
#define SV_SHOW_WARN_COND_NAMED(name, cond, markers)
#define SV_SHOW_WARN_THROTTLE(rate, markers)
#define SV_SHOW_WARN_THROTTLE_NAMED(name, rate, markers)
#define SV_SHOW_WARN_ONCE(markers)
#define SV_SHOW_WARN_ONCE_NAMED(name, markers)
#else
#define SV_SHOW_WARN(markers) SV_SHOW(::sbpl::visual::Level::Warn, SV_NAME_PREFIX, markers)
#define SV_SHOW_WARN_NAMED(name, markers) SV_SHOW(::sbpl::visual::Level::Warn, std::string(SV_NAME_PREFIX) + "." + name, markers)
#define SV_SHOW_WARN_COND(cond, markers) SV_SHOW_COND(cond, ::sbpl::visual::Level::Warn, SV_NAME_PREFIX, markers)
#define SV_SHOW_WARN_COND_NAMED(name, cond, markers) SV_SHOW_COND(cond, ::sbpl::visual::Level::Warn, std::string(SV_NAME_PREFIX) + "." + name, markers)
#define SV_SHOW_WARN_THROTTLE(rate, markers) SV_SHOW_THROTTLE(rate, ::sbpl::visual::Level::Warn, SV_NAME_PREFIX, markers)
#define SV_SHOW_WARN_THROTTLE_NAMED(name, rate, markers) SV_SHOW_THROTTLE(rate, ::sbpl::visual::Level::Warn, std::string(SV_NAME_PREFIX) + "." + name, markers)
#define SV_SHOW_WARN_ONCE(markers) SV_SHOW_ONCE(::sbpl::visual::Level::Warn, SV_NAME_PREFIX, markers)
#define SV_SHOW_WARN_ONCE_NAMED(name, markers) SV_SHOW_ONCE(::sbpl::visual::Level::Warn, std::string(SV_NAME_PREFIX) + "." + name, markers)
#endif

#if (SBPL_VISUALIZE_MIN_SEVERITY > SBPL_VISUALIZE_SEVERITY_ERROR)
#define SV_SHOW_ERROR(markers)
#define SV_SHOW_ERROR_NAMED(name, markers)
#define SV_SHOW_ERROR_COND(cond, markers)
#define SV_SHOW_ERROR_COND_NAMED(name, cond, markers)
#define SV_SHOW_ERROR_THROTTLE(rate, markers)
#define SV_SHOW_ERROR_THROTTLE_NAMED(name, rate, markers)
#define SV_SHOW_ERROR_ONCE(markers)
#define SV_SHOW_ERROR_ONCE_NAMED(name, markers)
#else
#define SV_SHOW_ERROR(markers) SV_SHOW(::sbpl::visual::Level::Error, SV_NAME_PREFIX, markers)
#define SV_SHOW_ERROR_NAMED(name, markers) SV_SHOW(::sbpl::visual::Level::Error, std::string(SV_NAME_PREFIX) + "." + name, markers)
#define SV_SHOW_ERROR_COND(cond, markers) SV_SHOW_COND(cond, ::sbpl::visual::Level::Error, SV_NAME_PREFIX, markers)
#define SV_SHOW_ERROR_COND_NAMED(name, cond, markers) SV_SHOW_COND(cond, ::sbpl::visual::Level::Error, std::string(SV_NAME_PREFIX) + "." + name, markers)
#define SV_SHOW_ERROR_THROTTLE(rate, markers) SV_SHOW_THROTTLE(rate, ::sbpl::visual::Level::Error, SV_NAME_PREFIX, markers)
#define SV_SHOW_ERROR_THROTTLE_NAMED(name, rate, markers) SV_SHOW_THROTTLE(rate, ::sbpl::visual::Level::Error, std::string(SV_NAME_PREFIX) + "." + name, markers)
#define SV_SHOW_ERROR_ONCE(markers) SV_SHOW_ONCE(::sbpl::visual::Level::Error, SV_NAME_PREFIX, markers)
#define SV_SHOW_ERROR_ONCE_NAMED(name, markers) SV_SHOW_ONCE(::sbpl::visual::Level::Error, std::string(SV_NAME_PREFIX) + "." + name, markers)
#endif

#if (SBPL_VISUALIZE_MIN_SEVERITY > SBPL_VISUALIZE_SEVERITY_FATAL)
#define SV_SHOW_FATAL(markers)
#define SV_SHOW_FATAL_NAMED(name, markers)
#define SV_SHOW_FATAL_COND(cond, markers)
#define SV_SHOW_FATAL_COND_NAMED(name, cond, markers)
#define SV_SHOW_FATAL_THROTTLE(rate, markers)
#define SV_SHOW_FATAL_THROTTLE_NAMED(name, rate, markers)
#define SV_SHOW_FATAL_ONCE(markers)
#define SV_SHOW_FATAL_ONCE_NAMED(name, markers)
#else
#define SV_SHOW_FATAL(markers) SV_SHOW(::sbpl::visual::Level::Fatal, SV_NAME_PREFIX, markers)
#define SV_SHOW_FATAL_NAMED(name, markers) SV_SHOW(::sbpl::visual::Level::Fatal, std::string(SV_NAME_PREFIX) + "." + name, markers)
#define SV_SHOW_FATAL_COND(cond, markers) SV_SHOW_COND(cond, ::sbpl::visual::Level::Fatal, SV_NAME_PREFIX, markers)
#define SV_SHOW_FATAL_COND_NAMED(name, cond, markers) SV_SHOW_COND(cond, ::sbpl::visual::Level::Fatal, std::string(SV_NAME_PREFIX) + "." + name, markers)
#define SV_SHOW_FATAL_THROTTLE(rate, markers) SV_SHOW_THROTTLE(rate, ::sbpl::visual::Level::Fatal, SV_NAME_PREFIX, markers)
#define SV_SHOW_FATAL_THROTTLE_NAMED(name, rate, markers) SV_SHOW_THROTTLE(rate, ::sbpl::visual::Level::Fatal, std::string(SV_NAME_PREFIX) + "." + name, markers)
#define SV_SHOW_FATAL_ONCE(markers) SV_SHOW_ONCE(::sbpl::visual::Level::Fatal, SV_NAME_PREFIX, markers)
#define SV_SHOW_FATAL_ONCE_NAMED(name, markers) SV_SHOW_ONCE(::sbpl::visual::Level::Fatal, std::string(SV_NAME_PREFIX) + "." + name, markers)
#endif

#endif
