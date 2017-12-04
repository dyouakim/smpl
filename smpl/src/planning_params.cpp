////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2010, Benjamin Cohen, Andrew Dornbush
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

/// \author Benjamin Cohen
/// \author Andrew Dornbush

#include <smpl/planning_params.h>

// standard includes
#include <sstream>

// system includes
#include <smpl/console/console.h>

namespace sbpl {
namespace motion {

const std::string PlanningParams::DefaultRobotModelLog = "robot";
const std::string PlanningParams::DefaultGraphLog = "graph";
const std::string PlanningParams::DefaultHeuristicLog = "heuristic";
const std::string PlanningParams::DefaultExpandsLog = "graph.expansions";
const std::string PlanningParams::DefaultPostProcessingLog = "post_process";
const std::string PlanningParams::DefaultSolutionLog = "solution";
const std::string PlanningParams::DefaultSuccessorsLog = "successors";

std::string to_string(ShortcutType type)
{
    switch (type) {
    case ShortcutType::INVALID_SHORTCUT_TYPE:
        return "INVALID_SHORTCUT_TYPE";
    case ShortcutType::JOINT_SPACE:
        return "JOINT_SPACE";
    case ShortcutType::JOINT_POSITION_VELOCITY_SPACE:
        return "JOINT_POSITION_VELOCITY_SPACE";
    case ShortcutType::EUCLID_SPACE:
        return "EUCLID_SPACE";
    default:
        return "UNRECOGNIZED_SHORTCUT_TYPE";
    }
}

PlanningParams::PlanningParams() :
    planning_frame(),

    cost_per_cell(DefaultCostPerCell),

    planning_link_sphere_radius(DefaultPlanningLinkSphereRadius),

    shortcut_path(DefaultShortcutPath),
    interpolate_path(DefaultInterpolatePath),
    shortcut_type(DefaultShortcutType),

    robot_log(DefaultRobotModelLog),
    graph_log(DefaultGraphLog),
    heuristic_log(DefaultHeuristicLog),
    expands_log(DefaultExpandsLog),
    successors_log(DefaultSuccessorsLog),
    post_processing_log(DefaultPostProcessingLog),
    solution_log(DefaultSolutionLog),

    m_warn_defaults(false)
{
}

template <typename T>
std::string construct_warn_string(const std::string& name, T def)
{
    std::stringstream ss;
    ss << "Missing parameter '" << name << "'. Default set to " << def;
    return ss.str();
}

void PlanningParams::addParam(const std::string& name, bool val)
{
    params[name] = val;
}

void PlanningParams::addParam(const std::string& name, int val)
{
    params[name] = val;
}

void PlanningParams::addParam(const std::string& name, double val)
{
    params[name] = val;
}

void PlanningParams::addParam(const std::string& name, const std::string& val)
{
    params[name] = val;
}

void PlanningParams::param(const std::string& name, bool& val, bool def) const
{
    auto it = params.find(name);
    if (it == params.end()) {
        val = def;
        if (m_warn_defaults) {
            SMPL_WARN("%s", construct_warn_string(name, def).c_str());
        }
        return;
    }

    convertToBool(it->second, val);
}

void PlanningParams::param(const std::string& name, int& val, int def) const
{
    auto it = params.find(name);
    if (it == params.end()) {
        val = def;
        if (m_warn_defaults) {
            SMPL_WARN("%s", construct_warn_string(name, def).c_str());
        }
        return;
    }

    convertToInt(it->second, val);
}

void PlanningParams::param(const std::string& name, double& val, double def) const
{
    auto it = params.find(name);
    if (it == params.end()) {
        val = def;
        if (m_warn_defaults) {
            SMPL_WARN("%s", construct_warn_string(name, def).c_str());
        }
        return;
    }

    convertToDouble(it->second, val);
}

void PlanningParams::param(
    const std::string& name,
    std::string& val,
    const std::string& def) const
{
    auto it = params.find(name);
    if (it == params.end()) {
        val = def;
        if (m_warn_defaults) {
            SMPL_WARN("%s", construct_warn_string(name, def).c_str());
        }
        return;
    }

    convertToString(it->second, val);
}

bool PlanningParams::getParam(const std::string& name, bool& val) const
{
    auto it = params.find(name);
    if (it == params.end()) {
        return false;
    }

    convertToBool(it->second, val);
    return true;
}

bool PlanningParams::getParam(const std::string& name, int& val) const
{
    auto it = params.find(name);
    if (it == params.end()) {
        return false;
    }

    convertToInt(it->second, val);
    return true;
}

bool PlanningParams::getParam(const std::string& name, double& val) const
{
    auto it = params.find(name);
    if (it == params.end()) {
        return false;
    }

    convertToDouble(it->second, val);
    return true;
}

bool PlanningParams::getParam(const std::string& name, std::string& val) const
{
    auto it = params.find(name);
    if (it == params.end()) {
        return false;
    }

    convertToString(it->second, val);
    return true;
}

bool PlanningParams::hasParam(const std::string& name) const
{
    auto it = params.find(name);
    return it != params.end();
}

void PlanningParams::convertToBool(const Parameter& p, bool& val) const
{
    struct bool_converter : public boost::static_visitor<bool> {
        bool operator()(bool val) const { return (bool)val; }
        bool operator()(double val) const { return (bool)val; }
        bool operator()(int val) const { return (bool)val; }
        bool operator()(const std::string& val) const { return val == "true"; }
    };

    val = boost::apply_visitor(bool_converter(), p);
}

void PlanningParams::convertToInt(const Parameter& p, int& val) const
{
    struct int_converter : public boost::static_visitor<int> {
        int operator()(bool val) const { return (int)val; }
        int operator()(double val) const { return (int)val; }
        int operator()(int val) const { return (int)val; }
        int operator()(const std::string& val) const { return std::stoi(val); }
    };
    val = boost::apply_visitor(int_converter(), p);
}

void PlanningParams::convertToDouble(const Parameter& p, double& val) const
{
    struct double_converter : public boost::static_visitor<double> {
        double operator()(bool val) const { return (double)val; }
        double operator()(double val) const { return (double)val; }
        double operator()(int val) const { return (double)val; }
        double operator()(const std::string& val) const { return std::stod(val); }
    };

    val = boost::apply_visitor(double_converter(), p);
}

void PlanningParams::convertToString(const Parameter& p, std::string& val) const
{
    struct string_converter : public boost::static_visitor<std::string> {
        std::string operator()(bool val) const { return val ? "true" : "false"; }
        std::string operator()(double val) const { return std::to_string(val); }
        std::string operator()(int val) const { return std::to_string(val); }
        const std::string& operator()(const std::string& val) const { return val; }
    };

    val = boost::apply_visitor(string_converter(), p);
}

} // namespace motion
} // namespace sbpl
