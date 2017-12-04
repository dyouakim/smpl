#include <smpl/debug/visualizer_ros.h>

// standard includes
#include <sstream>

#include <smpl/debug/marker_conversions.h>

namespace sbpl {

VisualizerROS::VisualizerROS(
    const ros::NodeHandle& nh,
    size_t queue_size)
:
    m_nh(nh)
{
    m_pub = ros::NodeHandle().advertise<visualization_msgs::MarkerArray>(
            "visualization_markers", queue_size);

    std::stringstream disabled_regex;
    bool empty = true;

    XmlRpc::XmlRpcValue disabled_value;
    if (nh.getParam("disabled_namespaces", disabled_value)) {
        if (disabled_value.getType() == XmlRpc::XmlRpcValue::TypeArray) {
            for (int i = 0; i < disabled_value.size(); ++i) {
                XmlRpc::XmlRpcValue& e = disabled_value[i];
                if (e.getType() != XmlRpc::XmlRpcValue::TypeString) {
                    ROS_WARN("'disabled_namespaces' element is not a string. skipping.");
                    continue;
                }

                if (!empty) {
                    disabled_regex << '|';
                }

                disabled_regex << '(' << (std::string&)e << ')';
                empty = false;
            }
        }
        else {
            ROS_WARN("param 'disabled_namespaces' is not a vector");
        }
    }

    m_disabled_regex = boost::regex(disabled_regex.str(), boost::regex_constants::optimize);
}

void VisualizerROS::visualize(visual::Level level, const visual::Marker& marker)
{
    boost::smatch sm;
    if (boost::regex_match(marker.ns, sm, m_disabled_regex)) {
        return;
    }

    m_enabled.markers.clear();
    m_enabled.markers.resize(1);
    visual::ConvertMarkerToMarkerMsg(marker, m_enabled.markers.back());
    m_enabled.markers.back().header.stamp = ros::Time::now();
    m_pub.publish(m_enabled);
}

void VisualizerROS::visualize(
    visual::Level level,
    const std::vector<visual::Marker>& markers)
{
    size_t exclude_count = 0;

    m_excluded.assign(markers.size(), false);

    boost::smatch sm;
    for (size_t i = 0; i < markers.size(); ++i) {
        auto& marker = markers[i];
        if (boost::regex_match(marker.ns, sm, m_disabled_regex)) {
            m_excluded[i] = true;
            ++exclude_count;
        }
    }

    auto now = ros::Time::now();

    if (exclude_count == 0) {
        m_enabled.markers.resize(markers.size());
        for (size_t i = 0; i < markers.size(); ++i) {
            auto& marker = markers[i];
            auto& marker_msg = m_enabled.markers[i];
            visual::ConvertMarkerToMarkerMsg(marker, marker_msg);
            marker_msg.header.stamp = now;
        }

        m_pub.publish(m_enabled);
    } else if (exclude_count == markers.size()) {
        return;
    } else {
        m_enabled.markers.resize(markers.size() - exclude_count);
        size_t j = 0;
        for (size_t i = 0; i < markers.size(); ++i) {
            auto& marker = markers[i];
            if (!m_excluded[i]) {
                visual::ConvertMarkerToMarkerMsg(marker, m_enabled.markers[j]);
                m_enabled.markers[j].header.stamp = now;
                j++;
            }
        }
        m_pub.publish(m_enabled);
    }
}

void VisualizerROS::visualize(
    visual::Level level,
    const visualization_msgs::Marker& marker)
{
    boost::smatch sm;
    if (boost::regex_match(marker.ns, sm, m_disabled_regex)) {
        return;
    }

    m_enabled.markers.clear();
    m_enabled.markers.push_back(marker);
    m_pub.publish(m_enabled);
}

void VisualizerROS::visualize(
    visual::Level level,
    const visualization_msgs::MarkerArray& markers)
{
    size_t exclude_count = 0;
    m_excluded.assign(markers.markers.size(), false);

    boost::smatch sm;
    for (size_t i = 0; i < markers.markers.size(); ++i) {
        auto& marker = markers.markers[i];
        if (boost::regex_match(marker.ns, sm, m_disabled_regex)) {
            m_excluded[i] = true;
            ++exclude_count;
        }
    }

    if (exclude_count == 0) {
        // include all markers
        m_pub.publish(markers);
    }
    else if (exclude_count == markers.markers.size()) {
        // exclude all markers
        return;
    }
    else {
        m_enabled.markers.clear();
        for (size_t i = 0; i < markers.markers.size(); ++i) {
            auto& marker = markers.markers[i];
            if (!m_excluded[i]) {
                m_enabled.markers.push_back(marker);
            }
        }
        m_pub.publish(m_enabled);
    }
}

} // namespace sbpl
