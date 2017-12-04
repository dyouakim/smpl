#ifndef SMPL_MARKER_CONVERSIONS_H
#define SMPL_MARKER_CONVERSIONS_H

// system includes
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// project includes
#include <smpl/debug/marker.h>

namespace sbpl {
namespace visual {

void ConvertMarkerMsgToMarker(
    const visualization_msgs::Marker& mm,
    Marker& m);

void ConvertMarkerToMarkerMsg(
    const Marker& m,
    visualization_msgs::Marker& mm);

} // namespace visual
} // namespace sbpl

#endif
